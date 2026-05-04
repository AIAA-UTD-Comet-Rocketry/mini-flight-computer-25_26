/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "BSP.h"
#include "gpio_mgr.h"
#include "sd_logger.h"
#include "sensor_mgr.h"
#include "FlightFSM.h"
#include "attitude_ekf.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#define MIN_STACK_SIZE configMINIMAL_STACK_SIZE * 2 // original minimum causes stack overflow
#define SENSOR_DELAY_MS 10 // 1/10ms = 100Hz
#define LOGGING_DELAY_MS 100 // ms

// EKF on-pad calibration phase: 5 s at 100 Hz
#define EKF_CAL_CYCLES 500
// IMU task queues to SD logger at 10 Hz (every 10th cycle of 100 Hz IMU loop)
#define IMU_QUEUE_DECIMATE 10

// Uncomment to run pyro bench test on boot (DO NOT fly with this enabled)
//#define PYRO_BENCH_TEST

// Uncomment, flash, perform figure-8 rotation, observe save log, halt.
// Recomment and re-flash for normal flight.
//#define DO_MAG_CAL

// TODO: CAN Aerospace integration with telemetry data

static const char *TAG = "Main";
static imu_cal_t imu_cal;
static mag_cal_t mag_cal;
static FlightState flight_state;

static void init_nvs_flash_memory(void);

// RTOS Tasks
void vImuHandlerTask(void *pvParameters);
void vAltHandlerTask(void *pvParameters);
void vSdLoggerTask(void *pvParameters);
void vFsmTask(void *pvParameters);

static QueueHandle_t imu_queue = NULL, alt_queue = NULL, sensor_queue = NULL;
SemaphoreHandle_t xSemaphore;
TaskHandle_t xPyroTaskHandle = NULL;

// Macros
#define CHECK_TASK_CREATION(ret, err_msg) \
    if((ret) != pdPASS) { \
        ESP_LOGI(TAG, err_msg); \
    }

void app_main(void) {
    (void)TAG; // Stop compile warnings, unused debug variables are not a concern

    // Handle config
    static board_handle_t mini_fc_handle;

    bsp_config_t bsp_init_cfg = {
        .Bsp_GetTick = xTaskGetTickCount,
        .Bsp_Delay = vTaskDelay
    };

    if (bsp_init(&mini_fc_handle, &bsp_init_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "BSP init failed. Stopping program!");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // Initialize NVS storage
    init_nvs_flash_memory();

#ifdef DO_MAG_CAL
    // Bench-only mode: rotate the board through all orientations during sampling.
    // Result is persisted to NVS and reused by every subsequent flight boot.
    LED_setPattern(led_status, pattern_fast_blink);
    ESP_LOGW(TAG, "=== MAG CALIBRATION MODE ===");
    if (mag_calibrate(mini_fc_handle->iis2mdc_handle, &mag_cal) == ESP_OK) {
        if (mag_cal_save_nvs(&mag_cal) == ESP_OK) {
            ESP_LOGW(TAG, "Mag cal saved to NVS. Recomment DO_MAG_CAL and re-flash.");
            LED_setPattern(led_status, pattern_on);
        } else {
            ESP_LOGE(TAG, "Mag cal NVS save failed.");
            LED_setPattern(led_status, pattern_off);
        }
    } else {
        ESP_LOGE(TAG, "Mag cal collection failed.");
        LED_setPattern(led_status, pattern_off);
    }
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
#endif

    // Load mag cal from NVS (persisted from a previous bench session).
    if (mag_cal_load_nvs(&mag_cal) != ESP_OK || !mag_cal.is_calibrated) {
        ESP_LOGW(TAG, "Mag cal not in NVS. Flight will run with raw mag; EKF will absorb slowly.");
        memset(&mag_cal, 0, sizeof(mag_cal));
        // identity soft-iron so sensor_update_mag passthrough is reasonable
        mag_cal.soft_iron[0][0] = 1.0f;
        mag_cal.soft_iron[1][1] = 1.0f;
        mag_cal.soft_iron[2][2] = 1.0f;
    } else {
        ESP_LOGI(TAG, "Mag cal loaded from NVS.");
    }

    // Gyro bias calibration (blocking, ~5 seconds)
    imu_calibrate(mini_fc_handle->lsm6dsv80x_handle, &imu_cal);

    // Sample ambient pressure for ~1 s and use as the altitude=0 reference
    if (baro_calibrate_ground(mini_fc_handle->lps22df_handle) != ESP_OK) {
        ESP_LOGW(TAG, "Ground pressure cal failed, falling back to sea-level default.");
    }

    // Bring up the EKF and seed gyro bias (mdps -> dps)
    attitude_ekf_init();
    float gyro_seed_dps[3] = {
        imu_cal.gyro_bias_mdps[0] / 1000.0f,
        imu_cal.gyro_bias_mdps[1] / 1000.0f,
        imu_cal.gyro_bias_mdps[2] / 1000.0f,
    };
    attitude_ekf_seed_gyro_bias_dps(gyro_seed_dps);

    /// Flight State Machine
    initFlightState(&flight_state);

    /// SD Card Logger
    sd_logger_init(); // non-fatal if SD card is absent
    
    // Queue inits
    imu_queue = xQueueCreate(10, sizeof(imu_calibrated_t));
    alt_queue = xQueueCreate(10, sizeof(AltData_t));
    sensor_queue = xQueueCreate(20, sizeof(SensorMessage_t));

    // Semaphore init
    xSemaphore = xSemaphoreCreateMutex();

    // RTOS Task creation
    BaseType_t task_ret;
    TaskHandle_t xImuTaskHandle, xAltTaskHandle;

    // SD Logger
    {
        TaskHandle_t xSdLoggerHandle;
        task_ret = xTaskCreate(vSdLoggerTask,
                               "SD Logger",
                               4096,
                               NULL,
                               2,
                               &xSdLoggerHandle);
        CHECK_TASK_CREATION(task_ret, "SD Logger task failed to create!");
    }
    // IMU + EKF + Mag (all sensor fusion in one 100 Hz task).
    // EKF allocates several dspm::Mat scratch matrices per Process()/Update*()
    // call; 8 KB gives comfortable headroom over the ~3 KB peak observed.
    task_ret = xTaskCreate(vImuHandlerTask,
                           "IMU+EKF",
                           8192,
                           (void*) mini_fc_handle,
                           2,
                           &xImuTaskHandle);
    CHECK_TASK_CREATION(task_ret, "IMU task failed to create!");
    // Pressure
    task_ret = xTaskCreate(vAltHandlerTask,
                           "Absolute Pressure Data Collection",
                           MIN_STACK_SIZE,
                           (void*) mini_fc_handle->lps22df_handle,
                           2,
                           &xAltTaskHandle);
    // Flight State Machine
    TaskHandle_t xFsmTaskHandle;
    task_ret = xTaskCreate(vFsmTask,
                           "Flight FSM",
                           4096,
                           NULL,
                           2,  // higher priority than sensor tasks
                           &xFsmTaskHandle);
    CHECK_TASK_CREATION(task_ret, "FSM task failed to create!");
    // LED
    xTaskCreate((TaskFunction_t)LED_Task, "LED MGR", 4096, (void *)&mini_fc_handle, 0, NULL);
    // Pyro
    xTaskCreate((TaskFunction_t)Pyro_Task, "PYRO MGR", 4096, (void *)&mini_fc_handle, 4, &xPyroTaskHandle);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for everything to settle (TODO event based wait)

    // drive led_status with pattern
    LED_setPattern(led_status, pattern_burst);

    #ifdef PYRO_BENCH_TEST
        // Bench test: fire each pyro channel one at a time with 3s gaps
        // Monitor GPIO 6,7,8,9 with multimeter/LED — DO NOT connect real charges
        ESP_LOGW(TAG, "=== PYRO BENCH TEST MODE ===");
        ESP_LOGW(TAG, "Firing channels in 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000));

        const char *channel_names[] = {"APO1 (35g CO2)", "APO2 (45g CO2)", "MAIN1 (TD2)", "MAIN2"};
        for (int ch = 0; ch < 4; ch++) {
            ESP_LOGW(TAG, "Firing channel %d: %s", ch, channel_names[ch]);
            xTaskNotify(xPyroTaskHandle, (1 << ch), eSetBits);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        ESP_LOGW(TAG, "=== PYRO BENCH TEST COMPLETE ===");
        ESP_LOGW(TAG, "Pyro status bitmask: 0x%02X", gPyroStatus);
    #endif
    }
  
/**
 * @brief Initialize NVS flash memory
 */
static void init_nvs_flash_memory(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}

void vImuHandlerTask(void *pvParameters) {
    board_handle_t board = (board_handle_t)pvParameters;
    LSM6DSV80X_Object_t *imu = board->lsm6dsv80x_handle;
    IIS2MDC_Object_t   *mag = board->iis2mdc_handle;

    LSM6DSV80X_Axes_t accel_axes, gyro_axes;
    IIS2MDC_Axes_t mag_axes;
    imu_calibrated_t cal_data;
    SensorMessage_t msg;

    int print_counter = 0;
    int decim_counter = 0;
    int cycle = 0;
    int64_t prev_us = esp_timer_get_time();

    ESP_LOGW(TAG, "EKF calibrating: hold board still for ~5 s...");

    while (1) {
        LSM6DSV80X_ACC_GetAxes(imu, &accel_axes);
        LSM6DSV80X_GYRO_GetAxes(imu, &gyro_axes);
        IIS2MDC_MAG_GetAxes(mag, &mag_axes);

        imu_apply_calibration(&imu_cal, &accel_axes, &gyro_axes, &cal_data);
        mag_apply_calibration(&mag_cal, &mag_axes, &cal_data);
        sensor_update_flight_data(&cal_data);

        int64_t now_us = esp_timer_get_time();
        float dt_s = (float)(now_us - prev_us) * 1e-6f;
        prev_us = now_us;
        if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = SENSOR_DELAY_MS * 0.001f; // fallback on jitter

        if (cycle < EKF_CAL_CYCLES) {
            attitude_ekf_calibrate_step(cal_data.accel_g, cal_data.gyro_dps, cal_data.mag_axes, dt_s);
        } else {
            attitude_ekf_update(cal_data.accel_g, cal_data.gyro_dps, cal_data.mag_axes, dt_s);
            gDegOffVert = attitude_ekf_get_tilt_deg();
        }

        if (cycle == EKF_CAL_CYCLES) {
            float bias_dps[3];
            attitude_ekf_get_gyro_bias_dps(bias_dps);
            ESP_LOGW(TAG, "EKF cal done. Gyro bias (dps): X=%.3f Y=%.3f Z=%.3f",
                     bias_dps[0], bias_dps[1], bias_dps[2]);
        }
        cycle++;

        if (++print_counter >= 100) { // 1 Hz at 100 Hz task rate
            attitude_t att;
            attitude_ekf_get_attitude(&att);
            ESP_LOGI("IMU", "Cal  accel(g)=[%.3f,%.3f,%.3f]  gyro(dps)=[%.3f,%.3f,%.3f]  mag=[%.1f,%.1f,%.1f]",
                     cal_data.accel_g[0], cal_data.accel_g[1], cal_data.accel_g[2],
                     cal_data.gyro_dps[0], cal_data.gyro_dps[1], cal_data.gyro_dps[2],
                     cal_data.mag_axes[0], cal_data.mag_axes[1], cal_data.mag_axes[2]);
            ESP_LOGI("AHRS", "yaw=%+7.2f  pitch=%+6.2f  roll=%+7.2f  tilt=%5.1f  q=[%.3f,%.3f,%.3f,%.3f]",
                     att.yaw_deg, att.pitch_deg, att.roll_deg, att.tilt_deg,
                     att.quat[0], att.quat[1], att.quat[2], att.quat[3]);
            print_counter = 0;
        }

        // Decimate to 10 Hz before pushing to SD logger queue
        if (++decim_counter >= IMU_QUEUE_DECIMATE) {
            msg.type = SENSOR_IMU;
            msg.data.imu = cal_data;

            if (xQueueSend(sensor_queue, (void*) &msg, pdMS_TO_TICKS(10)) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send IMU data to queue");
            }
            decim_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY_MS));
    }
}

void vFsmTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        updateState(&flight_state);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vAltHandlerTask(void *pvParameters) {
    LPS22DF_Object_t* alt = (LPS22DF_Object_t*)pvParameters;
    AltData_t alt_data;
    SensorMessage_t msg;
    int alt_print_counter = 0;
    int decim_counter = 0;

    while(1) {
        LPS22DF_PRESS_GetPressure(alt, &alt_data.pressure);
        LPS22DF_TEMP_GetTemperature(alt, &alt_data.temp);
        if (alt_data.pressure || alt_data.temp != LPS22DF_ERROR) {
            // Only re-zero ground pressure while the rocket is physically on
            // the pad. Apogee/descent free-fall must NOT update the reference.
            if (flight_state.currentState == STATE_IDLE || flight_state.currentState == STATE_ARMED) {
                sensor_track_ground_pressure(alt_data.pressure);
            }
            alt_data.altitude = sensor_get_altitude(alt_data.pressure, alt_data.temp);
        }
        else {
            ESP_LOGE("PRESS", "Failed to obtain Altitude data");
            continue;
        }

        // Convert to F
        alt_data.temp = alt_data.temp * 1.8 + 32.0;

        if(++alt_print_counter >= 100) {
            ESP_LOGI("PRESS", "Pressure (hpa): %.2f, Altitude (ft): %.2f Temp (F): %.2f", alt_data.pressure, alt_data.altitude, alt_data.temp);
            alt_print_counter = 0;
        }
        // Encapsulate data
        // Decimate to 10 Hz before pushing to SD logger queue
        if (++decim_counter >= IMU_QUEUE_DECIMATE) {
            msg.type = SENSOR_ALT;
            msg.data.alt = alt_data;

            if (xQueueSend(sensor_queue, (void*) &msg, pdMS_TO_TICKS(10)) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send Press data to queue");
            }
            decim_counter = 0;
        }
    
        vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY_MS));
    }
}

void vSdLoggerTask(void *pvParameters) {
    //SensorDataPacket_t packet;
    //while (1) {
        // Block task until packet is sent from IMU task
        // if (xQueueReceive(imu_queue, &packet.imu, portMAX_DELAY) == pdTRUE) {
        //     // Check for latest ALT packet, if no then move on with no delay to reduce timing mismatches
        //     xQueueReceive(alt_queue, &packet.alt, 0);

        //     // Use a binary semaphore to lock the write SD card when calling task frequently
        //     if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        //         if (write_packet(packet) == ESP_OK) {
        //             ESP_LOGI(TAG, "Wrote packet to SD.");
        //         }
        //         xSemaphoreGive(xSemaphore);
        //     }
        // }
        //vTaskDelay(pdMS_TO_TICKS(LOGGING_DELAY_MS)); //add delay for every 100ms

    SensorMessage_t msg;
    SensorDataPacket_t packet;

    while (1) {
        // Block task until packet is sent from IMU/Alt task
        if (xQueueReceive(sensor_queue, &msg, portMAX_DELAY)) {
            switch (msg.type) {
                case SENSOR_IMU:
                    packet.imu = msg.data.imu;
                    break;
                case SENSOR_ALT:
                    packet.alt = msg.data.alt;
                    break;
            }

            // Use a mutex to lock the write SD card task when calling frequently
            if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
                if (write_packet(packet) == ESP_OK) {
                    //ESP_LOGI(TAG, "Wrote packet to SD.");
                }
                else {
                    // ESP_LOGE(TAG, "Failed to write packet to SD.");
                }
                xSemaphoreGive(xSemaphore);
            }
        }
    }
    //}
}
