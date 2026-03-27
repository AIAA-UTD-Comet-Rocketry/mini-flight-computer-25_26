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
#include "esp_timer.h"

#define MIN_STACK_SIZE configMINIMAL_STACK_SIZE * 2 // original minimum causes stack overflow
#define SENSOR_DELAY_MS 100 // 1/100ms = 10Hz

// Uncomment to run pyro bench test on boot (DO NOT fly with this enabled)
//#define PYRO_BENCH_TEST

static const char *TAG = "Main";
static imu_cal_t imu_cal;
static FlightState flight_state;

// Tasks
void vImuHandlerTask(void *pvParameters);
void vMagHandlerTask(void *pvParameters);
void vAltHandlerTask(void *pvParameters);
void vSdLoggerTask(void *pvParameters);
void vFsmTask(void *pvParameters);

static QueueHandle_t imu_queue = NULL, alt_queue = NULL, mag_queue = NULL;

SemaphoreHandle_t xSemaphore;

TaskHandle_t xPyroTaskHandle = NULL;

// Macros
#define CHECK_TASK_CREATION(ret, err_msg) \
    if((ret) != pdPASS) { \
        ESP_LOGI(TAG, err_msg); \
    }

void app_main(void) {
    (void)TAG; // Stop compile warnings, unused debug variables are not a concern

    ///////////////////////////////
    /// Configuration
    ////////////////////////////////
    static board_handle_t mini_fc_handle;

    bsp_config_t bsp_init_cfg = {
        .Bsp_GetTick = xTaskGetTickCount,
        .Bsp_Delay = vTaskDelay
    };

    //ESP_ERROR_CHECK(bsp_init(&bsp_init_cfg));
    bsp_init(&mini_fc_handle, &bsp_init_cfg);

    // IMU Calibration (blocking, ~5 seconds)
    imu_calibrate(mini_fc_handle->lsm6dsv80x_handle, &imu_cal);

    // Ground Pressure Reference (average samples for stability)
    float ground_pressure = 0;
    float sample;
    int SAMPLE_NUM = 100;

    for (int i = 0; i < SAMPLE_NUM; i++) {
        LPS22DF_PRESS_GetPressure(mini_fc_handle->lps22df_handle, &sample);
        ground_pressure += sample;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ground_pressure /= (SAMPLE_NUM * 1.0f);
    sensor_set_ground_pressure(ground_pressure);

    /// Flight State Machine
    initFlightState(&flight_state);

    /// SD Card Logger
    sd_logger_init(); // non-fatal if SD card is absent
    
    // Queue inits
    imu_queue = xQueueCreate(10, sizeof(imu_calibrated_t));
    alt_queue = xQueueCreate(10, sizeof(AltData_t));
    mag_queue = xQueueCreate(10, sizeof(MagData_t));

    // Semaphore init
    xSemaphore = xSemaphoreCreateMutex();

    ///////////////////////////////
    /// Task Creation
    ////////////////////////////////
    BaseType_t task_ret;
    TaskHandle_t xImuTaskHandle, xMagTaskHandle, xAltTaskHandle;

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
    // IMU
    task_ret = xTaskCreate(vImuHandlerTask, 
                           "IMU Data Collection", 
                           MIN_STACK_SIZE, 
                           (void*) mini_fc_handle->lsm6dsv80x_handle, 
                           1, 
                           &xImuTaskHandle);
    CHECK_TASK_CREATION(task_ret, "IMU task failed to create!");
    // Magnetometer
    task_ret = xTaskCreate(vMagHandlerTask,
                           "Magnetometer Data Collection",
                           MIN_STACK_SIZE,
                           (void*) mini_fc_handle->iis2mdc_handle,
                           1,
                           &xMagTaskHandle);
    CHECK_TASK_CREATION(task_ret, "Magnetometer task failed to create!");
    // Pressure
    task_ret = xTaskCreate(vAltHandlerTask,
                           "Absolute Pressure Data Collection",
                           MIN_STACK_SIZE,
                           (void*) mini_fc_handle->lps22df_handle,
                           1,
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

void vImuHandlerTask(void *pvParameters) {
    LSM6DSV80X_Object_t* imu = (LSM6DSV80X_Object_t*)pvParameters;
    LSM6DSV80X_Axes_t accel_axes, gyro_axes;
    imu_calibrated_t cal_data;

    int imu_print_counter = 0;
    while(1) {
        LSM6DSV80X_ACC_GetAxes(imu, &accel_axes);
        LSM6DSV80X_GYRO_GetAxes(imu, &gyro_axes);

        // Apply calibration and convert units
        imu_apply_calibration(&imu_cal, &accel_axes, &gyro_axes, &cal_data);

        // Update shared flight data for FSM (also populates gAccel/gGyro for SD logger)
        sensor_update_flight_data(&cal_data);

        if(++imu_print_counter >= 10) {
            ESP_LOGI("IMU", "Accel (g): X=%.3f Y=%.3f Z=%.3f",
                     cal_data.accel_g[0], cal_data.accel_g[1], cal_data.accel_g[2]);
            ESP_LOGI("IMU", "Gyro (dps): X=%.3f Y=%.3f Z=%.3f",
                     cal_data.gyro_dps[0], cal_data.gyro_dps[1], cal_data.gyro_dps[2]);
            imu_print_counter = 0;
        }
        if(xQueueSend(imu_queue, (void*) &cal_data, pdMS_TO_TICKS(10)) != pdPASS) {
            ESP_LOGE(TAG, "Failed to send IMU data to queue");
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY_MS));
    }
}
void vMagHandlerTask(void *pvParameters) {
    IIS2MDC_Object_t* mag = (IIS2MDC_Object_t*)pvParameters;
    IIS2MDC_Axes_t next_axis;
    MagData_t mag_data;
    while(1) {
        IIS2MDC_MAG_GetAxes(mag, &next_axis);
        mag_data = sensor_update_mag(next_axis);

        // ESP_LOGI("MAG", "Mag X: %ld, Mag Y: %ld, Mag Z: %ld", next_axis.x, next_axis.y, next_axis.z);
        if(xQueueSend(mag_queue, (void*) &mag_data, pdMS_TO_TICKS(10)) != pdPASS) {
            ESP_LOGE(TAG, "Failed to send Mag data to queue");
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
    int alt_print_counter = 0;
    while(1) {
        LPS22DF_PRESS_GetPressure(alt, &alt_data.pressure);
        LPS22DF_TEMP_GetTemperature(alt, &alt_data.temp);
        if (alt_data.pressure || alt_data.temp != LPS22DF_ERROR) {
            alt_data.altitude = sensor_update_altitude(alt_data.pressure, alt_data.temp);
        }
        else {
            ESP_LOGE("PRESS", "Failed to obtain Altitude data");
            continue;
        }

        if(++alt_print_counter >= 10) {
            ESP_LOGI("PRESS", "Pressure (hpa): %f, Altitude (ft): %f", alt_data.pressure, alt_data.altitude);
            alt_print_counter = 0;
        }
        // Encapsulate data
        if(xQueueSend(alt_queue, (void*) &alt_data, pdMS_TO_TICKS(10)) != pdPASS) {

        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY_MS));
    }
}


void vSdLoggerTask(void *pvParameters) {
    SensorDataPacket_t packet;
    int write_counter = 0;
    while (1) {
        xQueueReceive(imu_queue, &packet.imu, portMAX_DELAY);
        xQueueReceive(alt_queue, &packet.alt, portMAX_DELAY);
        xQueueReceive(mag_queue, &packet.mag, portMAX_DELAY);
        if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            write_log(packet, &write_counter);
            xSemaphoreGive(xSemaphore);
        }
    }
}
