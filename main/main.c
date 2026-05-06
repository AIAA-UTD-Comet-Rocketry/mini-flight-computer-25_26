/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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
#include "can_telemetry.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"

#define UTDMALLOC(n, els)              (els *) malloc((n)*sizeof(els))
#define MIN_STACK_SIZE configMINIMAL_STACK_SIZE // original minimum causes stack overflow
#define SENSOR_DELAY_MS 10 // 1/10ms = 100Hz
#define LOGGING_DELAY_MS 100 // ms (10 Hz SD log cadence)

// EKF on-pad calibration phase: 5 s at 100 Hz
#define EKF_CAL_CYCLES 500

// Uncomment to run pyro bench test on boot (DO NOT fly with this enabled)
//#define PYRO_BENCH_TEST

// Uncomment, flash, perform figure-8 rotation, observe save log, halt.
// Recomment and re-flash for normal flight.
//#define DO_MAG_CAL

static const char *TAG = "Main";
static imu_cal_t imu_cal;
static mag_cal_t mag_cal;
static FlightState flight_state;

static void init_nvs_flash_memory(void);
void load_mag_cal();
void printData();

// RTOS Tasks
void vImuHandlerTask(void *pvParameters);
void vAltHandlerTask(void *pvParameters);
void vSdLoggerTask(void *pvParameters);
void vFsmTask(void *pvParameters);

SemaphoreHandle_t xSemaphore;
TaskHandle_t xPyroTaskHandle = NULL, xLEDTaskHandle = NULL;
TaskHandle_t xSdLoggerHandle = NULL;

FusedPacket_ptr fusedData_p;
// Spinlock guarding writes/reads of *fusedData_p. Held only for struct-field
// copies (microseconds), never across SD I/O.
static portMUX_TYPE g_fused_mux = portMUX_INITIALIZER_UNLOCKED;

static bool sd_logger_started = false;

// Macros
#define CHECK_TASK_CREATION(ret, err_msg) \
    if((ret) != pdPASS) { \
        ESP_LOGI(TAG, err_msg); \
    }

void app_main(void) {
    (void)TAG; // Stop compile warnings, unused debug variables are not a concern

    fusedData_p = UTDMALLOC(1, FusedPacket_t);
    if (fusedData_p == NULL) {
        ESP_LOGE(TAG, "Failed to allocate fused packet buffer.");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
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
    load_mag_cal();

    // Gyro bias calibration (blocking, ~5 seconds)
    imu_calibrate(mini_fc_handle->lsm6dsv80x_handle, &imu_cal);

    // Sample ambient pressure for ~1 s and use as the altitude=0 reference
    if (baro_calibrate_ground(mini_fc_handle->lps22df_handle) != ESP_OK) {
        ESP_LOGW(TAG, "Ground pressure cal failed, falling back to sea-level default.");
        can_telemetry_set_status_bit(CAN_TLM_FLAG_GROUND_PRESS_VALID, false);
    } else {
        can_telemetry_set_status_bit(CAN_TLM_FLAG_GROUND_PRESS_VALID, true);
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
    registerFlightState(&flight_state);  // bind for getCurrentFlightState()

    // CAN telemetry up early so subsequent steps can fire status bits and events.
    can_telemetry_start(*mini_fc_handle->can_node_hdl);
    can_telemetry_event(EVT_BOOT, (uint8_t)esp_reset_reason());

    /// SD Card Logger
    if (sd_logger_init() != ESP_OK) {
        can_telemetry_event(EVT_SD_FAIL, 0);
    }
    
    // Semaphore init (guards FATFS calls in the SD logger task)
    xSemaphore = xSemaphoreCreateMutex();

    // RTOS Task creation
    BaseType_t task_ret;
    TaskHandle_t xImuTaskHandle;
    TaskHandle_t xAltTaskHandle;
    TaskHandle_t xFsmTaskHandle;

    task_ret = xTaskCreate(vSdLoggerTask,
                            "SD Logger",
                            6 * MIN_STACK_SIZE,
                            NULL,
                            1,
                            &xSdLoggerHandle);
    CHECK_TASK_CREATION(task_ret, "SD Logger task failed to create!");

    // Suspend task until EK3 is initialized
    vTaskSuspend( xSdLoggerHandle );

    // IMU + EKF + Mag (all sensor fusion in one 100 Hz task).
    // EKF allocates several dspm::Mat scratch matrices per Process()/Update*()
    // call; 8 KB gives comfortable headroom over the ~3 KB peak observed.
    task_ret = xTaskCreate(vImuHandlerTask,
                           "IMU+EKF",
                           4 * MIN_STACK_SIZE,
                           (void*) mini_fc_handle,
                           2,
                           &xImuTaskHandle);
    CHECK_TASK_CREATION(task_ret, "IMU task failed to create!");
    // Pressure
    task_ret = xTaskCreate(vAltHandlerTask,
                           "Absolute Pressure Data Collection",
                           3 * MIN_STACK_SIZE,
                           (void*) mini_fc_handle->lps22df_handle,
                           2,
                           &xAltTaskHandle);
    // Flight State Machine
    task_ret = xTaskCreate(vFsmTask,
                           "Flight FSM",
                           3 * MIN_STACK_SIZE,
                           NULL,
                           2,  
                           &xFsmTaskHandle);
    CHECK_TASK_CREATION(task_ret, "FSM task failed to create!");

    xTaskCreate((TaskFunction_t)LED_Task, "LED MGR", 2 * MIN_STACK_SIZE, (void *)&mini_fc_handle, 0, &xLEDTaskHandle);
    xTaskCreate((TaskFunction_t)Pyro_Task, "PYRO MGR", 2 * MIN_STACK_SIZE, (void *)&mini_fc_handle, 4, &xPyroTaskHandle);

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

    int cycle = 0;
    int64_t prev_us = esp_timer_get_time();

    ESP_LOGW(TAG, "EKF calibrating: hold board still for ~5 s...");

    while (1) {
        LSM6DSV80X_ACC_GetAxes(imu, &accel_axes);
        LSM6DSV80X_GYRO_GetAxes(imu, &gyro_axes);
        IIS2MDC_MAG_GetAxes(mag, &mag_axes);

        imu_apply_calibration(&imu_cal, &accel_axes, &gyro_axes, &cal_data);
        mag_apply_calibration(&mag_cal, &mag_axes, &cal_data);

        // Remap sensor axes to rocket body frame using compile-time alignment.
        // After this, EKF, FSM, telemetry, and logger all see rocket-frame data.
        // No-op if BOARD_AXIS_ALIGNMENT is the default identity (PXPYPZ).
        sensor_remap_axes(cal_data.accel_g,  BOARD_AXIS_ALIGNMENT, cal_data.accel_g);
        sensor_remap_axes(cal_data.gyro_dps, BOARD_AXIS_ALIGNMENT, cal_data.gyro_dps);
        sensor_remap_axes(cal_data.mag_axes, BOARD_AXIS_ALIGNMENT, cal_data.mag_axes);

        sensor_update_flight_data(&cal_data);

        int64_t now_us = esp_timer_get_time();
        float dt_s = (float)(now_us - prev_us) * 1e-6f;
        prev_us = now_us;
        if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = SENSOR_DELAY_MS * 0.001f; // fallback on jitter

        // EKF is fed PCB-frame accel/gyro/mag — its model assumes the body
        // frame the sensors live in. Mount correction is applied to OUTPUTS
        // afterwards.
        if (cycle < EKF_CAL_CYCLES) {
            attitude_ekf_calibrate_step(cal_data.accel_g, cal_data.gyro_dps, cal_data.mag_axes, dt_s);
        } else {
            attitude_ekf_update(cal_data.accel_g, cal_data.gyro_dps, cal_data.mag_axes, dt_s);
            gDegOffVert = attitude_ekf_get_tilt_deg();
        }

        if (cycle == EKF_CAL_CYCLES && !sd_logger_started) {
            float bias_dps[3];
            attitude_ekf_get_gyro_bias_dps(bias_dps);
            ESP_LOGW(TAG, "EKF cal done. Gyro bias (dps): X=%.3f Y=%.3f Z=%.3f",
                     bias_dps[0], bias_dps[1], bias_dps[2]);
            // Snapshot current EKF quaternion as the PCB-to-rocket mounting
            // offset. From now on, accel/gyro and yaw/pitch/roll readouts will
            // be in rocket body frame regardless of how the PCB is bolted in.
            attitude_ekf_capture_mounting();
            ESP_LOGW(TAG, "Mounting offset captured.");
            can_telemetry_set_status_bit(CAN_TLM_FLAG_EKF_LOCKED, true);
            vTaskResume( xSdLoggerHandle );
            sd_logger_started = true;
        }
        cycle++;

        // After mounting capture, rotate calibrated accel and gyro into the
        // rocket body frame so logging, FSM, and CAN telemetry all see them
        // as if the PCB were mounted nose-up. No-op until cycle reaches
        // EKF_CAL_CYCLES, and effectively a no-op when BOARD_AXIS_ALIGNMENT
        // already maps to identity (since EKF then converges to identity).
        attitude_ekf_apply_mount(cal_data.accel_g, cal_data.accel_g);
        attitude_ekf_apply_mount(cal_data.gyro_dps, cal_data.gyro_dps);

        // Complementary filter predict step. Uses rocket-frame body Z accel
        // (after remap + mount apply) as the vertical-axis input. Drift is
        // bounded by sensor_velocity_correct() in the alt task.
        sensor_velocity_predict(cal_data.accel_g[2], dt_s);

        // Publish IMU-side fields into FusedPacket. Both producer tasks share
        // the spinlock; the SD logger snapshots the whole struct atomically.
        attitude_t att;
        attitude_ekf_get_attitude(&att);
        portENTER_CRITICAL(&g_fused_mux);
        fusedData_p->currTick_ms       = sensor_get_tick_ms();
        fusedData_p->currAcc.x         = cal_data.accel_g[0];
        fusedData_p->currAcc.y         = cal_data.accel_g[1];
        fusedData_p->currAcc.z         = cal_data.accel_g[2];
        fusedData_p->currGyro.x        = cal_data.gyro_dps[0];
        fusedData_p->currGyro.y        = cal_data.gyro_dps[1];
        fusedData_p->currGyro.z        = cal_data.gyro_dps[2];
        fusedData_p->currMag.x         = cal_data.mag_axes[0];
        fusedData_p->currMag.y         = cal_data.mag_axes[1];
        fusedData_p->currMag.z         = cal_data.mag_axes[2];
        fusedData_p->attitude          = att;
        fusedData_p->gTotalAcc         = gTotalAcc;
        fusedData_p->gVerticalVelocity = gVerticalVelocity_fps;
        portEXIT_CRITICAL(&g_fused_mux);

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
            // Complementary-filter correct step: pull gVerticalVelocity_fps
            // toward the baro-derived velocity (Δalt/Δt). Bounds the drift
            // accumulated by sensor_velocity_predict() at 100 Hz.
            sensor_velocity_correct(alt_data.altitude);
        }
        else {
            ESP_LOGE("PRESS", "Failed to obtain Altitude data");
            continue;
        }

        // Convert to F
        alt_data.temp = alt_data.temp * 1.8 + 32.0;

        // Publish baro-side fields into FusedPacket.
        portENTER_CRITICAL(&g_fused_mux);
        fusedData_p->currPress = alt_data.pressure;
        fusedData_p->currTempF = alt_data.temp;
        fusedData_p->gAltitude = alt_data.altitude;
        portEXIT_CRITICAL(&g_fused_mux);

        vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY_MS));
    }
}

void vSdLoggerTask(void *pvParameters) {
    LogSensorRecord_t record;
    FusedPacket_t snap;
    uint8_t print_counter = 0;
    static bool loggingFlag;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(LOGGING_DELAY_MS));

        fused_snapshot(&snap);

        record.timestamp_s   = snap.currTick_ms / 1000.0f;
        record.accel         = snap.currAcc;
        record.baro.pressure = snap.currPress;
        record.baro.temp     = snap.currTempF;
        record.baro.altitude = snap.gAltitude;
        record.orientation   = snap.attitude;
        record.gTotalAcc     = snap.gTotalAcc;
        record.gVertVelocity = snap.gVerticalVelocity;
        record.flightState   = getCurrentFlightState();
        record.pyroStatus    = gPyroStatus;

        if (record.flightState == STATE_DISARM && loggingFlag) {
            ESP_LOGI(TAG, "Rocket Landed. Stopping live telemetry logging...");
            loggingFlag = false;
            sd_safe_unmount();
        }
        loggingFlag = sd_logger_is_active();

        if (loggingFlag) {
            if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
                write_packet(record);
                xSemaphoreGive(xSemaphore);
            }
        }

        if (print_counter++ % 100 == 0) {
            ESP_LOGI("AHRS", "\tYaw: %d deg\tPitch: %d deg\tRoll: %d deg\tTilt: %d deg",
                (int)snap.attitude.yaw_deg, (int)snap.attitude.pitch_deg,
                (int)snap.attitude.roll_deg, (int)snap.attitude.tilt_deg);
            ESP_LOGI("IMU", "Accel: \tX: %.1f,\tY: %.1f,\tZ: %.1f",
                snap.currAcc.x, snap.currAcc.y, snap.currAcc.z);
            ESP_LOGI("IMU", "\tTotal Accel: %.1f g", snap.gTotalAcc);
            ESP_LOGI("BARO", "\tPressure: %.1f hPa, \tTemp: %.1f F",
                snap.currPress, snap.currTempF);
            ESP_LOGI("BARO", "\tAltitude: %.1f ft", snap.gAltitude);
            ESP_LOGI("FUSION", "\tVertical Velocity: %.1f ft/s", snap.gVerticalVelocity);
        }
    }
}

// Atomic snapshot of the fused packet. Both producer tasks write
// under the same spinlock, so this copy is a coherent mix of the
// most recent IMU and baro samples.
static inline void fused_snapshot(FusedPacket_t *out) {
    portENTER_CRITICAL(&g_fused_mux);
    *out = *fusedData_p;
    portEXIT_CRITICAL(&g_fused_mux);
}

void load_mag_cal() {
    if (mag_cal_load_nvs(&mag_cal) != ESP_OK || !mag_cal.is_calibrated) {
        ESP_LOGW(TAG, "Mag cal not in NVS. Flight will run with raw mag; EKF will absorb slowly.");
        memset(&mag_cal, 0, sizeof(mag_cal));
        // identity soft-iron so sensor_update_mag passthrough is reasonable
        mag_cal.soft_iron[0][0] = 1.0f;
        mag_cal.soft_iron[1][1] = 1.0f;
        mag_cal.soft_iron[2][2] = 1.0f;
        can_telemetry_set_status_bit(CAN_TLM_FLAG_MAG_CAL_VALID, false);
    } else {
        ESP_LOGI(TAG, "Mag cal loaded from NVS.");
        can_telemetry_set_status_bit(CAN_TLM_FLAG_MAG_CAL_VALID, true);
    }
}
