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
#include "Fusion.h"
#include "imu_calibration.h"

// Macros
#define CHECK_TASK_CREATION(ret, err_msg) \
    if((ret) != pdPASS) { \
        ESP_LOGI(TAG, err_msg); \
    }
#define UTDMALLOC(n, els)              (els *) malloc((n)*sizeof(els))
#define MIN_STACK_SIZE          configMINIMAL_STACK_SIZE // original minimum causes stack overflow
#define SENSOR_SAMPLE_RATE_MS   10  // 1/10ms = 100Hz
#define LOGGING_SAMPLE_RATE_MS  100 // ms (10 Hz SD log cadence)
#define FSM_SAMPLE_RATE         10

// Uncomment to run pyro bench test on boot (DO NOT fly with this enabled)
//#define PYRO_BENCH_TEST

static const char *TAG = "Main";

static FlightState flight_state;
FusedPacket_ptr fusedData_p;
FusionAhrs ahrs;
FusionBias bias;

// use identity matrix to ensure no cross axis correction
FusionMatrix const FUSION_IDENTITY_MATRIX = {
    .array = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
    }
};

// Calibration parameters
FusionMatrix gyroscopeMisalignment = FUSION_IDENTITY_MATRIX; 
FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
FusionVector gyroOffset = {{0.0f, 0.0f, 0.0f}}; // default to zero if no calibration found

FusionMatrix accelerometerMisalignment = FUSION_IDENTITY_MATRIX;
FusionVector accelSensitivity = {{1.0f, 1.0f, 1.0f}};
FusionVector accelOffset = {{0.0f, 0.0f, 0.0f}};

static void init_nvs_flash_memory(void);
static inline void fused_snapshot(FusedPacket_t *out);
uint32_t sensor_get_tick_ms(void);
static void load_params(void);
static void init_AHRS(void);
void sensor_update_flight_data(void);

// RTOS Tasks
static void vImuHandlerTask(void *pvParameters);
static void vAltHandlerTask(void *pvParameters);
static void vSdLoggerTask(void *pvParameters);
static void vFsmTask(void *pvParameters);

SemaphoreHandle_t xSemaphore;

BaseType_t task_ret;
TaskHandle_t xImuTaskHandle = NULL;
TaskHandle_t xAltTaskHandle = NULL;
TaskHandle_t xFsmTaskHandle = NULL;
TaskHandle_t xLEDTaskHandle = NULL;
TaskHandle_t xSdLoggerHandle = NULL;
TaskHandle_t xPyroTaskHandle = NULL;

// Spinlock guarding writes/reads of *fusedData_p. Held only for struct-field
// copies (microseconds), never across SD I/O.
static portMUX_TYPE g_fused_mux = portMUX_INITIALIZER_UNLOCKED;

static bool sd_logger_started = false;

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

    init_nvs_flash_memory();

    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for everything to settle (TODO event based wait)

    if(1) {
    calibration_run_menu(
        mini_fc_handle->lsm6dsv80x_handle,
        &accelOffset,
        &accelSensitivity,
        &gyroOffset
    );
}

    load_params();
    init_AHRS();

    // Sample ambient pressure for ~1 s and use as the altitude=0 reference
    if (baro_calibrate_ground(mini_fc_handle->lps22df_handle) != ESP_OK) {
        ESP_LOGW(TAG, "Ground pressure cal failed, falling back to sea-level default.");
        can_telemetry_set_status_bit(CAN_TLM_FLAG_GROUND_PRESS_VALID, false);
    } else {
        can_telemetry_set_status_bit(CAN_TLM_FLAG_GROUND_PRESS_VALID, true);
    }

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

    task_ret = xTaskCreate(vSdLoggerTask,
                            "SD Logger",
                            6 * MIN_STACK_SIZE,
                            NULL,
                            1,
                            &xSdLoggerHandle);
    CHECK_TASK_CREATION(task_ret, "SD Logger task failed to create!");

    // Suspend task until AHRS is initialized
    //vTaskSuspend( xSdLoggerHandle );

    // IMU + EKF + Mag (all sensor fusion in one 100 Hz task).
    // EKF allocates several dspm::Mat scratch matrices per Process()/Update*()
    // call; 8 KB gives comfortable headroom over the ~3 KB peak observed.
    task_ret = xTaskCreate(vImuHandlerTask,
                           "IMU",
                           4 * MIN_STACK_SIZE,
                           (void*) mini_fc_handle->lsm6dsv80x_handle,
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

void vImuHandlerTask(void *pvParameters) 
{
    LSM6DSV80X_Object_t *imu = (LSM6DSV80X_Object_t *)pvParameters;

    LSM6DSV80X_Axes_t accel_axes, gyro_axes;
    FusionVector accel_axes_f, gyro_axes_f;

    while (1) {
        LSM6DSV80X_ACC_GetAxes(imu, &accel_axes);
        LSM6DSV80X_GYRO_GetAxes(imu, &gyro_axes);

        // mg -> g
        accel_axes_f.axis.x = (float)accel_axes.x / 1000.0;
        accel_axes_f.axis.y = (float)accel_axes.y / 1000.0;
        accel_axes_f.axis.z = (float)accel_axes.z / 1000.0;

        // mdps -> dps
        gyro_axes_f.axis.x = (float)gyro_axes.x / 1000.0;
        gyro_axes_f.axis.y = (float)gyro_axes.y / 1000.0;
        gyro_axes_f.axis.z = (float)gyro_axes.z / 1000.0;

        // Apply calibration
        FusionVector accel_cal = FusionModelInertial(
            accel_axes_f,
            accelerometerMisalignment,
            accelSensitivity,
            accelOffset
        );
        FusionVector gyro_cal = FusionModelInertial(
            gyro_axes_f,
            gyroscopeMisalignment,
            gyroscopeSensitivity,
            gyroOffset
        );

        // Remap sensor axes to rocket body frame.
        // After this, EKF, FSM, telemetry, and logger all see rocket-frame data.
        // No-op if BOARD_AXIS_ALIGNMENT is the default identity (PXPYPZ).
        accel_cal = FusionRemap(accel_cal, BOARD_AXIS_ALIGNMENT);
        gyro_cal = FusionRemap(gyro_cal, BOARD_AXIS_ALIGNMENT);

        // Update bias algorithm
        gyro_cal = FusionBiasUpdate(&bias, gyro_cal);

        // Calculate delta time to compensate for gyroscope sample clock errors
        const uint32_t timestamp = sensor_get_tick_ms();
        static uint32_t previousTimestamp;
        const float deltaTime = (float) (timestamp - previousTimestamp) / 1000.0f; 
        previousTimestamp = timestamp;

        // Update AHRS algorithm
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyro_cal, accel_cal, deltaTime);

        // Store AHRS outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        float totalAccG = sqrt(accel_cal.axis.x * accel_cal.axis.x + accel_cal.axis.y * accel_cal.axis.y + accel_cal.axis.z * accel_cal.axis.z);

        // Publish IMU-side fields into FusedPacket. Both producer tasks share
        // the spinlock; the SD logger snapshots the whole struct atomically.
        portENTER_CRITICAL(&g_fused_mux);
        fusedData_p->currTick_ms  = timestamp;
        fusedData_p->currAcc.axis.x    = accel_cal.axis.x;
        fusedData_p->currAcc.axis.y    = accel_cal.axis.y;
        fusedData_p->currAcc.axis.z    = accel_cal.axis.z;
        fusedData_p->currGyro.axis.x   = gyro_cal.axis.x;
        fusedData_p->currGyro.axis.y   = gyro_cal.axis.y;
        fusedData_p->currGyro.axis.z   = gyro_cal.axis.z;
        fusedData_p->orientation.angle.roll  = euler.angle.roll;
        fusedData_p->orientation.angle.pitch = euler.angle.pitch;
        fusedData_p->orientation.angle.yaw   = euler.angle.yaw;
        // earth accel = linear accel with gravity removed (useful for velocity)
        fusedData_p->linearAcc.axis.x = earth.axis.x;
        fusedData_p->linearAcc.axis.y = earth.axis.y;
        fusedData_p->linearAcc.axis.z = earth.axis.z;
        fusedData_p->gTotalAcc    = totalAccG;
        portEXIT_CRITICAL(&g_fused_mux);

        vTaskDelay(pdMS_TO_TICKS(SENSOR_SAMPLE_RATE_MS));
    }
}

static void vAltHandlerTask(void *pvParameters) 
{
    LPS22DF_Object_t *alt = (LPS22DF_Object_t *)pvParameters;
    AltData_t alt_data;
    float verticalVel = 0;

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
            verticalVel = getVerticalVelocity(alt_data.altitude, sensor_get_tick_ms());
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
        fusedData_p->gVerticalVelocity = verticalVel;
        portEXIT_CRITICAL(&g_fused_mux);

        vTaskDelay(pdMS_TO_TICKS(SENSOR_SAMPLE_RATE_MS));
    }
}

static void vFsmTask(void *pvParameters) 
{
    (void)pvParameters;
    while (1) {
        sensor_update_flight_data();
        updateState(&flight_state);
        vTaskDelay(pdMS_TO_TICKS(FSM_SAMPLE_RATE));
    }
}

static void vSdLoggerTask(void *pvParameters) 
{
    LogSensorRecord_t record;
    FusedPacket_t snap;
    uint8_t print_counter = 0;
    static bool loggingFlag;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(LOGGING_SAMPLE_RATE_MS));

        fused_snapshot(&snap);

        record.timestamp_s   = snap.currTick_ms / 1000.0f;
        record.accel.axis    = snap.currAcc.axis;
        record.baro.pressure = snap.currPress;
        record.baro.temp     = snap.currTempF;
        record.orientation.angle   = snap.orientation.angle;
        record.baro.altitude = snap.gAltitude;
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
            ESP_LOGI("AHRS", "\tYaw: %d deg\tPitch: %d deg\tRoll: %d deg",
                (int)snap.orientation.angle.yaw, (int)snap.orientation.angle.pitch,
                (int)snap.orientation.angle.roll);
            ESP_LOGI("IMU", "Accel: \tX: %.1f,\tY: %.1f,\tZ: %.1f",
                snap.currAcc.axis.x, snap.currAcc.axis.y, snap.currAcc.axis.z);
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

void sensor_update_flight_data(void) {
    gAltitude = fusedData_p->gAltitude;
    gVerticalVelocity_fps = fusedData_p->gVerticalVelocity;
    gTotalAcc = fusedData_p->gTotalAcc;
    gOrient[0] = fusedData_p->orientation.angle.yaw;
    gOrient[1] = fusedData_p->orientation.angle.pitch;
    gOrient[2] = fusedData_p->orientation.angle.roll;
    gAccel[0] = fusedData_p->currAcc.axis.x;
    gAccel[1] = fusedData_p->currAcc.axis.y;
    gAccel[2] = fusedData_p->currAcc.axis.z;
}

/**
 * @brief Initialize NVS flash memory
 */
static void init_nvs_flash_memory(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}

// Load calibrated params from NVS (if they exist).
static void load_params(void) 
{
    FusionVector loaded;
    
    if (cal_nvs_load("accel_offset", &loaded) == ESP_OK) {
        accelOffset = loaded;  // make these non-const
        ESP_LOGI(TAG, "Loaded accel offset from NVS");
    }
    if (cal_nvs_load("accel_sens", &loaded) == ESP_OK) {
        accelSensitivity = loaded;
        ESP_LOGI(TAG, "Loaded accel sensitivity from NVS");
    }
    if (cal_nvs_load("gyro_offset", &loaded) == ESP_OK) {
        gyroOffset = loaded;
        ESP_LOGI(TAG, "Loaded gyro offset from NVS");
    }
}

uint32_t sensor_get_tick_ms(void) 
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void init_AHRS(void) 
{
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 250.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 0, // mag sensor disabled
        .recoveryTriggerPeriod = 5 * (1000 / SENSOR_SAMPLE_RATE_MS), /* 500 samples in 5 sec */
    };

    FusionAhrsSetSettings(&ahrs, &settings);
    FusionBiasInitialise(&bias);

    FusionBiasSettings biasSettings = fusionBiasDefaultSettings;
    biasSettings.sampleRate = SENSOR_SAMPLE_RATE_MS;

    FusionBiasSetSettings(&bias, &biasSettings);

    ESP_LOGI(TAG, "Fusion AHRS Initialized.");
}
