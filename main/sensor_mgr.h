/*
 * File: sensor_mgr.h
 *
 * Sensor calibration and data processing for the flight computer IMU.
 *
 * Created: 2025
 */

#ifndef SENSOR_MGR_H
#define SENSOR_MGR_H

#include <stdbool.h>
#include "esp_err.h"
     #include "iis2mdc.h"
#include "lsm6dsv80x.h"
#include "lps22df.h"

static float g_ground_pressure_hpa = 1013.25f; // default sea level pressure

#define IMU_CAL_NUM_SAMPLES     500
#define MAG_CAL_NUM_SAMPLES     500
#define PRESS_CAL_NUM_SAMPLES   100
#define IMU_CAL_SAMPLE_DELAY_MS 10
#define MAG_CAL_SAMPLE_DELAY_MS 60
#define PRESS_CAL_SAMPLE_DELAY_MS 10

#define CAL_NVS_NAMESPACE "sensor_cal"

typedef struct {
    float gyro_bias_mdps[3]; // gyroscope bias in mdps
    bool is_calibrated;
} imu_cal_t;

typedef struct {
    float hard_iron[3];      // X[0..2]: offset to subtract
    float soft_iron[3][3];   // 3x3 correction matrix
    bool is_calibrated;
} mag_cal_t;

typedef struct {
    float accel_g[3]; // calibrated accelerometer in g
    float gyro_dps[3]; // calibrated gyroscope in deg/s
    float mag_axes[3]; // calibrated mag axes
} imu_calibrated_t;

typedef struct {
    float pressure;
    float temp;
    float altitude;
} AltData_t;

typedef enum {
    SENSOR_IMU,
    SENSOR_ALT
} SensorType_t;

typedef struct {
    SensorType_t type;
    union {
        imu_calibrated_t imu;
        AltData_t alt;
    } data;
} SensorMessage_t;

typedef struct {
    float x;
    float y;
    float z;
} Accel_Axes_t;

typedef struct {
    float x;
    float y;
    float z;
} Gyro_Axes_t;

typedef struct {
    float x;
    float y;
    float z;
} Mag_Axes_t;

/**
 * Run startup gyro-bias calibration by averaging samples while stationary.
 * Blocks for ~5 seconds. Must be called before tasks start.
 * Accel bias is intentionally not estimated here (orientation-dependent and
 * absorbed by the EKF's calibration phase instead).
 */
esp_err_t imu_calibrate(LSM6DSV80X_Object_t *imu, imu_cal_t *cal);

esp_err_t mag_calibrate(IIS2MDC_Object_t *mag, mag_cal_t *cal);

/**
 * Convert raw IMU readings to physical units, subtracting gyro bias.
 * Accel: mg -> g (no bias)
 * Gyro:  mdps -> deg/s (with bias removed)
 */
void imu_apply_calibration(const imu_cal_t *cal, const LSM6DSV80X_Axes_t *raw_accel, 
                           const LSM6DSV80X_Axes_t *raw_gyro, imu_calibrated_t *out);

// Update gMag from magnetometer axes (applies hard/soft iron from cal if calibrated)
void mag_apply_calibration(const mag_cal_t *cal, IIS2MDC_Axes_t *axes, imu_calibrated_t *out);

/* Persist mag calibration only; gyro is re-cal'd on each boot, accel handled by EKF. */
esp_err_t mag_cal_save_nvs(const mag_cal_t *mag_cal);
esp_err_t mag_cal_load_nvs(mag_cal_t *mag_cal);

// Shared flight data globals (updated by sensor tasks, read by FSM and SD logger)
extern float gTotalAcc;       // total acceleration magnitude in g
extern float gAltitude;       // barometric altitude AGL in feet
extern float gDegOffVert;     // degrees off vertical
extern float gAccel[3];       // calibrated accelerometer (g)
extern float gGyro[3];        // calibrated gyroscope (dps)
extern float gMag[3];         // magnetometer (raw axes)
extern float gVerticalVelocity_fps;  // vertical velocity from baro (ft/s, +up)
extern uint8_t gPyroStatus;   // pyro fired bitmask (bit 0-3 = channels 1-4)

// Millisecond tick for FSM timing (wraps esp_timer)
uint32_t sensor_get_tick_ms(void);

// Set ground-level reference pressure (call once at startup)
void sensor_set_ground_pressure(float pressure_hpa);

// Read the stored ground reference (used by alt task each cycle)
float sensor_get_ground_pressure(void);

// Sample LPS22DF ~1 s, average, and store as ground reference. Blocking.
esp_err_t baro_calibrate_ground(LPS22DF_Object_t *baro);

// Slowly re-zero the stored ground pressure with one EMA step. The caller
// MUST only call this while the rocket is on the pad (FSM in IDLE/ARMED) —
// at apogee gTotalAcc is also ~1g, so any accel-based "at rest" gate is unsafe.
void sensor_track_ground_pressure(float pressure_hpa);

// Update gAltitude from current pressure reading
float sensor_get_altitude(float pressure_hpa, float temp);

// Update gTotalAcc, gDegOffVert, gAccel, gGyro from calibrated IMU data
void sensor_update_flight_data(const imu_calibrated_t *imu);

#endif // SENSOR_MGR_H
