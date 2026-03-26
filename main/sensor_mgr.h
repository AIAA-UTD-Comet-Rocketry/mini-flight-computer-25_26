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
#include "lsm6dsv80x.h"

#define IMU_CAL_NUM_SAMPLES     500
#define IMU_CAL_SAMPLE_DELAY_MS 10

typedef struct {
    float accel_bias_mg[3]; // accelerometer bias in mg
    float gyro_bias_mdps[3]; // gyroscope bias in mdps
    bool is_calibrated;
} imu_cal_t;

typedef struct {
    float accel_g[3]; // calibrated accelerometer in g
    float gyro_dps[3]; // calibrated gyroscope in deg/s
} imu_calibrated_t;

/**
 * Run startup calibration by collecting samples while stationary.
 * Blocks for ~5 seconds. Must be called before tasks start.
 * Assumes board is stationary with Z-axis aligned to gravity.
 */
esp_err_t imu_calibrate(LSM6DSV80X_Object_t *imu, imu_cal_t *cal);

/**
 * Apply calibration offsets to raw readings and convert units.
 * Accel: mg -> g (with bias removed)
 * Gyro: mdps -> deg/s (with bias removed)
 */
void imu_apply_calibration(const imu_cal_t *cal,
                           const LSM6DSV80X_Axes_t *raw_accel,
                           const LSM6DSV80X_Axes_t *raw_gyro,
                           imu_calibrated_t *out);

// Shared flight data globals (updated by sensor tasks, read by FSM and SD logger)
extern float gTotalAcc;       // total acceleration magnitude in g
extern float gAltitude;       // barometric altitude AGL in feet
extern float gDegOffVert;     // degrees off vertical
extern float gAccel[3];       // calibrated accelerometer (g)
extern float gGyro[3];        // calibrated gyroscope (dps)
extern float gMag[3];         // magnetometer (raw axes)
extern float gPressure;       // barometric pressure (hPa)
extern float gTemperature_F;  // temperature (Fahrenheit)
extern uint8_t gPyroStatus;   // pyro fired bitmask (bit 0-3 = channels 1-4)

// Millisecond tick for FSM timing (wraps esp_timer)
uint32_t sensor_get_tick_ms(void);

// Set ground-level reference pressure (call once at startup)
void sensor_set_ground_pressure(float pressure_hpa);

// Update gAltitude from current pressure reading
void sensor_update_altitude(float pressure_hpa, float temp);

// Update gTotalAcc, gDegOffVert, gAccel, gGyro from calibrated IMU data
void sensor_update_flight_data(const imu_calibrated_t *imu);

// Update gMag from magnetometer axes
void sensor_update_mag(int32_t x, int32_t y, int32_t z);

#endif // SENSOR_MGR_H
