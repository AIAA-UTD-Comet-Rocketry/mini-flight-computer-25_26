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
 * Board-to-rocket axis alignment. Mirrors the FusionRemap enum naming.
 * Letters describe how the SENSOR axes map to the BODY (rocket) axes:
 *   PXPYPZ = sensor +X→body +X, +Y→body +Y, +Z→body +Z (no remap)
 *   PYPZPX = sensor +Y→body +X, +Z→body +Y, +X→body +Z
 *   PZNYPX = sensor +Z→body +X, -Y→body +Y, +X→body +Z
 * etc.
 * The 24 entries cover every right-handed 90° axis-aligned rotation.
 * Pick the one that makes gravity (1 g pointing down on the pad) read on
 * body +Z when the rocket sits nose-up — that's the correct alignment.
 */
typedef enum {
    AXIS_ALIGN_PXPYPZ,  AXIS_ALIGN_PXPZNY,  AXIS_ALIGN_PXNZPY,  AXIS_ALIGN_PXNYNZ,
    AXIS_ALIGN_PYPXNZ,  AXIS_ALIGN_PYPZPX,  AXIS_ALIGN_PYNZNX,  AXIS_ALIGN_PYNXPZ,
    AXIS_ALIGN_PZPXPY,  AXIS_ALIGN_PZPYNX,  AXIS_ALIGN_PZNYPX,  AXIS_ALIGN_PZNXNY,
    AXIS_ALIGN_NZPXNY,  AXIS_ALIGN_NZPYPX,  AXIS_ALIGN_NZNYNX,  AXIS_ALIGN_NZNXPY,
    AXIS_ALIGN_NYPXPZ,  AXIS_ALIGN_NYPZNX,  AXIS_ALIGN_NYNZPX,  AXIS_ALIGN_NYNXNZ,
    AXIS_ALIGN_NXPYNZ,  AXIS_ALIGN_NXPZPY,  AXIS_ALIGN_NXNZNY,  AXIS_ALIGN_NXNYPZ,
} sensor_axis_align_t;

// Rocket mounting alignment for THIS board. Edit this one constant if the
// PCB orientation changes. Default is identity (sensor frame = rocket frame).
#ifndef BOARD_AXIS_ALIGNMENT
#define BOARD_AXIS_ALIGNMENT AXIS_ALIGN_PXPYPZ
#endif

// Permute and sign-flip a 3-vector according to the alignment. in/out may alias.
void sensor_remap_axes(const float in[3], sensor_axis_align_t align, float out[3]);

// Complementary filter on vertical velocity.
// Predict: integrate (vert_accel_g - 1g) into gVerticalVelocity_fps. Call
// from the 100 Hz IMU task with body-frame Z accel after axis remap.
void sensor_velocity_predict(float vert_accel_g, float dt_s);

// Correct: pull gVerticalVelocity_fps toward (Δalt/Δt) computed from
// consecutive baro samples. Call from the alt task after sensor_get_altitude.
void sensor_velocity_correct(float new_alt_ft);

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
