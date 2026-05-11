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
#include "Fusion.h"

#define IMU_CAL_NUM_SAMPLES         500
#define MAG_CAL_NUM_SAMPLES         500
#define PRESS_CAL_NUM_SAMPLES       100
#define IMU_CAL_SAMPLE_DELAY_MS     10
#define MAG_CAL_SAMPLE_DELAY_MS     60
#define PRESS_CAL_SAMPLE_DELAY_MS   10
#define SEA_LEVEL_PRESSURE_HPA      1013.25

/**
 * Board-to-rocket axis alignment.
 * Letters describe how the SENSOR axes map to the BODY (rocket) axes:
 *   PXPYPZ = sensor +X→body +X, +Y→body +Y, +Z→body +Z (no remap)
 *   PYPZPX = sensor +Y→body +X, +Z→body +Y, +X→body +Z
 *   PZNYPX = sensor +Z→body +X, -Y→body +Y, +X→body +Z
 * etc.
 * The 24 entries cover every right-handed 90° axis-aligned rotation.
 * Pick the one that makes gravity (1 g pointing down on the pad) read on
 * body +Z when the rocket sits nose-up — that's the correct alignment.
 */
#ifndef BOARD_AXIS_ALIGNMENT
#define BOARD_AXIS_ALIGNMENT FusionRemapAlignmentPXPYPZ
#endif

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
} Sensor_Axes_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Euler_Angles_t;

// Shared flight data globals (updated by sensor tasks, read by FSM and SD logger)
extern float gTotalAcc;       // total acceleration magnitude in g
extern float gAltitude;       // barometric altitude AGL in feet
extern float gAccel[3];       // calibrated accelerometer (g)
extern float gGyro[3];        // calibrated gyroscope (dps)
extern float gOrient[3];
extern float gVerticalVelocity_fps;  // vertical velocity from baro (ft/s, +up)
extern uint8_t gPyroStatus;   // pyro fired bitmask (bit 0-3 = channels 1-4)

// Apply 2nd-order Butterworth IIR LPF (fc=5 Hz) to raw barometric pressure.
// Call from a single task only — state is not thread-safe.
float sensor_pressure_filter(float raw_hpa);

// Complementary filter on vertical velocity.
// earth_z_g is Fusion earth accel Z in g
// Predict: integrate earth z accel into gVerticalVelocity. Call
// from the 100 Hz IMU task with body-frame Z accel after axis remap.
// (NWU: positive = up)
void sensor_velocity_predict(float earth_z_g, float dt_s);

// Correct: pull gVerticalVelocity toward (Δalt/Δt) computed from
// consecutive baro samples. Call from the alt task after sensor_get_altitude.
void sensor_velocity_correct(float baro_altitude_ft, uint32_t tick_ms);

// Set ground-level reference pressure (call once at startup)
void sensor_set_ground_pressure(float pressure_hpa);

// Read the stored ground reference (used by alt task each cycle)
float sensor_get_ground_pressure(void);

// Slowly re-zero the stored ground pressure with one EMA step. The caller
// MUST only call this while the rocket is on the pad (FSM in IDLE/ARMED) —
// at apogee gTotalAcc is also ~1g, so any accel-based "at rest" gate is unsafe.
void sensor_track_ground_pressure(float pressure_hpa);

// Update gAltitude from current pressure reading
float sensor_get_altitude(float pressure_hpa, float temp);

float sensor_get_vertical_velocity(void);

// Update gTotalAcc, gAccel, gGyro from calibrated IMU data


#endif // SENSOR_MGR_H
