#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Aerospace-convention attitude bundle. Convenient for logging and as the
// payload struct for CAN Aerospace telemetry frames.
//   roll  (phi)   : rotation about body X, +/- 180 deg
//   pitch (theta) : rotation about body Y, +/- 90 deg
//   yaw   (psi)   : rotation about body Z, +/- 180 deg
//   quat          : [w, x, y, z] — body-to-inertial rotation
//   tilt_deg      : angle between body Z and inertial Z, 0..180 deg
typedef struct {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float tilt_deg;
    float quat[4];
} attitude_t;

void attitude_ekf_init(void);

void attitude_ekf_seed_gyro_bias_dps(const float gyro_bias_dps[3]);

// Calibration phase: also updates mag amplitude/offset states (X[7..12]).
// Run for ~5s while board is stationary on the pad.
void attitude_ekf_calibrate_step(const float accel_g[3],
                                 const float gyro_dps[3],
                                 const float mag[3],
                                 float dt_s);

// Normal phase: updates attitude + gyro bias only.
void attitude_ekf_update(const float accel_g[3],
                         const float gyro_dps[3],
                         const float mag[3],
                         float dt_s);

void attitude_ekf_get_quaternion(float q[4]);   // [w, x, y, z]
void attitude_ekf_get_gyro_bias_dps(float bias[3]);
float attitude_ekf_get_tilt_deg(void);

// Pack the current quaternion into yaw/pitch/roll (degrees, ZYX intrinsic).
// Single read per call — fills all fields atomically from the same X snapshot.
// After attitude_ekf_capture_mounting() has been called, this returns the
// rocket-frame attitude (i.e. yaw/pitch/roll = 0 on the pad regardless of
// how the PCB is physically mounted).
void attitude_ekf_get_attitude(attitude_t *out);

// Snapshot the EKF quaternion as the PCB-to-rocket mounting offset and
// pre-compute the rotation matrix used to bring sensor vectors into the
// rocket body frame. Call once after EKF calibration completes, with the
// rocket sitting in its desired zero orientation (typically nose-up on the
// pad). After this:
//   - attitude_ekf_get_attitude() / get_quaternion() / get_tilt_deg() return
//     rocket-frame values (zero on the pad)
//   - attitude_ekf_apply_mount() rotates raw accel/gyro vectors from PCB
//     frame to rocket frame
void attitude_ekf_capture_mounting(void);

// Rotate a 3-vector from PCB body frame to rocket body frame using the
// captured mounting matrix. If mounting has not been captured yet, copies
// in to out unchanged. Use this on calibrated accel and gyro before publishing
// to logging/telemetry so consumers see rocket-frame readings.
void attitude_ekf_apply_mount(const float in[3], float out[3]);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_EKF_H
