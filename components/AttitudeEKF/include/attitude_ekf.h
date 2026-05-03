#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_EKF_H
