#include "attitude_ekf.h"

#include <math.h>
#include "ekf_imu13states.h"

namespace {

constexpr float DEG_TO_RAD = 0.01745329252f;
constexpr float RAD_TO_DEG = 57.2957795131f;

ekf_imu13states *g_ekf = nullptr;

inline void normalize3(const float in[3], float out[3])
{
    float n = sqrtf(in[0] * in[0] + in[1] * in[1] + in[2] * in[2]);
    if (n < 1e-9f) {
        out[0] = 1.0f; out[1] = 0.0f; out[2] = 0.0f;
        return;
    }
    float inv = 1.0f / n;
    out[0] = in[0] * inv;
    out[1] = in[1] * inv;
    out[2] = in[2] * inv;
}

inline void run_process(const float gyro_dps[3], float dt_s)
{
    float u[3] = {
        gyro_dps[0] * DEG_TO_RAD,
        gyro_dps[1] * DEG_TO_RAD,
        gyro_dps[2] * DEG_TO_RAD,
    };
    g_ekf->Process(u, dt_s);

    float *q = g_ekf->X.data;
    float qn = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (qn > 1e-9f) {
        float inv = 1.0f / qn;
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    }
}

} // namespace

extern "C" void attitude_ekf_init(void)
{
    if (g_ekf == nullptr) {
        g_ekf = new ekf_imu13states();
    }
    g_ekf->Init();
}

extern "C" void attitude_ekf_seed_gyro_bias_dps(const float gyro_bias_dps[3])
{
    if (g_ekf == nullptr) return;
    g_ekf->X.data[4] = gyro_bias_dps[0] * DEG_TO_RAD;
    g_ekf->X.data[5] = gyro_bias_dps[1] * DEG_TO_RAD;
    g_ekf->X.data[6] = gyro_bias_dps[2] * DEG_TO_RAD;
}

extern "C" void attitude_ekf_calibrate_step(const float accel_g[3],
                                            const float gyro_dps[3],
                                            const float mag[3],
                                            float dt_s)
{
    if (g_ekf == nullptr) return;
    run_process(gyro_dps, dt_s);

    float accel_n[3], mag_n[3];
    normalize3(accel_g, accel_n);
    normalize3(mag, mag_n);

    float R[6] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
    g_ekf->UpdateRefMeasurementMagn(accel_n, mag_n, R);
}

extern "C" void attitude_ekf_update(const float accel_g[3],
                                    const float gyro_dps[3],
                                    const float mag[3],
                                    float dt_s)
{
    if (g_ekf == nullptr) return;
    run_process(gyro_dps, dt_s);

    float accel_n[3], mag_n[3];
    normalize3(accel_g, accel_n);
    normalize3(mag, mag_n);

    float R[6] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
    g_ekf->UpdateRefMeasurement(accel_n, mag_n, R);
}

extern "C" void attitude_ekf_get_quaternion(float q[4])
{
    if (g_ekf == nullptr) {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
        return;
    }
    q[0] = g_ekf->X.data[0];
    q[1] = g_ekf->X.data[1];
    q[2] = g_ekf->X.data[2];
    q[3] = g_ekf->X.data[3];
}

extern "C" void attitude_ekf_get_gyro_bias_dps(float bias[3])
{
    if (g_ekf == nullptr) {
        bias[0] = bias[1] = bias[2] = 0.0f;
        return;
    }
    bias[0] = g_ekf->X.data[4] * RAD_TO_DEG;
    bias[1] = g_ekf->X.data[5] * RAD_TO_DEG;
    bias[2] = g_ekf->X.data[6] * RAD_TO_DEG;
}

extern "C" float attitude_ekf_get_tilt_deg(void)
{
    if (g_ekf == nullptr) return 0.0f;

    // Angle between body Z axis and inertial Z axis.
    // R[2][2] = 1 - 2*(qx^2 + qy^2) where q = [w, x, y, z].
    float qx = g_ekf->X.data[1];
    float qy = g_ekf->X.data[2];
    float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);
    if (cos_tilt > 1.0f)  cos_tilt = 1.0f;
    if (cos_tilt < -1.0f) cos_tilt = -1.0f;
    return acosf(cos_tilt) * RAD_TO_DEG;
}

extern "C" void attitude_ekf_get_attitude(attitude_t *out)
{
    if (out == nullptr) return;
    if (g_ekf == nullptr) {
        out->yaw_deg = out->pitch_deg = out->roll_deg = out->tilt_deg = 0.0f;
        out->quat[0] = 1.0f;
        out->quat[1] = out->quat[2] = out->quat[3] = 0.0f;
        return;
    }

    // Snapshot quaternion once so the four Euler components describe the same instant.
    float w = g_ekf->X.data[0];
    float x = g_ekf->X.data[1];
    float y = g_ekf->X.data[2];
    float z = g_ekf->X.data[3];
    out->quat[0] = w;
    out->quat[1] = x;
    out->quat[2] = y;
    out->quat[3] = z;

    // ZYX intrinsic Euler angles (aerospace standard).
    // pitch is asin(arg) — clamp to handle gimbal lock at +/- 90 deg.
    float sin_pitch = 2.0f * (w * y - z * x);
    if (sin_pitch >  1.0f) sin_pitch =  1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    out->pitch_deg = asinf(sin_pitch) * RAD_TO_DEG;
    out->roll_deg  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG;
    out->yaw_deg   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEG;

    float cos_tilt = 1.0f - 2.0f * (x * x + y * y);
    if (cos_tilt >  1.0f) cos_tilt =  1.0f;
    if (cos_tilt < -1.0f) cos_tilt = -1.0f;
    out->tilt_deg = acosf(cos_tilt) * RAD_TO_DEG;
}
