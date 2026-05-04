#include "attitude_ekf.h"

#include <math.h>
#include "ekf_imu13states.h"

namespace {

constexpr float DEG_TO_RAD = 0.01745329252f;
constexpr float RAD_TO_DEG = 57.2957795131f;

ekf_imu13states *g_ekf = nullptr;

// Mounting offset captured at EKF cal completion. Identity until captured.
// g_q_mount[] is q_pcb_to_rocket (=q_pcb_to_inertial at cal time, since the
// rocket is presumed to be in its nominal orientation = inertial).
// g_R_pcb_to_rocket is R(g_q_mount), used to rotate sensor vectors from PCB
// body frame to rocket body frame.
float g_q_mount[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_R_pcb_to_rocket[3][3] = { {1.0f,0.0f,0.0f}, {0.0f,1.0f,0.0f}, {0.0f,0.0f,1.0f} };
bool  g_mount_captured = false;

// Quaternion product r = a * b, q = [w, x, y, z]
inline void quat_mul(const float a[4], const float b[4], float r[4]) {
    r[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    r[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    r[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    r[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

// Read EKF quaternion and apply the mounting correction (if captured).
// Result is q_rocket_to_inertial.
inline void corrected_quat(float q_out[4]) {
    if (g_ekf == nullptr) {
        q_out[0] = 1.0f; q_out[1] = q_out[2] = q_out[3] = 0.0f;
        return;
    }
    float q_now[4] = { g_ekf->X.data[0], g_ekf->X.data[1],
                       g_ekf->X.data[2], g_ekf->X.data[3] };
    if (!g_mount_captured) {
        q_out[0] = q_now[0]; q_out[1] = q_now[1];
        q_out[2] = q_now[2]; q_out[3] = q_now[3];
        return;
    }
    // q_corrected = q_now * conj(q_mount)
    float q_conj[4] = { g_q_mount[0], -g_q_mount[1], -g_q_mount[2], -g_q_mount[3] };
    quat_mul(q_now, q_conj, q_out);
}

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
    corrected_quat(q);
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

    // Angle between rocket body Z axis and inertial Z axis, computed from
    // the mount-corrected quaternion so the readout is in rocket frame.
    // R[2][2] = 1 - 2*(qx^2 + qy^2) where q = [w, x, y, z].
    float q[4]; corrected_quat(q);
    float cos_tilt = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
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

    // Snapshot mount-corrected quaternion once so the four Euler components
    // describe the same instant. Without mounting capture this falls back
    // to the raw EKF quaternion.
    float q[4]; corrected_quat(q);
    float w = q[0], x = q[1], y = q[2], z = q[3];
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

extern "C" void attitude_ekf_capture_mounting(void)
{
    if (g_ekf == nullptr) return;

    g_q_mount[0] = g_ekf->X.data[0];
    g_q_mount[1] = g_ekf->X.data[1];
    g_q_mount[2] = g_ekf->X.data[2];
    g_q_mount[3] = g_ekf->X.data[3];

    // Build R(g_q_mount) = matrix that rotates PCB-frame vectors to rocket frame.
    float w = g_q_mount[0], x = g_q_mount[1], y = g_q_mount[2], z = g_q_mount[3];
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;

    g_R_pcb_to_rocket[0][0] = 1.0f - 2.0f * (yy + zz);
    g_R_pcb_to_rocket[0][1] = 2.0f * (xy - wz);
    g_R_pcb_to_rocket[0][2] = 2.0f * (xz + wy);
    g_R_pcb_to_rocket[1][0] = 2.0f * (xy + wz);
    g_R_pcb_to_rocket[1][1] = 1.0f - 2.0f * (xx + zz);
    g_R_pcb_to_rocket[1][2] = 2.0f * (yz - wx);
    g_R_pcb_to_rocket[2][0] = 2.0f * (xz - wy);
    g_R_pcb_to_rocket[2][1] = 2.0f * (yz + wx);
    g_R_pcb_to_rocket[2][2] = 1.0f - 2.0f * (xx + yy);

    g_mount_captured = true;
}

extern "C" void attitude_ekf_apply_mount(const float in[3], float out[3])
{
    if (!g_mount_captured) {
        out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
        return;
    }
    // Use a temporary so callers can pass the same pointer for in and out.
    float r0 = g_R_pcb_to_rocket[0][0]*in[0] + g_R_pcb_to_rocket[0][1]*in[1] + g_R_pcb_to_rocket[0][2]*in[2];
    float r1 = g_R_pcb_to_rocket[1][0]*in[0] + g_R_pcb_to_rocket[1][1]*in[1] + g_R_pcb_to_rocket[1][2]*in[2];
    float r2 = g_R_pcb_to_rocket[2][0]*in[0] + g_R_pcb_to_rocket[2][1]*in[1] + g_R_pcb_to_rocket[2][2]*in[2];
    out[0] = r0; out[1] = r1; out[2] = r2;
}
