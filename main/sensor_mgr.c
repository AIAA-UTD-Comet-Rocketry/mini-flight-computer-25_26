#include <math.h>
#include "sensor_mgr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

static const char *TAG = "SensorMgr";

// Shared flight data globals
float gTotalAcc = 0;
float gAltitude = 0;
float gDegOffVert = 0;
float gAccel[3] = {0};
float gGyro[3] = {0};
float gMag[3] = {0};
float gVerticalVelocity_fps = 0;
uint8_t gPyroStatus = 0;

esp_err_t imu_calibrate(LSM6DSV80X_Object_t *imu, imu_cal_t *cal) {
    if (imu == NULL || cal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cal->is_calibrated = false;

    float gyro_sum[3] = {0};
    LSM6DSV80X_Axes_t accel_axes, gyro_axes;

    ESP_LOGI(TAG, "Starting gyro bias calibration (%d samples, ~%d seconds)...",
             IMU_CAL_NUM_SAMPLES, (IMU_CAL_NUM_SAMPLES * IMU_CAL_SAMPLE_DELAY_MS) / 1000);
    ESP_LOGI(TAG, "Keep the board stationary!");

    for (int i = 0; i < IMU_CAL_NUM_SAMPLES; i++) {
        LSM6DSV80X_ACC_GetAxes(imu, &accel_axes); // drain register; not used here
        LSM6DSV80X_GYRO_GetAxes(imu, &gyro_axes);

        gyro_sum[0] += (float)gyro_axes.x;
        gyro_sum[1] += (float)gyro_axes.y;
        gyro_sum[2] += (float)gyro_axes.z;

        vTaskDelay(pdMS_TO_TICKS(IMU_CAL_SAMPLE_DELAY_MS));
    }

    cal->gyro_bias_mdps[0] = gyro_sum[0] / IMU_CAL_NUM_SAMPLES;
    cal->gyro_bias_mdps[1] = gyro_sum[1] / IMU_CAL_NUM_SAMPLES;
    cal->gyro_bias_mdps[2] = gyro_sum[2] / IMU_CAL_NUM_SAMPLES;

    cal->is_calibrated = true;

    ESP_LOGI(TAG, "Gyro bias (mdps): X=%.2f  Y=%.2f  Z=%.2f",
             cal->gyro_bias_mdps[0], cal->gyro_bias_mdps[1], cal->gyro_bias_mdps[2]);

    return ESP_OK;
}

esp_err_t mag_calibrate(IIS2MDC_Object_t *mag, mag_cal_t *cal)
{
    float min[3] = { 1e9f,  1e9f,  1e9f};
    float max[3] = {-1e9f, -1e9f, -1e9f};
    IIS2MDC_Axes_t axes;

    ESP_LOGI(TAG, "Mag calibration: rotate board through all orientations");
    ESP_LOGI(TAG, "Collecting %d samples over ~%d seconds...",
             MAG_CAL_NUM_SAMPLES,
             (MAG_CAL_NUM_SAMPLES * MAG_CAL_SAMPLE_DELAY_MS) / 1000);

    for (int i = 0; i < MAG_CAL_NUM_SAMPLES; i++) {
        IIS2MDC_MAG_GetAxes(mag, &axes);
        float v[3] = {(float)axes.x, (float)axes.y, (float)axes.z};

        for (int j = 0; j < 3; j++) {
            if (v[j] < min[j]) min[j] = v[j];
            if (v[j] > max[j]) max[j] = v[j];
        }
        vTaskDelay(pdMS_TO_TICKS(MAG_CAL_SAMPLE_DELAY_MS));
    }

    // Hard-iron: midpoint of min/max sphere
    for (int j = 0; j < 3; j++) {
        cal->hard_iron[j] = (max[j] + min[j]) / 2.0f;
    }

    // Soft-iron: scale each axis to unit sphere
    // (diagonal-only correction — sufficient without full ellipsoid fit)
    float avg_delta = 0;
    float delta[3];
    for (int j = 0; j < 3; j++) {
        delta[j] = (max[j] - min[j]) / 2.0f;
        avg_delta += delta[j];
    }
    avg_delta /= 3.0f;

    memset(cal->soft_iron, 0, sizeof(cal->soft_iron));
    for (int j = 0; j < 3; j++) {
        cal->soft_iron[j][j] = (delta[j] > 0.01f) ? (avg_delta / delta[j]) : 1.0f;
    }

    cal->is_calibrated = true;

    ESP_LOGI(TAG, "Hard-iron (mGauss): X=%.1f Y=%.1f Z=%.1f",
             cal->hard_iron[0], cal->hard_iron[1], cal->hard_iron[2]);
    ESP_LOGI(TAG, "Soft-iron scale:    X=%.4f Y=%.4f Z=%.4f",
             cal->soft_iron[0][0], cal->soft_iron[1][1], cal->soft_iron[2][2]);

    return ESP_OK;
}

void imu_apply_calibration(const imu_cal_t *cal, const LSM6DSV80X_Axes_t *raw_accel, const LSM6DSV80X_Axes_t *raw_gyro, imu_calibrated_t *out) {
    // Accel: convert mg -> g (no bias; EKF absorbs offset during cal phase)
    out->accel_g[0] = (float)raw_accel->x / 1000.0f;
    out->accel_g[1] = (float)raw_accel->y / 1000.0f;
    out->accel_g[2] = (float)raw_accel->z / 1000.0f;

    // Gyro: subtract bias (mdps) then convert mdps -> deg/s
    out->gyro_dps[0] = ((float)raw_gyro->x - cal->gyro_bias_mdps[0]) / 1000.0f;
    out->gyro_dps[1] = ((float)raw_gyro->y - cal->gyro_bias_mdps[1]) / 1000.0f;
    out->gyro_dps[2] = ((float)raw_gyro->z - cal->gyro_bias_mdps[2]) / 1000.0f;
}

void mag_apply_calibration(const mag_cal_t *cal, IIS2MDC_Axes_t *raw_mag, imu_calibrated_t *out)
{
    float v[3] = {(float)raw_mag->x, (float)raw_mag->x, (float)raw_mag->x};

    if (cal && cal->is_calibrated) {
        float corrected[3];
        for (int i = 0; i < 3; i++) {
            float biased = v[i] - cal->hard_iron[i];
            corrected[i] = 0;
            for (int j = 0; j < 3; j++) {
                corrected[i] += cal->soft_iron[i][j] * biased;
            }
        }
        out->mag_axes[0] = corrected[0];
        out->mag_axes[1] = corrected[1];
        out->mag_axes[2] = corrected[2];
    } else {
        out->mag_axes[0] = v[0]; 
        out->mag_axes[0] = v[1]; 
        out->mag_axes[0] = v[2];
    }

    //gMag[0] = out[0]; gMag[1] = out[1]; gMag[2] = out[2];
}

uint32_t sensor_get_tick_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void sensor_set_ground_pressure(float pressure_hpa) {
    g_ground_pressure_hpa = pressure_hpa;
    ESP_LOGI(TAG, "Ground pressure set: %.2f hPa", pressure_hpa);
}

float sensor_get_ground_pressure(void) {
    return g_ground_pressure_hpa;
}

esp_err_t baro_calibrate_ground(LPS22DF_Object_t *baro) {
    if (baro == NULL) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Sampling ground pressure (%d samples, ~%d s)...",
             PRESS_CAL_NUM_SAMPLES,
             (PRESS_CAL_NUM_SAMPLES * PRESS_CAL_SAMPLE_DELAY_MS) / 1000);

    float sum = 0.0f;
    int count = 0;
    for (int i = 0; i < PRESS_CAL_NUM_SAMPLES; i++) {
        float p = 0.0f;
        if (LPS22DF_PRESS_GetPressure(baro, &p) == LPS22DF_OK && p > 0.0f) {
            sum += p;
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(PRESS_CAL_SAMPLE_DELAY_MS));
    }

    if (count == 0) {
        ESP_LOGE(TAG, "Ground pressure cal failed — no valid samples.");
        return ESP_FAIL;
    }

    float gnd_p = sum / (float) count;

    sensor_set_ground_pressure(gnd_p);
    return ESP_OK;
}

// Slowly re-zero ground pressure to absorb LPS22DF warmup drift and slow
// atmospheric shifts. Caller must only invoke this while the rocket is
// physically on the pad (i.e. FSM in IDLE or ARMED). 
#define GROUND_TRACK_ALPHA 0.01f  // EMA coefficient (smaller = slower)

void sensor_track_ground_pressure(float pressure_hpa) {
    if (pressure_hpa <= 0.0f) return;
    g_ground_pressure_hpa = (1.0f - GROUND_TRACK_ALPHA) * g_ground_pressure_hpa
                          + GROUND_TRACK_ALPHA * pressure_hpa;
}

float sensor_get_altitude(float pressure_hpa, float temp) {

    float ground_pressure_hpa = sensor_get_ground_pressure();

    // Avoid divide-by-zero or nonsense inputs
    if (ground_pressure_hpa <= 0.0f) return 0.0f;

    // Simplified formula
    float ratio = pressure_hpa / ground_pressure_hpa;
    float altitude_m = 44330.0f * (1.0f - powf(ratio, 0.1903f)); // meters
    gAltitude = altitude_m * 3.28084f; // feet

    // Derive vertical velocity by finite-differencing altitude. EMA smooths
    // the 1 hPa pressure jitter that would otherwise create ~5 ft/s spikes.
    static int64_t prev_us = 0;
    static float   prev_alt_ft = 0.0f;
    int64_t now_us = esp_timer_get_time();
    if (prev_us != 0) {
        float dt_s = (float)(now_us - prev_us) * 1e-6f;
        if (dt_s > 1e-3f) {
            float vel_inst = (gAltitude - prev_alt_ft) / dt_s;
            const float alpha = 0.2f;
            gVerticalVelocity_fps = (1.0f - alpha) * gVerticalVelocity_fps
                                  + alpha * vel_inst;
        }
    }
    prev_us = now_us;
    prev_alt_ft = gAltitude;

    return gAltitude;
}

void sensor_update_flight_data(const imu_calibrated_t *imu) {
    float ax = imu->accel_g[0];
    float ay = imu->accel_g[1];
    float az = imu->accel_g[2];

    gAccel[0] = ax;  
    gAccel[1] = ay;  
    gAccel[2] = az;
    gGyro[0] = imu->gyro_dps[0];
    gGyro[1] = imu->gyro_dps[1];
    gGyro[2] = imu->gyro_dps[2];

    gTotalAcc = sqrtf(ax * ax + ay * ay + az * az);
    // gDegOffVert is now driven by the EKF quaternion in the IMU task.
}

esp_err_t mag_cal_save_nvs(const mag_cal_t *mag_cal)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_blob(h, "mag_cal", mag_cal, sizeof(mag_cal_t));
    if (ret == ESP_OK) ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}

esp_err_t mag_cal_load_nvs(mag_cal_t *mag_cal)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) return ret;

    size_t sz = sizeof(mag_cal_t);
    ret = nvs_get_blob(h, "mag_cal", mag_cal, &sz);
    nvs_close(h);
    return ret;
}
