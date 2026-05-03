#include <math.h>
#include "sensor_mgr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

static const char *TAG = "SensorMgr";

// Expected gravity vector when board is flat, Z-up (in mg)
#define GRAVITY_MG 1000.0f

// Shared flight data globals
float gTotalAcc = 0;
float gAltitude = 0;
float gDegOffVert = 0;
float gAccel[3] = {0};
float gGyro[3] = {0};
float gMag[3] = {0};
float gPressure = 0;
float gTemperature_F = 0;
uint8_t gPyroStatus = 0;

static float SEALEVELPRESSURE_HPA = 1013.25f; // default sea level

esp_err_t imu_calibrate(LSM6DSV80X_Object_t *imu, imu_cal_t *cal) {
    if (imu == NULL || cal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cal->is_calibrated = false;

    float accel_sum[3] = {0};
    float gyro_sum[3] = {0};
    LSM6DSV80X_Axes_t accel_axes, gyro_axes;

    ESP_LOGI(TAG, "Starting IMU calibration (%d samples, ~%d seconds)...",
             IMU_CAL_NUM_SAMPLES, (IMU_CAL_NUM_SAMPLES * IMU_CAL_SAMPLE_DELAY_MS) / 1000);
    ESP_LOGI(TAG, "Keep the board stationary!");

    for (int i = 0; i < IMU_CAL_NUM_SAMPLES; i++) {
        LSM6DSV80X_ACC_GetAxes(imu, &accel_axes);
        LSM6DSV80X_GYRO_GetAxes(imu, &gyro_axes);

        accel_sum[0] += (float)accel_axes.x;
        accel_sum[1] += (float)accel_axes.y;
        accel_sum[2] += (float)accel_axes.z;

        gyro_sum[0] += (float)gyro_axes.x;
        gyro_sum[1] += (float)gyro_axes.y;
        gyro_sum[2] += (float)gyro_axes.z;

        vTaskDelay(pdMS_TO_TICKS(IMU_CAL_SAMPLE_DELAY_MS));
    }

    // Accel bias: average minus expected gravity on Z
    cal->accel_bias_mg[0] = accel_sum[0] / IMU_CAL_NUM_SAMPLES;
    cal->accel_bias_mg[1] = accel_sum[1] / IMU_CAL_NUM_SAMPLES;
    cal->accel_bias_mg[2] = accel_sum[2] / IMU_CAL_NUM_SAMPLES - GRAVITY_MG;

    // Gyro bias: average (should be near zero when stationary)
    cal->gyro_bias_mdps[0] = gyro_sum[0] / IMU_CAL_NUM_SAMPLES;
    cal->gyro_bias_mdps[1] = gyro_sum[1] / IMU_CAL_NUM_SAMPLES;
    cal->gyro_bias_mdps[2] = gyro_sum[2] / IMU_CAL_NUM_SAMPLES;

    cal->is_calibrated = true;

    ESP_LOGI(TAG, "Calibration complete.");
    ESP_LOGI(TAG, "Accel bias (mg):  X=%.2f  Y=%.2f  Z=%.2f",
             cal->accel_bias_mg[0], cal->accel_bias_mg[1], cal->accel_bias_mg[2]);
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

void imu_apply_calibration(const imu_cal_t *cal,
                           const LSM6DSV80X_Axes_t *raw_accel,
                           const LSM6DSV80X_Axes_t *raw_gyro,
                           imu_calibrated_t *out) {
    // Subtract bias (mg) then convert mg -> g
    out->accel_g[0] = ((float)raw_accel->x - cal->accel_bias_mg[0]) / 1000.0f;
    out->accel_g[1] = ((float)raw_accel->y - cal->accel_bias_mg[1]) / 1000.0f;
    out->accel_g[2] = ((float)raw_accel->z - cal->accel_bias_mg[2]) / 1000.0f;

    // Subtract bias (mdps) then convert mdps -> deg/s
    out->gyro_dps[0] = ((float)raw_gyro->x - cal->gyro_bias_mdps[0]) / 1000.0f;
    out->gyro_dps[1] = ((float)raw_gyro->y - cal->gyro_bias_mdps[1]) / 1000.0f;
    out->gyro_dps[2] = ((float)raw_gyro->z - cal->gyro_bias_mdps[2]) / 1000.0f;
}

uint32_t sensor_get_tick_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void sensor_set_ground_pressure(float pressure_hpa) {
    SEALEVELPRESSURE_HPA = pressure_hpa;
    ESP_LOGI(TAG, "Ground pressure set: %.2f hPa", SEALEVELPRESSURE_HPA);
}

float sensor_update_altitude(float pressure_hpa, float temp) {
    const float R = 287.05f;   // Specific gas constant for dry air (J/(kg·K))
    const float g = 9.80665f;  // Gravity (m/s²)

    // Convert temperature to Kelvin
    float temp_k = temp + 273.15f;

    // Hypsometric equation: altitude relative to ground reference
    float altitude_m = (R * temp_k / g) * logf(SEALEVELPRESSURE_HPA / pressure_hpa);
    gAltitude = altitude_m * 3.28084f;

    // Store pressure and temperature for SD logging
    gPressure = pressure_hpa;
    gTemperature_F = temp * 9.0f / 5.0f + 32.0f;
    return gAltitude;
}

void sensor_update_flight_data(const imu_calibrated_t *imu) {
    float ax = imu->accel_g[0];
    float ay = imu->accel_g[1];
    float az = imu->accel_g[2];

    // Store calibrated values for SD logging
    gAccel[0] = ax;  gAccel[1] = ay;  gAccel[2] = az;
    gGyro[0] = imu->gyro_dps[0];
    gGyro[1] = imu->gyro_dps[1];
    gGyro[2] = imu->gyro_dps[2];

    gTotalAcc = sqrtf(ax * ax + ay * ay + az * az);

    // Degrees off vertical: angle between accel vector and Z-axis
    if (gTotalAcc > 0.01f) {
        gDegOffVert = acosf(az / gTotalAcc) * (180.0f / (float)M_PI);
    }
    //printf("gTotalAcc: %.2f, gDegOffVert: %.2f\n", gTotalAcc, gDegOffVert);
}

// Apply in sensor_update_mag()
MagData_t sensor_update_mag(IIS2MDC_Axes_t axes, const mag_cal_t *cal)
{
    float v[3] = {(float)axes.x, (float)axes.y, (float)axes.z};
    MagData_t out = {0};

    if (cal && cal->is_calibrated) {
        float corrected[3];
        for (int i = 0; i < 3; i++) {
            float biased = v[i] - cal->hard_iron[i];
            corrected[i] = 0;
            for (int j = 0; j < 3; j++) {
                corrected[i] += cal->soft_iron[i][j] * biased;
            }
        }
        out.x = corrected[0];
        out.y = corrected[1];
        out.z = corrected[2];
    } else {
        out.x = v[0]; out.y = v[1]; out.z = v[2];
    }

    gMag[0] = out.x; gMag[1] = out.y; gMag[2] = out.z;
    return out;
}

esp_err_t cal_save_nvs(const imu_cal_t *imu_cal, const mag_cal_t *mag_cal)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    nvs_set_blob(h, "imu_cal", imu_cal, sizeof(imu_cal_t));
    nvs_set_blob(h, "mag_cal", mag_cal, sizeof(mag_cal_t));
    nvs_commit(h);
    nvs_close(h);
    return ESP_OK;
}

esp_err_t cal_load_nvs(imu_cal_t *imu_cal, mag_cal_t *mag_cal)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) return ret;

    size_t sz = sizeof(imu_cal_t);
    nvs_get_blob(h, "imu_cal", imu_cal, &sz);
    sz = sizeof(mag_cal_t);
    nvs_get_blob(h, "mag_cal", mag_cal, &sz);
    nvs_close(h);
    return ESP_OK;
}
