#include <math.h>
#include "sensor_mgr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"
#include "main.h"
#include "dsps_biquad.h"

static const char *TAG = "SensorMgr";
static portMUX_TYPE vel_mux = portMUX_INITIALIZER_UNLOCKED;
static float GROUND_PRESSURE_HPA = 1013.25f; // default sea level pressure
static float gVerticalVelocity = 0.0f;   // complementary-filtered, ft/s

// 2nd-order Butterworth low-pass, fc=5 Hz, fs=100 Hz.
// Coefficients from scipy.signal.butter(N=2, Wn=5.0, fs=100.0).
// DC gain = 1.0000000 (verified). Poles at |z|=0.8008 (stable).
static float s_press_lpf_coef[5] = {
    0.020083366f,   // b0
    0.040166731f,   // b1
    0.020083366f,   // b2
   -1.561018076f,   // a1
    0.641351538f    // a2
};
static float s_press_lpf_w[2] = {0.0f, 0.0f};  // delay-line state

// Shared flight data globals
float gTotalAcc = 0;
float gAltitude = 0;
float gAccel[3] = {0};
float gGyro[3] = {0};
float gOrient[3] = {0};
float gVerticalVelocity_fps = 0;
uint8_t gPyroStatus = 0;

float sensor_pressure_filter(float raw_hpa) {
    float out = 0.0f;
    dsps_biquad_f32(&raw_hpa, &out, 1, s_press_lpf_coef, s_press_lpf_w);
    return out;
}

void sensor_set_ground_pressure(float pressure_hpa) {
    GROUND_PRESSURE_HPA = pressure_hpa;
    ESP_LOGI(TAG, "Ground pressure set: %.2f hPa", pressure_hpa);
}

float sensor_get_ground_pressure(void) {
    return GROUND_PRESSURE_HPA;
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

float sensor_get_altitude(float pressure_hpa, float temp) {
    // International Standard Atmosphere altitude approximation.
    float ratio = pressure_hpa / SEA_LEVEL_PRESSURE_HPA;
    float altitude_m = 44330.0f * (1.0f - powf(ratio, 0.1903f));
    float altitude_ft = altitude_m * 3.28084f;

    return altitude_ft;
}

void sensor_velocity_predict(float earth_z_g, float dt_s) {
    float accel_fps2 = earth_z_g * 32.174f;  // g -> ft/s^2

    portENTER_CRITICAL(&vel_mux);
    gVerticalVelocity += accel_fps2 * dt_s;
    portEXIT_CRITICAL(&vel_mux);
}

// Called from baro task at 100 Hz.
// Pulls integrated velocity toward baro-derived velocity.
// alpha controls the blend: small = trust accel more (fast response),
// large = trust baro more (less drift).
#define COMP_ALPHA 0.02f

void sensor_velocity_correct(float baro_altitude_ft, uint32_t tick_ms) {
    static float prev_alt = 0.0f;
    static uint32_t prev_tick = 0;
    static bool initialized = false;

    if (!initialized) {
        prev_alt = baro_altitude_ft;
        prev_tick = tick_ms;
        initialized = true;
        return;
    }

    uint32_t dt_ms = tick_ms - prev_tick;
    if (dt_ms == 0) return;

    float baro_vel = (baro_altitude_ft - prev_alt) / ((float)dt_ms * 0.001f);
    prev_alt = baro_altitude_ft;
    prev_tick = tick_ms;

    // Nudge integrated velocity toward baro velocity
    portENTER_CRITICAL(&vel_mux);
    gVerticalVelocity += COMP_ALPHA * (baro_vel - gVerticalVelocity);
    portEXIT_CRITICAL(&vel_mux);
}

float sensor_get_vertical_velocity(void) {
    float v;
    portENTER_CRITICAL(&vel_mux);
    v = gVerticalVelocity;
    portEXIT_CRITICAL(&vel_mux);
    return v;
}
