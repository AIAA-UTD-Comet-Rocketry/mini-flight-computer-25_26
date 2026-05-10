#include <math.h>
#include "sensor_mgr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"
#include "main.h"

static const char *TAG = "SensorMgr";

// Shared flight data globals
float gTotalAcc = 0;
float gAltitude = 0;
float gAccel[3] = {0};
float gGyro[3] = {0};
float gOrient[3] = {0};
float gVerticalVelocity_fps = 0;
uint8_t gPyroStatus = 0;

static int32_t prevTick = 0;
static int32_t prevAlt = 0;

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

// Slowly re-zero ground pressure to absorb LPS22DF warmup drift and slow
// atmospheric shifts. Caller must only invoke this while the rocket is
// physically on the pad (i.e. FSM in IDLE or ARMED). 
#define GROUND_TRACK_ALPHA 0.01f  // EMA coefficient (smaller = slower)

void sensor_track_ground_pressure(float pressure_hpa) {
    if (pressure_hpa <= 0.0f) return;
    GROUND_PRESSURE_HPA = (1.0f - GROUND_TRACK_ALPHA) * GROUND_PRESSURE_HPA
                          + GROUND_TRACK_ALPHA * pressure_hpa;
}

float sensor_get_altitude(float pressure_hpa, float temp) {

    float ground_pressure_hpa = sensor_get_ground_pressure();

    // Avoid divide-by-zero or nonsense inputs
    if (ground_pressure_hpa <= 0.0f) return 0.0f;

    // International Standard Atmosphere altitude approximation.
    float ratio = pressure_hpa / ground_pressure_hpa;
    float altitude_m = 44330.0f * (1.0f - powf(ratio, 0.1903f));
    float altitude = altitude_m * 3.28084f;

    return altitude;
}

float getVerticalVelocity(float currAlt, uint32_t currTick) {
    float vv = 0;
    vv = (prevAlt - currAlt) / ((currTick - prevTick) * 0.001f);
    prevTick = currTick;
    prevAlt = currAlt;

    return vv;
}
