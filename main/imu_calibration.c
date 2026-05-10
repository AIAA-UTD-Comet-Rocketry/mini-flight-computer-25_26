#include "imu_calibration.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

// ----------------------------------------------------------------
// NVS helpers
// ----------------------------------------------------------------

esp_err_t cal_nvs_save(const char *key, const FusionVector *vec)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_blob(h, key, vec, sizeof(FusionVector));
    if (ret == ESP_OK) ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}

esp_err_t cal_nvs_load(const char *key, FusionVector *vec)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) return ret;

    size_t sz = sizeof(FusionVector);
    ret = nvs_get_blob(h, key, vec, &sz);
    nvs_close(h);
    return ret;
}

// ----------------------------------------------------------------
// Serial helpers
// ----------------------------------------------------------------

// Read one byte from the console UART, blocking until one arrives.
// Uses uart_read_bytes directly so it works regardless of VFS state.
static int serial_getchar(void)
{
    uint8_t c;
    while (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &c, 1, pdMS_TO_TICKS(100)) < 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return (int)c;
}

static void wait_for_enter(void)
{
    printf("  >> Press ENTER when ready...\r\n");
    int c;
    do {
        c = serial_getchar();
    } while (c != '\n' && c != '\r');
    // Drain any trailing byte (e.g. \n after \r) so next read starts clean.
    uint8_t dummy;
    uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &dummy, 1, pdMS_TO_TICKS(50));
}

// Drain all buffered bytes within a short window.
static void drain_line(void)
{
    uint8_t buf[64];
    while (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(200)) > 0) {}
}

// ----------------------------------------------------------------
// Sample collection (uses LSM6DSV80X driver directly)
// ----------------------------------------------------------------

static void collect_accel_avg(LSM6DSV80X_Object_t *imu, float avg[3])
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    LSM6DSV80X_Axes_t axes;

    printf("  Collecting %d samples...\r\n", CAL_NUM_SAMPLES);

    for (int n = 0; n < CAL_NUM_SAMPLES; n++) {
        LSM6DSV80X_ACC_GetAxes(imu, &axes);
        sum[0] += (float)axes.x / 1000.0f;  // mg -> g
        sum[1] += (float)axes.y / 1000.0f;
        sum[2] += (float)axes.z / 1000.0f;
        vTaskDelay(pdMS_TO_TICKS(CAL_SAMPLE_DELAY_MS));
    }

    avg[0] = sum[0] / (float)CAL_NUM_SAMPLES;
    avg[1] = sum[1] / (float)CAL_NUM_SAMPLES;
    avg[2] = sum[2] / (float)CAL_NUM_SAMPLES;

    printf("  Averaged: [%.5f, %.5f, %.5f] g\r\n", avg[0], avg[1], avg[2]);
}

static void collect_gyro_avg(LSM6DSV80X_Object_t *imu, float avg[3])
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    LSM6DSV80X_Axes_t axes;

    printf("  Collecting %d samples...\r\n", CAL_NUM_SAMPLES);

    for (int n = 0; n < CAL_NUM_SAMPLES; n++) {
        LSM6DSV80X_GYRO_GetAxes(imu, &axes);
        sum[0] += (float)axes.x / 1000.0f;  // mdps -> dps
        sum[1] += (float)axes.y / 1000.0f;
        sum[2] += (float)axes.z / 1000.0f;
        vTaskDelay(pdMS_TO_TICKS(CAL_SAMPLE_DELAY_MS));
    }

    avg[0] = sum[0] / (float)CAL_NUM_SAMPLES;
    avg[1] = sum[1] / (float)CAL_NUM_SAMPLES;
    avg[2] = sum[2] / (float)CAL_NUM_SAMPLES;

    printf("  Averaged: [%.5f, %.5f, %.5f] dps\r\n", avg[0], avg[1], avg[2]);
}

// ----------------------------------------------------------------
// 6-position accelerometer calibration
// ----------------------------------------------------------------
//
//   offset[i]      = (pos + neg) / 2       (zero-g bias)
//   sensitivity[i] = 2.0 / (pos - neg)     (scale correction)

static const char *position_labels[6] = {
    "Z-UP   : lay flat, chip facing UP",
    "Z-DOWN : lay flat, chip facing DOWN",
    "X-UP   : on edge, X axis pointing UP",
    "X-DOWN : on edge, X axis pointing DOWN",
    "Y-UP   : on edge, Y axis pointing UP",
    "Y-DOWN : on edge, Y axis pointing DOWN",
};

static bool do_accel_calibration(LSM6DSV80X_Object_t *imu,
                                  FusionVector *offset_out,
                                  FusionVector *sens_out)
{
    float samples[6][3];

    printf("\r\n========================================\r\n");
    printf(" ACCELEROMETER 6-POSITION CALIBRATION\r\n");
    printf("========================================\r\n");
    printf(" Hold the sensor STILL in each position\r\n");
    printf(" for ~5 seconds while samples collect.\r\n");
    printf("========================================\r\n\r\n");

    for (int pos = 0; pos < 6; pos++) {
        printf("[%d/6] %s\r\n", pos + 1, position_labels[pos]);
        wait_for_enter();
        collect_accel_avg(imu, samples[pos]);
        printf("  Done.\r\n\r\n");
    }

    // X: positions 2(+1g) and 3(-1g), X component
    float x_p = samples[2][0], x_n = samples[3][0];
    // Y: positions 4(+1g) and 5(-1g), Y component
    float y_p = samples[4][1], y_n = samples[5][1];
    // Z: positions 0(+1g) and 1(-1g), Z component
    float z_p = samples[0][2], z_n = samples[1][2];

    offset_out->axis.x = (x_p + x_n) / 2.0f;
    offset_out->axis.y = (y_p + y_n) / 2.0f;
    offset_out->axis.z = (z_p + z_n) / 2.0f;

    sens_out->axis.x = 2.0f / (x_p - x_n);
    sens_out->axis.y = 2.0f / (y_p - y_n);
    sens_out->axis.z = 2.0f / (z_p - z_n);

    printf("========================================\r\n");
    printf(" RESULTS\r\n");
    printf("========================================\r\n");
    printf(" Offset:      [%+.6f, %+.6f, %+.6f] g\r\n",
           offset_out->axis.x, offset_out->axis.y, offset_out->axis.z);
    printf(" Sensitivity: [%.6f, %.6f, %.6f]\r\n",
           sens_out->axis.x, sens_out->axis.y, sens_out->axis.z);

    // Sanity checks
    bool ok = true;
    float off[3] = {offset_out->axis.x, offset_out->axis.y, offset_out->axis.z};
    float sen[3] = {sens_out->axis.x, sens_out->axis.y, sens_out->axis.z};
    for (int i = 0; i < 3; i++) {
        if (fabsf(off[i]) > 0.15f) {
            printf(" WARNING: axis %d offset %.4f g seems large\r\n", i, off[i]);
            ok = false;
        }
        if (fabsf(sen[i] - 1.0f) > 0.05f) {
            printf(" WARNING: axis %d sensitivity %.4f far from 1.0\r\n", i, sen[i]);
            ok = false;
        }
    }
    if (ok) printf(" All values look reasonable.\r\n");
    printf("========================================\r\n\r\n");

    if (cal_nvs_save("accel_off", offset_out) == ESP_OK &&
        cal_nvs_save("accel_sens", sens_out) == ESP_OK) {
        printf(" Saved to NVS.\r\n");
    } else {
        printf(" ERROR: NVS write failed!\r\n");
        return false;
    }
    return true;
}

// ----------------------------------------------------------------
// Gyroscope bias calibration
// ----------------------------------------------------------------

static bool do_gyro_calibration(LSM6DSV80X_Object_t *imu,
                                 FusionVector *offset_out)
{
    printf("\r\n========================================\r\n");
    printf(" GYROSCOPE BIAS CALIBRATION\r\n");
    printf("========================================\r\n");
    printf(" Place sensor on a flat, stable surface.\r\n");
    printf(" Do NOT touch it during collection.\r\n");
    printf("========================================\r\n\r\n");

    wait_for_enter();

    float avg[3];
    collect_gyro_avg(imu, avg);

    offset_out->axis.x = avg[0];
    offset_out->axis.y = avg[1];
    offset_out->axis.z = avg[2];

    printf("========================================\r\n");
    printf(" RESULTS\r\n");
    printf("========================================\r\n");
    printf(" Gyro bias: [%+.4f, %+.4f, %+.4f] dps\r\n",
           offset_out->axis.x, offset_out->axis.y, offset_out->axis.z);

    for (int i = 0; i < 3; i++) {
        float v = fabsf((&offset_out->axis.x)[i]);
        if (v > 5.0f) {
            printf(" WARNING: axis %d bias %.2f dps seems large\r\n", i, v);
        }
    }
    printf("========================================\r\n\r\n");

    if (cal_nvs_save("gyro_off", offset_out) == ESP_OK) {
        printf(" Saved to NVS.\r\n");
    } else {
        printf(" ERROR: NVS write failed!\r\n");
        return false;
    }
    return true;
}

// ----------------------------------------------------------------
// Interactive menu
// ----------------------------------------------------------------

esp_err_t calibration_run_menu(
    LSM6DSV80X_Object_t *imu,
    FusionVector *accel_offset,
    FusionVector *accel_sensitivity,
    FusionVector *gyro_offset)
{
    // Install the UART driver so uart_read_bytes works for interactive input.
    // ESP_ERR_INVALID_STATE means it was already installed — that's fine.
    esp_err_t uart_ret = uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 1024, 0, 0, NULL, 0);
    if (uart_ret != ESP_OK && uart_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("CAL", "uart_driver_install failed: %s", esp_err_to_name(uart_ret));
        return ESP_FAIL;
    }
    setvbuf(stdout, NULL, _IONBF, 0);

    bool did_something = false;

    while (1) {
        printf("\r\n");
        printf("========================================\r\n");
        printf(" IMU CALIBRATION MENU\r\n");
        printf("========================================\r\n");
        printf("  1) Accelerometer (6-position)\r\n");
        printf("  2) Gyroscope (stationary bias)\r\n");
        printf("  3) Exit and continue to flight mode\r\n");
        printf("========================================\r\n");
        printf(" Select [1-3]: \n");
        fflush(stdout);

        int choice = serial_getchar();
        printf("%c\r\n", (char)choice);
        drain_line();

        switch (choice) {
        case '1':
            if (do_accel_calibration(imu, accel_offset, accel_sensitivity)) {
                did_something = true;
            }
            break;

        case '2':
            if (do_gyro_calibration(imu, gyro_offset)) {
                did_something = true;
            }
            break;

        case '3':
            printf("\r\n Exiting calibration. Continuing boot...\r\n\r\n");
            return did_something ? ESP_OK : ESP_ERR_NOT_FINISHED;

        default:
            printf(" Invalid choice. Try again.\r\n");
            break;
        }
    }
}