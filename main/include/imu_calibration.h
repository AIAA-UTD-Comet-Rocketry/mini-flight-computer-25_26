#ifndef _IMU_CALIBRATION_H_
#define _IMU_CALIBRATION_H_

#include "esp_err.h"
#include "Fusion.h"
#include "BSP.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAL_NVS_NAMESPACE   "imu_cal"
#define CAL_NUM_SAMPLES     500      // samples per position (~5s at 100 Hz)
#define CAL_SAMPLE_DELAY_MS 10       // 100 Hz collection rate
#define CAL_REPOSITION_S    5        // seconds to reposition before each collection

/**
 * @brief Run the interactive serial calibration menu.
 *
 * Blocks until the user selects "exit and continue".
 * Calibrated parameters are saved to NVS and written
 * into the Fusion vectors passed by pointer so the
 * caller can use them immediately without a reboot.
 *
 * @param[in]  imu               LSM6DSV80X sensor handle
 * @param[out] accel_offset      Accelerometer offset (written on success)
 * @param[out] accel_sensitivity Accelerometer sensitivity (written on success)
 * @param[out] gyro_offset       Gyroscope offset (written on success)
 * @return ESP_OK if at least one calibration completed, ESP_ERR_* otherwise
 */
esp_err_t calibration_run_menu(
    LSM6DSV80X_Object_t *imu,
    FusionVector *accel_offset,
    FusionVector *accel_sensitivity,
    FusionVector *gyro_offset
);

/**
 * @brief Save a FusionVector to NVS under the given key.
 */
esp_err_t cal_nvs_save(const char *key, const FusionVector *vec);

/**
 * @brief Load a FusionVector from NVS under the given key.
 */
esp_err_t cal_nvs_load(const char *key, FusionVector *vec);

#ifdef __cplusplus
}
#endif

#endif // _IMU_CALIBRATION_H_