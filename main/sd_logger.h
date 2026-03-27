#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "esp_err.h"
#include "sensor_mgr.h"

typedef struct {
    imu_calibrated_t imu;
    AltData_t alt;
    MagData_t mag;
} SensorDataPacket_t;

esp_err_t sd_logger_init(void);
esp_err_t write_log(SensorDataPacket_t packet, int* write_counter);

#endif // SD_LOGGER_H
