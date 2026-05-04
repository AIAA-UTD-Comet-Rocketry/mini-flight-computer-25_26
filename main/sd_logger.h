#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "esp_err.h"
#include "sensor_mgr.h"

typedef struct {
    imu_calibrated_t imu;
    AltData_t alt;
} SensorDataPacket_t;

esp_err_t sd_logger_init(void);
esp_err_t write_packet(SensorDataPacket_t packet);
esp_err_t sd_write_log(const void* data, size_t len);

// True iff the log file is open and writes will land on the SD card.
bool sd_logger_is_active(void);

#endif // SD_LOGGER_H
