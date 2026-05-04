#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "esp_err.h"
#include "sensor_mgr.h"
#include "attitude_ekf.h"
#include "FlightFSM.h"

typedef struct {
    imu_calibrated_t imu;
    AltData_t alt;
} SensorDataPacket_t; // TODO: depreciate this

typedef struct {
    float timestamp_s;         // sec
    Accel_Axes_t accel;       // g
    Gyro_Axes_t gyro;       // dps
    AltData_t baro;
    attitude_t attitude;
    State flightState;
    uint8_t pyroStatus;
    float gVerticalVelocity;
    float gTotalAcc;                // gravity vector
    float gAccelVelocity;           // m/s^2, gravity removed
} LogSensorRecord_t; //unused currently

typedef struct {
    // TODO: Capture flight transition state timestamp
} LogEventRecord_t;


esp_err_t sd_logger_init(void);
esp_err_t write_packet(LogSensorRecord_t packet);
esp_err_t sd_write_log(const void* data, size_t len);

// True iff the log file is open and writes will land on the SD card.
bool sd_logger_is_active(void);

#endif // SD_LOGGER_H
