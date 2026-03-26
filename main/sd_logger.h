#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    SD_LOG_ACCEL,
    SD_LOG_GYRO,
    SD_LOG_MAG,
    SD_LOG_PRESSURE,
} sd_log_type_t;

typedef struct {
    sd_log_type_t type;
    int64_t timestamp_us;
    union {
        struct { int32_t x, y, z; } axes;
        float pressure_hpa;
    } data;
} sd_log_msg_t;

extern QueueHandle_t sd_log_queue;

esp_err_t sd_logger_init(void);
void vSdLoggerTask(void *pvParameters);

#endif // SD_LOGGER_H
