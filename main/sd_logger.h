#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "esp_err.h"

esp_err_t sd_logger_init(void);
void vSdLoggerTask(void *pvParameters);

#endif // SD_LOGGER_H
