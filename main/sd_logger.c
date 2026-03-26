#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include "sd_logger.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "board_config.h"

#define SD_MOUNT_POINT "/sdcard"
#define SD_LOG_QUEUE_DEPTH 64

static const char *TAG = "SD_Logger";

QueueHandle_t sd_log_queue = NULL;
static FILE *log_file = NULL;

static const char *log_type_str[] = {
    [SD_LOG_ACCEL]    = "ACCEL",
    [SD_LOG_GYRO]     = "GYRO",
    [SD_LOG_MAG]      = "MAG",
    [SD_LOG_PRESSURE] = "PRESS",
};

static int find_next_log_number(void) {
    DIR *dir = opendir(SD_MOUNT_POINT);
    if (dir == NULL) {
        return 0;
    }

    int max_num = -1;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        int num;
        if (sscanf(entry->d_name, "log_%d.csv", &num) == 1) {
            if (num > max_num) {
                max_num = num;
            }
        }
    }
    closedir(dir);
    return max_num + 1;
}

esp_err_t sd_logger_init(void) {
    // Configure SDMMC host
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = SD_CLK_GPIO_NUM;
    slot_config.cmd = SD_CMD_GPIO_NUM;
    slot_config.d0  = SD_DATA0_GPIO_NUM;
    slot_config.d1  = SD_DATA1_GPIO_NUM;
    slot_config.d2  = SD_DATA2_GPIO_NUM;
    slot_config.d3  = SD_DATA3_GPIO_NUM;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 2,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SD card not available: %s. Logging disabled.", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SD card mounted successfully");
    sdmmc_card_print_info(stdout, card);

    // Open log file with incrementing name
    int log_num = find_next_log_number();
    char filepath[64];
    snprintf(filepath, sizeof(filepath), SD_MOUNT_POINT "/log_%05d.csv", log_num);

    log_file = fopen(filepath, "w");
    if (log_file == NULL) {
        ESP_LOGE(TAG, "Failed to open log file: %s", filepath);
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
        return ESP_FAIL;
    }

    // Write CSV header
    fprintf(log_file, "timestamp_us,type,x,y,z,pressure_hpa\n");
    fflush(log_file);
    fsync(fileno(log_file));

    ESP_LOGI(TAG, "Logging to: %s", filepath);

    // Create the queue only after SD is ready
    sd_log_queue = xQueueCreate(SD_LOG_QUEUE_DEPTH, sizeof(sd_log_msg_t));
    if (sd_log_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create log queue");
        fclose(log_file);
        log_file = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void vSdLoggerTask(void *pvParameters) {
    (void)pvParameters;
    sd_log_msg_t msg;
    char line[128];
    int len;
    int flush_counter = 0;

    while (1) {
        if (xQueueReceive(sd_log_queue, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
            switch (msg.type) {
                case SD_LOG_ACCEL:
                case SD_LOG_GYRO:
                case SD_LOG_MAG:
                    len = snprintf(line, sizeof(line), "%lld,%s,%ld,%ld,%ld,\n",
                                   msg.timestamp_us, log_type_str[msg.type],
                                   msg.data.axes.x, msg.data.axes.y, msg.data.axes.z);
                    break;
                case SD_LOG_PRESSURE:
                    len = snprintf(line, sizeof(line), "%lld,%s,,,,%.4f\n",
                                   msg.timestamp_us, log_type_str[msg.type],
                                   msg.data.pressure_hpa);
                    break;
                default:
                    len = 0;
                    break;
            }

            if (len > 0) {
                fwrite(line, 1, len, log_file);
            }

            if (++flush_counter >= 10) {
                fflush(log_file);
                fsync(fileno(log_file));
                flush_counter = 0;
            }
        }
    }
}
