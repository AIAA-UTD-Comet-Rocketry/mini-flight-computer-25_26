#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include "sd_logger.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_timer.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "board_config.h"

#define SD_MOUNT_POINT "/sdcard"
#define SD_LOG_PERIOD_MS 500

static const char *TAG = "SD_Logger";

static FILE *log_file = NULL;

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
    fprintf(log_file,
        "timestamp_s,"
        "gyro_x,gyro_y,gyro_z,"
        "acc_x,acc_y,acc_z,"
        "mag_x,mag_y,mag_z,"
        "pressure_hpa,altitude_ft,temp_f,"
        "drogue1,drogue2,main1,main2\n");
    fflush(log_file);
    fsync(fileno(log_file));

    ESP_LOGI(TAG, "Logging to: %s", filepath);

    return ESP_OK;
}

esp_err_t write_log(SensorDataPacket_t packet, int* write_counter) {
    char line[256]; int len;
    if (log_file == NULL) return ESP_FAIL;

    float timestamp_s = (float)(esp_timer_get_time() / 1000000.0);

    len = snprintf(line, 256,
        "%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.0f,%.0f,%.0f,"
        "%.4f,%.2f,%.1f,"
        "%c,%c,%c,%c\n",
        timestamp_s,
        packet.imu.accel_g[0], packet.imu.accel_g[1], packet.imu.accel_g[2],
        packet.imu.gyro_dps[0], packet.imu.gyro_dps[1], packet.imu.gyro_dps[2],
        packet.mag.x, packet.mag.y, packet.mag.z,
        packet.alt.pressure, packet.alt.altitude, packet.alt.temp,
        (gPyroStatus & (1 << 0)) ? 'Y' : 'N',
        (gPyroStatus & (1 << 1)) ? 'Y' : 'N',
        (gPyroStatus & (1 << 2)) ? 'Y' : 'N',
        (gPyroStatus & (1 << 3)) ? 'Y' : 'N');

    if (len > 0) {
        fwrite(line, 1, len, log_file);
    }

    if ((*write_counter)++ >= 10) {
        fflush(log_file);
        fsync(fileno(log_file));
        *write_counter = 0;
    }
    return ESP_OK;
}
