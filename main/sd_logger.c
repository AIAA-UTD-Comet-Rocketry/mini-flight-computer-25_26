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

static FIL internal_log_file;
static bool file_open = false;

static const char *TAG = "SD_Logger";
static const char *file_header = 
        "timestamp_s,"
        "gyro_x,gyro_y,gyro_z,"
        "acc_x,acc_y,acc_z,"
        "mag_x,mag_y,mag_z,"
        "pressure_hpa,altitude_ft,temp_f,"
        "pyro1,pyro2,pyro3,pyro4\n";

static FILE *log_file = NULL;

// Global variables for the SD card
static sdmmc_host_t host = SDMMC_HOST_DEFAULT();
static sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
static esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 2,
    .allocation_unit_size = 16 * 1024,
};
static sdmmc_card_t *card;


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
    slot_config.width = 4;
    slot_config.clk = SD_CLK_GPIO_NUM;
    slot_config.cmd = SD_CMD_GPIO_NUM;
    slot_config.d0  = SD_DATA0_GPIO_NUM;
    slot_config.d1  = SD_DATA1_GPIO_NUM;
    slot_config.d2  = SD_DATA2_GPIO_NUM;
    slot_config.d3  = SD_DATA3_GPIO_NUM;

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
    // Required "0:" to use the 0'th device mounted, must mount to SD card before this works
    snprintf(filepath, sizeof(filepath), "0:/log_%05d.csv", log_num);

    // Using the internal FATFS layer (esp_idf/componenets/fatfs/src/ff.h)
    // This allows for error handling to happen at the application level for our software
    FRESULT sd_res = f_open(&internal_log_file, filepath, FA_OPEN_APPEND | FA_WRITE);
    if (sd_res != FR_OK) {
        ESP_LOGE(TAG, "Failed to open log file: %s", filepath);
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
        return ESP_FAIL;
    }


    // Write CSV header
    sd_write_log(
        file_header,
        (size_t)strlen(file_header)
    );

    ESP_LOGI(TAG, "Logging to: %s", filepath);

    return ESP_OK;
}

esp_err_t reset_sd() {
    
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
    ESP_LOGW(TAG, "Unmounted sdcard");
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if(ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to remount sdcard", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sd_write_log(const void* data, size_t len) {
    if(!file_open) return ESP_FAIL;

    UINT written;
    FRESULT res = f_write(&internal_log_file, data, len, &written);
    if(res != FR_OK || written != len) {
        ESP_LOGE(TAG, "f_write failed: res=%d written=%u", res, written);
        reset_sd();
        return ESP_FAIL;
    }

    res = f_sync(&internal_log_file);
    if(res != FR_OK) {
        ESP_LOGE(TAG, "f_sync failed: %d", res);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t write_packet(SensorDataPacket_t packet) {
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
        sd_write_log(line, (size_t)len);
    }

    return ESP_OK;
}
