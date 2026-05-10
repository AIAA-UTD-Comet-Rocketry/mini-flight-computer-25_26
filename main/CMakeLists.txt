idf_component_register(SRCS "imu_calibration.c" "sd_logger.c" "sensor_mgr.c"
                            "main.c" "cmd_i2ctools.c" "BSP.c" "gpio_mgr.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES "esp_driver_gpio"
                                "esp_driver_i2c"
                                "esp_driver_twai"
                                "esp_driver_ledc"
                                "esp_driver_sdmmc"
                                "esp_timer"
                                "espcoredump"

                       PRIV_REQUIRES IIS2MDC LPS22DF LSM6DSV80X CanAero FlightFSM Fusion fatfs nvs_flash)