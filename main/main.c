/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "BSP.h"
#include "gpio_mgr.h"

static const char *TAG = "Main";


// Tasks
void vImuHandlerTask(void *pvParameters);
void vMagHandlerTask(void *pvParameters);
// Helpers
#define CHECK_TASK_CREATION(ret, err_msg) \
    if((ret) != pdPASS) { \
        ESP_LOGI(TAG, err_msg); \
    }

void app_main(void) {
    (void)TAG; // Stop compile warnings, unused debug variables are not a concern

    ///////////////////////////////
    /// Configuration
    ////////////////////////////////
    board_handle_t mini_fc_handle;

    bsp_config_t bsp_init_cfg = {
        .Bsp_GetTick = xTaskGetTickCount,
        .Bsp_Delay = vTaskDelay
    };

    //ESP_ERROR_CHECK(bsp_init(&bsp_init_cfg));
    bsp_init(&mini_fc_handle, &bsp_init_cfg);


    ///////////////////////////////
    /// Task Creation
    ////////////////////////////////
    BaseType_t task_ret;
    TaskHandle_t xImuTaskHandle, xMagTaskHandle;

    task_ret = xTaskCreate(vImuHandlerTask, 
                           "IMU Data Collection", 
                           configMINIMAL_STACK_SIZE, 
                           (void*) mini_fc_handle->lsm6dsv80x_handle, 
                           1, 
                           &xImuTaskHandle);
    CHECK_TASK_CREATION(task_ret, "IMU task failed to create!");

    task_ret = xTaskCreate(vMagHandlerTask,
                           "Magnetometer Data Collection",
                           configMINIMAL_STACK_SIZE,
                           (void*) mini_fc_handle->iis2mdc_handle,
                           1,
                           &xMagTaskHandle);
    CHECK_TASK_CREATION(task_ret, "Magnetometer task failed to create!");


    ///////////////////////////////
    /// Start Scheduler
    ////////////////////////////////
    vTaskStartScheduler();



    
    // xTaskCreate((TaskFunction_t)LED_Task, "LED MGR", 4096, (void *)&mini_fc_handle, 0, NULL);
    // vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for everything to settle (TODO event based wait)

    // // drive led_status with pattern
    // LED_setPattern(led_status, pattern_burst);

    // IIS2MDC_Axes_t next_axis;
    // for(;;){
    //     IIS2MDC_MAG_GetAxes(mini_fc_handle->iis2mdc_handle, &next_axis);
    //     ESP_LOGI(TAG, "Mag X: %d, Mag Y: %d, Mag Z: %d", next_axis.x, next_axis.y, next_axis.z);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    /*
    The calibrated hard and soft iron can be represented with an ofset followed by a matrix
    to translate by rotation and scale.

    m = [c0, c1][m1-o1]
        [c2, c3][m2-o2]
    */
}

void vImuHandlerTask(void *pvParameters) {
    // Initialize the IMU object
    LSM6DSV80X_Object_t* imu = (LSM6DSV80X_Object_t*)pvParameters;
    LSM6DSV80X_Axes_t accel_axes;
    while(1) {
        LSM6DSV80X_ACC_GetAxes(imu, &accel_axes);
        ESP_LOGI(TAG, "Accel X: %ld, Accel Y: %ld, Accel Z: %ld", accel_axes.x, accel_axes.y, accel_axes.z);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
void vMagHandlerTask(void *pvParameters) {
    IIS2MDC_Object_t* mag = (IIS2MDC_Object_t*)pvParameters;
    IIS2MDC_Axes_t next_axis;
    while(1) {
        IIS2MDC_MAG_GetAxes(mag, &next_axis);
        ESP_LOGI(TAG, "Mag X: %d, Mag Y: %d, Mag Z: %d", next_axis.x, next_axis.y, next_axis.z);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
