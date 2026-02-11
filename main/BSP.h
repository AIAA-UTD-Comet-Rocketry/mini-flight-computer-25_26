/*
 * File: BSP.h
 * 
 * This file is the top of the BSP (Board Support Package) for the mini flight computer.
 * It contains the necessary includes and definitions for application level code to
 * interact with hardware connected to the ESP32-S3 module.
 * 
 * Set configurations structs, call initialization functions, and procede with calling
 * functions during runtime processes.
 * 
 * Created: 2025
 */


#ifndef _BSP_H_
#define _BSP_H_

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "iis2mdc.h"
#include "lps22df.h"
#include "lsm6dsv80x.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

// Include peripheral mappings
#include "board_config.h"

// Type Definitions
typedef enum {
    led_status,
    led_can_tx,
    led_can_rx,
    led_sd_tx,
    led_sd_rx,
    led_max
} led_index_t;

typedef enum {
    apo1_channel,
    apo2_channel,
    main1_channel,
    main2_channel,
    pyro_channel_max
} pyro_index_t;

typedef struct {
    uint32_t (*Bsp_GetTick)(void);
    void     (*Bsp_Delay)(uint32_t);
} bsp_config_t;

typedef struct {
    i2c_master_dev_handle_t *iis2mdc_i2c_handle;
    i2c_master_dev_handle_t *lps22df_i2c_handle;
    i2c_master_dev_handle_t *lsm6dsv80x_i2c_handle;
    twai_node_handle_t      *can_node_hdl;
    IIS2MDC_Object_t        *iis2mdc_handle;
    LPS22DF_Object_t        *lps22df_handle;
    LSM6DSV80X_Object_t     *lsm6dsv80x_handle;
    gpio_num_t               led_gpio_nums[led_max];
    gpio_num_t               pyro_gpio_nums[pyro_channel_max];
    uint32_t (*Bsp_GetTick)(void);
    void     (*Bsp_Delay)(uint32_t);
    bool isInitialized;
} bsp_object_t;
typedef bsp_object_t * board_handle_t;


// External variable declarations
extern i2c_master_bus_handle_t imu_bus_handle;
extern i2c_master_bus_handle_t press_bus_handle;

// Function Prototypes
esp_err_t bsp_init(board_handle_t *handle, bsp_config_t *bsp_init_obj);

#endif // _BSP_H_