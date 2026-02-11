/*
 * File: BSP.c
 * 
 * This file is the top of the BSP (Board Support Package) for the mini flight computer.
 * It contains the necessary includes and definitions for application level code to
 * interact with hardware connected to the ESP32-S3 module.
 * 
 * The primary purpose of this file is to initialize hardware components as well as
 * starting handler RTOS tasks. 
 * 
 * These functions consolidate interactions with peripheral devices by keeping links
 * to all current component drivers. Links to component drivers can be replaced as needed
 * especcially when supporting sensors that are drop in replacements for each other.
 * 
 * Created: 2025
 */

#include "BSP.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"

static const char *TAG = "BSP";

// Global handles
i2c_master_bus_handle_t imu_bus_handle;
i2c_master_bus_handle_t press_bus_handle;

i2c_master_dev_handle_t iis2mdc_i2c_handle;
i2c_master_dev_handle_t lps22df_i2c_handle;
i2c_master_dev_handle_t lsm6dsv80x_i2c_handle;

twai_node_handle_t can_node_hdl;

IIS2MDC_Object_t iis2mdc_handle;
LPS22DF_Object_t lps22df_handle;
LSM6DSV80X_Object_t lsm6dsv80x_handle;

bsp_object_t bsp_handle = {
    .iis2mdc_i2c_handle    = &iis2mdc_i2c_handle,
    .lps22df_i2c_handle    = &lps22df_i2c_handle,
    .lsm6dsv80x_i2c_handle = &lsm6dsv80x_i2c_handle,
    .can_node_hdl          = &can_node_hdl,
    .iis2mdc_handle        = &iis2mdc_handle,
    .lps22df_handle        = &lps22df_handle,
    .lsm6dsv80x_handle     = &lsm6dsv80x_handle,
    .led_gpio_nums         = { LED_STATUS_PIN, LED_CAN_TX_PIN, LED_CAN_RX_PIN,
                               LED_SD_TX_PIN, LED_SD_RX_PIN },
    .pyro_gpio_nums        = { PYRO_APO_1_PIN, PYRO_APO_2_PIN, PYRO_MAIN_1_PIN, PYRO_MAIN_2_PIN },
    .isInitialized         = false
};

uint8_t g_i2c_initialized = 0;

// Local function defines
void bsp_toggle_status_led( void );


/*---------------------------------------------------------------------
 * Function: bsp_les_init
 * 
 * Description: Configures GPIOs for on board LEDs.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: LEDS initialized to off state.
 *---------------------------------------------------------------------*/
static esp_err_t bsp_led_init(void)
{
    // Set LEDs to be off after configuration as output
    gpio_set_level(LED_STATUS_GPIO_NUM, 0);
    gpio_set_level(LED_CAN_TX_GPIO_NUM, 0);
    gpio_set_level(LED_CAN_RX_GPIO_NUM, 0);
    gpio_set_level(LED_SD_TX_GPIO_NUM, 0);
    gpio_set_level(LED_SD_RX_GPIO_NUM, 0);

    gpio_config_t led_io_conf = {
        .pin_bit_mask = LED_STATUS_GPIO_MASK | LED_CAN_TX_GPIO_MASK | LED_CAN_RX_GPIO_MASK |
                        LED_SD_TX_GPIO_MASK | LED_SD_RX_GPIO_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&led_io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED init fail: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "LEDs configured successfully.");
    return ESP_OK;
}


/*---------------------------------------------------------------------
 * Function: bsp_pyro_init
 * 
 * Description: Configures GPIOs for Pyro channels prior to use.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: Pyro channels initialized to off state.
 *---------------------------------------------------------------------*/
static esp_err_t bsp_pyro_init(void)
{
    // Set Pyro channels to be off after configuration as output
    gpio_set_level(PYRO_APO_1_GPIO_NUM, 0);
    gpio_set_level(PYRO_APO_2_GPIO_NUM, 0);
    gpio_set_level(PYRO_MAIN_1_GPIO_NUM, 0);
    gpio_set_level(PYRO_MAIN_2_GPIO_NUM, 0);

    gpio_config_t pyro_io_conf = {
        .pin_bit_mask = PYRO_APO_1_GPIO_MASK | PYRO_APO_2_GPIO_MASK |
                        PYRO_MAIN_1_GPIO_MASK | PYRO_MAIN_2_GPIO_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&pyro_io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Pyro channel init fail: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Pyro channels configured successfully.");
    return ESP_OK;
}


/*---------------------------------------------------------------------
 * Function: bsp_I2C_init
 * 
 * Description: Configures I2C buses for data collection.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: bus handles are stored in global variables.
 *---------------------------------------------------------------------*/
static esp_err_t bsp_I2C_init(void)
{
    esp_err_t ret;

    i2c_master_bus_config_t i2c_imu_bus_conf = {
        .i2c_port = I2C_IMU_PORT,
        .sda_io_num = I2C_IMU_SDA_GPIO_NUM,
        .scl_io_num = I2C_IMU_SCL_GPIO_NUM,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = I2C_IMU_INTERNAL_PULLUPS
    };
    ret = i2c_new_master_bus(&i2c_imu_bus_conf, &imu_bus_handle);

    i2c_master_bus_config_t i2c_press_bus_conf = {
        .i2c_port = I2C_PRESS_PORT,
        .sda_io_num = I2C_PRESS_SDA_GPIO_NUM,
        .scl_io_num = I2C_PRESS_SCL_GPIO_NUM,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = I2C_PRESS_INTERNAL_PULLUPS
    };
    ret = i2c_new_master_bus(&i2c_press_bus_conf, &press_bus_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C init fail: %s", esp_err_to_name(ret));
        return ret;
    }

    g_i2c_initialized = true;
    ESP_LOGI(TAG, "I2C buses initialized successfully.");
    return ESP_OK;
}


/*---------------------------------------------------------------------
 * Function: bsp_check_I2C_init
 * 
 * Description: Access the flag marking I2C initialization.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: must be called after the I2C port is initialized.
 *---------------------------------------------------------------------*/
static esp_err_t bsp_check_I2C_init(void)
{
    return g_i2c_initialized ? ESP_OK : ESP_ERR_INVALID_STATE;
}


/*---------------------------------------------------------------------
 * Function: bsp_I2C_readWrap
 * 
 * Description: Callback function from ST sensor drivers to read data
 *              over the I2C bus.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 *---------------------------------------------------------------------*/
static esp_err_t bsp_I2C_readWrap(uint16_t address, uint16_t reg, uint8_t *pdata, uint16_t len)
{
    i2c_master_dev_handle_t* targetbus;

    // Grab the desired destination device from the address
    switch(address)
    {
        case SENSOR_IIS2MDC_I2C_ADDRESS:
            targetbus = &iis2mdc_i2c_handle;
            break;
        case SENSOR_LPS22DF_I2C_ADDRESS:
            targetbus = &lps22df_i2c_handle;
            break;
        case SENSOR_LSM6DSV80X_I2C_ADDRESS:
            targetbus = &lsm6dsv80x_i2c_handle;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_buf[1];
    tx_buf[0] = (uint8_t)reg;

    i2c_master_transmit_receive(*targetbus, tx_buf, 1, pdata, len, -1);

    return ESP_OK;
}


/*---------------------------------------------------------------------
 * Function: bsp_I2C_writeWrap
 * 
 * Description: Callback function from ST sensor drivers to write data
 *              over the I2C bus.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 *---------------------------------------------------------------------*/
static esp_err_t bsp_I2C_writeWrap(uint16_t address, uint16_t reg, uint8_t *pdata, uint16_t len)
{
    i2c_master_dev_handle_t* targetbus;

    // Grab the desired destination device from the address
    switch(address)
    {
        case SENSOR_IIS2MDC_I2C_ADDRESS:
            targetbus = &iis2mdc_i2c_handle;
            break;
        case SENSOR_LPS22DF_I2C_ADDRESS:
            targetbus = &lps22df_i2c_handle;
            break;
        case SENSOR_LSM6DSV80X_I2C_ADDRESS:
            targetbus = &lsm6dsv80x_i2c_handle;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    uint8_t wr_buffer[len + 1];
    wr_buffer[0] = (uint8_t)reg;
    memcpy(wr_buffer + 1, pdata, len);

    i2c_master_transmit(*targetbus, wr_buffer, len + 1, -1);

    return ESP_OK;
}


/*---------------------------------------------------------------------
 * Function: bsp_iis2mdc_init
 * 
 * Description: Configures IIS2MDC magnetometer through the I2C bus.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: must be called after the I2C port is initialized.
 *---------------------------------------------------------------------*/
static esp_err_t bsp_iis2mdc_init(void)
{
    if(i2c_master_probe(press_bus_handle, SENSOR_IIS2MDC_I2C_ADDRESS, 50) != ESP_OK)
    {
        ESP_LOGE(TAG, "IIS2MDC I2C probe failed. Check I2C connection.");
        return ESP_FAIL;
    }

    i2c_device_config_t iis2mdc_i2c_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SENSOR_IIS2MDC_I2C_ADDRESS,
    .scl_speed_hz = 100 * 1000,
    };
    if(i2c_master_bus_add_device(press_bus_handle, &iis2mdc_i2c_cfg, &iis2mdc_i2c_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "IIS2MDC I2C add fail. Check I2C connection.");
        return ESP_FAIL;
    }

    IIS2MDC_IO_t iis2mdc_IO = {
        .Init     = (IIS2MDC_Init_Func)bsp_check_I2C_init, // Confirm I2C is initialized
        .DeInit   =  NULL, // No specific deinit function
        .BusType  =  IIS2MDC_I2C_BUS,
        .Address  =  SENSOR_IIS2MDC_I2C_ADDRESS,
        .WriteReg = (IIS2MDC_WriteReg_Func)bsp_I2C_writeWrap,
        .ReadReg  = (IIS2MDC_ReadReg_Func)bsp_I2C_readWrap,
        .GetTick  = (IIS2MDC_GetTick_Func)bsp_handle.Bsp_GetTick,
        .Delay    =  bsp_handle.Bsp_Delay
    };
    IIS2MDC_RegisterBusIO(&iis2mdc_handle, &iis2mdc_IO);

    if (IIS2MDC_Init(&iis2mdc_handle) != IIS2MDC_OK)
    {
        ESP_LOGE(TAG, "IIS2MDC init failed.");
        return ESP_FAIL;
    }

    int32_t ret = IIS2MDC_OK;
    ret |= IIS2MDC_MAG_Enable(&iis2mdc_handle);
    ret |= IIS2MDC_MAG_SetOutputDataRate(&iis2mdc_handle, 100.0f);

    if (ret != IIS2MDC_OK)
    {
        ESP_LOGE(TAG, "IIS2MDC config failed with errno: %d", (int)ret);
        return ESP_FAIL;
    }

    uint8_t hwID;
    ret |= IIS2MDC_ReadID(&iis2mdc_handle, &hwID);
    if (hwID != IIS2MDC_ID)
    {
        ESP_LOGE(TAG, "IIS2MDC ID mismatch: expected 0x%02X, got 0x%02X", IIS2MDC_ID, hwID);
        return ESP_FAIL;
    }
    if (ret != IIS2MDC_OK)
    {
        ESP_LOGE(TAG, "IIS2MDC config failed with errno: %d", (int)ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "IIS2MDC initialized successfully.");
    return ESP_OK;
}


/*---------------------------------------------------------------------
 * Function: bsp_lps22df_init
 * 
 * Description: Configures LPS22DF barometric sensor through the I2C bus.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: must be called after the I2C port is initialized.
 *---------------------------------------------------------------------*/
static esp_err_t bsp_lps22df_init(void)
{
    if(i2c_master_probe(press_bus_handle, SENSOR_LPS22DF_I2C_ADDRESS, 50) != ESP_OK)
    {
        ESP_LOGE(TAG, "LPS22DF I2C probe failed. Check I2C connection.");
        return ESP_FAIL;
    }

    i2c_device_config_t lps22df_i2c_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SENSOR_LPS22DF_I2C_ADDRESS,
    .scl_speed_hz = 100 * 1000,
    };
    if(i2c_master_bus_add_device(press_bus_handle, &lps22df_i2c_cfg, &lps22df_i2c_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "LPS22DF I2C add fail. Check I2C connection.");
        return ESP_FAIL;
    }

    LPS22DF_IO_t lps22df_IO = {
        .Init     = (LPS22DF_Init_Func)bsp_check_I2C_init, // Confirm I2C is initialized
        .DeInit   =  NULL, // No specific deinit function
        .BusType  =  LPS22DF_I2C_BUS,
        .Address  =  SENSOR_LPS22DF_I2C_ADDRESS,
        .WriteReg = (LPS22DF_WriteReg_Func)bsp_I2C_writeWrap,
        .ReadReg  = (LPS22DF_ReadReg_Func)bsp_I2C_readWrap,
        .GetTick  = (LPS22DF_GetTick_Func)bsp_handle.Bsp_GetTick,
        .Delay    =  bsp_handle.Bsp_Delay
    };
    LPS22DF_RegisterBusIO(&lps22df_handle, &lps22df_IO);

    if (LPS22DF_Init(&lps22df_handle) != LPS22DF_OK)
    {
        ESP_LOGE(TAG, "LPS22DF init failed.");
        return ESP_FAIL;
    }

    int32_t ret = LPS22DF_OK;
    ret |= LPS22DF_PRESS_Enable(&lps22df_handle);
    ret |= LPS22DF_TEMP_Enable(&lps22df_handle);
    ret |= LPS22DF_PRESS_SetOutputDataRate(&lps22df_handle, 100.0f);
    ret |= LPS22DF_TEMP_SetOutputDataRate(&lps22df_handle, 100.0f);

    if (ret != LPS22DF_OK)
    {
        ESP_LOGE(TAG, "LPS22DF config failed with errno: %d", (int)ret);
        return ESP_FAIL;
    }

    uint8_t hwID;
    ret |= LPS22DF_ReadID(&lps22df_handle, &hwID);
    if (hwID != LPS22DF_ID)
    {
        ESP_LOGE(TAG, "LPS22DF ID mismatch: expected 0x%02X, got 0x%02X", LPS22DF_ID, hwID);
        return ESP_FAIL;
    }
    if (ret != LPS22DF_OK)
    {
        ESP_LOGE(TAG, "LPS22DF config failed with errno: %d", (int)ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LPS22DF initialized successfully.");
    return ESP_OK;
}


/*---------------------------------------------------------------------
 * Function: bsp_lsm6dsv80x_init
 * 
 * Description: Configures LSM6DSV80X barometric sensor through the I2C bus.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: must be called after the I2C port is initialized.
 *---------------------------------------------------------------------*/
static esp_err_t bsp_lsm6dsv80x_init(void)
{
    if(i2c_master_probe(imu_bus_handle, SENSOR_LSM6DSV80X_I2C_ADDRESS, 50) != ESP_OK)
    {
        ESP_LOGE(TAG, "LSM6DSV80X I2C probe failed. Check I2C connection.");
        return ESP_FAIL;
    }

    i2c_device_config_t lsm6dsv80x_i2c_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = SENSOR_LSM6DSV80X_I2C_ADDRESS,
    .scl_speed_hz = 100 * 1000,
    };
    if(i2c_master_bus_add_device(imu_bus_handle, &lsm6dsv80x_i2c_cfg, &lsm6dsv80x_i2c_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "LSM6DSV80X I2C add fail. Check I2C connection.");
        return ESP_FAIL;
    }

    LSM6DSV80X_IO_t lsm6dsv80x_IO = {
        .Init     = (LSM6DSV80X_Init_Func)bsp_check_I2C_init, // Confirm I2C is initialized
        .DeInit   =  NULL, // No specific deinit function
        .BusType  =  LSM6DSV80X_I2C_BUS,
        .Address  =  SENSOR_LSM6DSV80X_I2C_ADDRESS,
        .WriteReg = (LSM6DSV80X_WriteReg_Func)bsp_I2C_writeWrap,
        .ReadReg  = (LSM6DSV80X_ReadReg_Func)bsp_I2C_readWrap,
        .GetTick  = (LSM6DSV80X_GetTick_Func)bsp_handle.Bsp_GetTick,
        .Delay    =  bsp_handle.Bsp_Delay
    };
    LSM6DSV80X_RegisterBusIO(&lsm6dsv80x_handle, &lsm6dsv80x_IO);

    if (LSM6DSV80X_Init(&lsm6dsv80x_handle) != LSM6DSV80X_OK)
    {
        ESP_LOGE(TAG, "LSM6DSV80X init failed.");
        return ESP_FAIL;
    }

    int32_t ret = LSM6DSV80X_OK;
    ret |= LSM6DSV80X_ACC_Enable(&lsm6dsv80x_handle);
    ret |= LSM6DSV80X_ACC_SetOutputDataRate(&lsm6dsv80x_handle, 120.0f); // 120Hz
    ret |= LSM6DSV80X_ACC_SetFullScale(&lsm6dsv80x_handle, 16); // 16g full scale

    ret |= LSM6DSV80X_ACC_HG_Enable(&lsm6dsv80x_handle);
    ret |= LSM6DSV80X_ACC_HG_SetOutputDataRate(&lsm6dsv80x_handle, 480.0f); // 480Hz (slowest option)
    ret |= LSM6DSV80X_ACC_HG_SetFullScale(&lsm6dsv80x_handle, 80); // 80g full scale

    ret |= LSM6DSV80X_GYRO_Enable(&lsm6dsv80x_handle);
    ret |= LSM6DSV80X_GYRO_SetOutputDataRate(&lsm6dsv80x_handle, 120.0f); // 120Hz
    ret |= LSM6DSV80X_GYRO_SetFullScale(&lsm6dsv80x_handle, 250); // 250dps full scale

    if (ret != LSM6DSV80X_OK)
    {
        ESP_LOGE(TAG, "LSM6DSV80X config failed with errno: %d", (int)ret);
        return ESP_FAIL;
    }

    uint8_t hwID;
    ret |= LSM6DSV80X_ReadID(&lsm6dsv80x_handle, &hwID);
    if (hwID != SENSOR_LSM6DSV80X_CHIP_ID)
    {
        ESP_LOGE(TAG, "LSM6DSV80X ID mismatch: expected 0x%02X, got 0x%02X", SENSOR_LSM6DSV80X_CHIP_ID, hwID);
        return ESP_FAIL;
    }
    if (ret != LSM6DSV80X_OK)
    {
        ESP_LOGE(TAG, "LSM6DSV80X config failed with errno: %d", (int)ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LSM6DSV80X initialized successfully.");
    return ESP_OK;
}


/*---------------------------------------------------------------------
 * Function: bsp_TWAI_init
 * 
 * Description: Start the TWAI (Two wire automotive interface) with pin
 *              mappings attached to the board's CAN transciever.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 *---------------------------------------------------------------------*/
static esp_err_t bsp_TWAI_init(void)
{
    esp_err_t ret = ESP_OK;
    twai_onchip_node_config_t can_node_config = {
        .io_cfg.tx = CAN_TX_PIN, // TWAI TX GPIO pin
        .io_cfg.rx = CAN_RX_PIN, // TWAI RX GPIO pin
        .io_cfg.bus_off_indicator = -1,
        .io_cfg.quanta_clk_out = -1,
        .bit_timing.bitrate = 200000,  // 200 kbps bitrate
        .tx_queue_depth = 5,        // Transmit queue depth set to 5
    };
    // Create a new TWAI controller driver instance
    ret = twai_new_node_onchip(&can_node_config, &can_node_hdl);
    // Start the TWAI controller
    if(ret == ESP_OK)
        ret = twai_node_enable(can_node_hdl);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "CAN init fail: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "CAN bus initialized successfully.");
    return ESP_OK;
}


/*---------------------------------------------------------------------
 * Function: bsp_init
 * 
 * Description: Initializes the embedded peripherals for the mini flight
 *              computer.
 *              This function sets up the LEDs, Pyro channels, I2C
 *              buses, and sensors.
 * 
 * Parameters: None
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: This function should be called at the start of the application
 *       to ensure all hardware specific components and interfaces are
 *       initialized before use.
 *---------------------------------------------------------------------*/
esp_err_t bsp_init(board_handle_t *handle, bsp_config_t *bsp_init_obj)
{
    ESP_LOGI(TAG, "Initializing BSP...");
    esp_err_t ret = ESP_OK;

    // Copy configuration to global variables
    if (bsp_init_obj == NULL)
        return ESP_ERR_INVALID_ARG;
    if (bsp_init_obj->Bsp_GetTick == NULL || bsp_init_obj->Bsp_Delay == NULL)
        return ESP_ERR_INVALID_ARG;
    bsp_handle.Bsp_Delay = bsp_init_obj->Bsp_Delay;
    bsp_handle.Bsp_GetTick = bsp_init_obj->Bsp_GetTick;

    ret |= bsp_led_init();  // Onboard LEDs
    ret |= bsp_pyro_init(); // Pyro channels
    ret |= bsp_I2C_init();  // I2C buses

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BSP init fail...exiting catch 1...");
        return ret; // Error code
    }

    // Initialize sensors
    ret |= bsp_iis2mdc_init();
    ret |= bsp_lps22df_init();
    //ret |= bsp_lsm6dsv80x_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BSP init fail...exiting catch 2...");
        return ret; // Error code
    }
    ESP_LOGI(TAG, "Sensors initialized successfully.");

    // Initialize CAN
    ret |= bsp_TWAI_init();

    // Additional initialization can be added here
    // ...

    bsp_handle.isInitialized = true;
    *handle = (board_handle_t)&bsp_handle;
    // Log successful initialization
    ESP_LOGI(TAG, "BSP initialized successfully.");
    return 0; // Success code
}