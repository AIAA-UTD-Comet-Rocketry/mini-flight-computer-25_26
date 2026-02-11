/*
 * File: gpio_mgr.h
 * 
 * Manages the low speed logic needed to drive the LED lights and pyro events.
 * The logic is wrapped into two RTOS tasks on the secondary core.
 * 
 * Created: 2025
 */

#ifndef _GPIO_MGR_H_
#define _GPIO_MGR_H_

#include "BSP.h"
#include "freertos/FreeRTOS.h"

typedef enum {
    pattern_alt_func,
    pattern_off,
    pattern_on,
    pattern_slow_blink,
    pattern_1sec_blink,
    pattern_fast_blink,
    pattern_very_fast_blink,
    pattern_1sec_flash,
    pattern_fast_flash,
    pattern_burst,
    pattern_twoblink,
    pattern_bahdum,
    pattern_max
} pattern_index_t;

void      LED_Task( board_handle_t *handle );
esp_err_t LED_setPattern( led_index_t led, pattern_index_t pattern );
esp_err_t LED_on( led_index_t led );
esp_err_t LED_off( led_index_t led );
esp_err_t LED_toggle( led_index_t led );

#endif // _GPIO_MGR_H_