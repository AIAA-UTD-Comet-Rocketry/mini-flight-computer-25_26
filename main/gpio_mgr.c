/*
 * File: gpio_mgr.h
 * 
 * Manages the low speed logic needed to drive the LED lights and pyro events.
 * The logic is wrapped into two RTOS tasks on the secondary core.
 * 
 * Created: 2025
 */

#include "gpio_mgr.h"

#define PYRO_MAX_ATTEMPTS 2 // total allowed pyro drives
#define PYRO_DRIVE_TIME  50 // ms to drive pyro channel for
#define PYRO_COOL_TIME   50 // ms to wait between attempted drives

#define patternMax     160 // 125ms per sample so 20 seconds of to cycle through
#define patternBytes    20 // Each bit is a time sample

const uint8_t PATTERN_ALT_FUNC[patternBytes] = {
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
}; // don't drive led from mgr task, 20 bytes allocated for memory safety

const uint8_t PATTERN_OFF[patternBytes] = {
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
}; // off

const uint8_t PATTERN_ON[patternBytes] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF
}; // on

const uint8_t PATTERN_SLOW_BLINK[patternBytes] = {
    0xFF, 0xFF, 0xF0, 0x00, 0x00,
    0xFF, 0xFF, 0xF0, 0x00, 0x00,
    0xFF, 0xFF, 0xF0, 0x00, 0x00,
    0xFF, 0xFF, 0xF0, 0x00, 0x00
}; // 2.5 sec on - 2.5 sec off

const uint8_t PATTERN_1SEC_BLINK[patternBytes] = {
    0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00
}; // 1 sec on - 1 sec off

const uint8_t PATTERN_FAST_BLINK[patternBytes] = {
    0x33, 0x33, 0x33, 0x33, 0x33,
    0x33, 0x33, 0x33, 0x33, 0x33,
    0x33, 0x33, 0x33, 0x33, 0x33,
    0x33, 0x33, 0x33, 0x33, 0x33
}; // 0.25 sec on - 0.25 sec off

const uint8_t PATTERN_VERY_FAST_BLINK[patternBytes] = {
    0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55
}; // 0.25 sec on - 0.25 sec off

const uint8_t PATTERN_1SEC_FLASH[patternBytes] = {
    0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01
}; // 0.125 sec on - 0.875 sec off

const uint8_t PATTERN_FAST_FLASH[patternBytes] = {
    0x11, 0x11, 0x11, 0x11, 0x11,
    0x11, 0x11, 0x11, 0x11, 0x11,
    0x11, 0x11, 0x11, 0x11, 0x11,
    0x11, 0x11, 0x11, 0x11, 0x11
}; // 0.125 sec on - 0.375 sec off

const uint8_t PATTERN_BURST[patternBytes] = {
    0x55, 0x00, 0x55, 0x00, 0x55,
    0x00, 0x55, 0x00, 0x55, 0x00,
    0x55, 0x00, 0x55, 0x00, 0x55,
    0x00, 0x55, 0x00, 0x55, 0x00
}; // 4 very-fast blinks - 1 sec off

const uint8_t PATTERN_TWOBLINK[patternBytes] = {
    0x55, 0x00, 0x55, 0x00, 0x55,
    0x00, 0x55, 0x00, 0x55, 0x00,
    0x55, 0x00, 0x55, 0x00, 0x55,
    0x00, 0x55, 0x00, 0x55, 0x00
}; // 2 fast blinks - 4 very-fast blinks

const uint8_t PATTERN_BAHDUM[patternBytes] = {
    0x0C, 0xFF, 0x0C, 0xFF, 0x0C,
    0xFF, 0x0C, 0xFF, 0x0C, 0xFF,
    0x0C, 0xFF, 0x0C, 0xFF, 0x0C,
    0xFF, 0x0C, 0xFF, 0x0C, 0xFF
}; // 0.5 sec off - 0.25 sec on - 0.25 sec off - 1 sec on

const uint8_t* g_patterns[pattern_max] = {
    PATTERN_ALT_FUNC,
    PATTERN_OFF,
    PATTERN_ON,
    PATTERN_SLOW_BLINK,
    PATTERN_1SEC_BLINK,
    PATTERN_FAST_BLINK,
    PATTERN_VERY_FAST_BLINK,
    PATTERN_1SEC_FLASH,
    PATTERN_FAST_FLASH,
    PATTERN_BURST,
    PATTERN_TWOBLINK,
    PATTERN_BAHDUM
};

uint8_t* led_set_patterns[led_max];


/*---------------------------------------------------------------------
 * Function: LED_Task
 * 
 * Description: Drives attached LED patterns by reading in compiled
 *              file. Loops each pattern after 20 seconds. Each LED
 *              uses a pointer to the specific pattern file used.
 * 
 * Parameters: handle - board specific configurations out of BSP
 * Returns: None
 * 
 * Note: Task creation relies on board hardware being initialized first
 *---------------------------------------------------------------------*/
void LED_Task( board_handle_t *handle )
{
    // Set up varaibles controlling light patterns
    uint8_t pattern_subindex = 0;

    for(uint8_t led = 0; led < led_max; led++)
    {
        led_set_patterns[led] = (uint8_t *)g_patterns[pattern_alt_func]; // Default to not driving any LED
    }

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(125)); // TODO: Use vTaskDelayUntil to remove frequency jitter
        if(++pattern_subindex >= patternMax)
        {
            pattern_subindex = 0;
        }

        for(uint8_t led = 0; led < led_max; led++)
        {
            if((led_set_patterns[led] == PATTERN_ALT_FUNC) || (led_set_patterns[led] == NULL))
                continue; // Don't drive alt function leds

            // Get bit from the pattern, div 8 equals the byte in the pattern, mod 8 equals the bit.
            // Within a byte, the MSB is zero for the purposes of the bit stream.
            uint8_t next_pattern_byte = led_set_patterns[led][pattern_subindex >> 3];
            uint32_t next_level = ((next_pattern_byte << (pattern_subindex & 0x7)) & 0x80) ? 1 : 0;
            gpio_set_level((*handle)->led_gpio_nums[led], next_level);
        }
    }
}


/*---------------------------------------------------------------------
 * Function: LED_setPattern
 * 
 * Description: Link pattern to a specific LED
 * 
 * Parameters: led     - specific led to attach the pattern to.
 *             pattern - specific pattern to attach.
 * Returns: ESP_OK on success, or an error code on failure.
 *---------------------------------------------------------------------*/
esp_err_t LED_setPattern(led_index_t led, pattern_index_t pattern)
{
    if((led >= led_max) || (pattern >= pattern_max))
    {
        return ESP_ERR_INVALID_ARG;
    }

    led_set_patterns[led] = (uint8_t *)g_patterns[pattern];
    return ESP_OK;
}


/*---------------------------------------------------------------------
 * Function: LED_on
 * 
 * Description: Simplified interface for LED level sets
 * 
 * Parameters: led     - specific led to attach the pattern to.
 *             pattern - specific pattern to attach.
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: Calling this function on an LED set to PATTERN_ALT_FUNC will
 *       return ESP_ERR_INVALID_STATE. LED_setPattern() must be called
 *       to force ALT_FUNC override
 *---------------------------------------------------------------------*/
esp_err_t LED_on(led_index_t led)
{
    if(led_set_patterns[led] == PATTERN_ALT_FUNC)
        return ESP_ERR_INVALID_STATE;
    return LED_setPattern(led, pattern_on);
}


/*---------------------------------------------------------------------
 * Function: LED_off
 * 
 * Description: Simplified interface for LED level sets
 * 
 * Parameters: led     - specific led to attach the pattern to.
 *             pattern - specific pattern to attach.
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: Calling this function on an LED set to PATTERN_ALT_FUNC will
 *       return ESP_ERR_INVALID_STATE. LED_setPattern() must be called
 *       to force ALT_FUNC override
 *---------------------------------------------------------------------*/
esp_err_t LED_off(led_index_t led)
{
    if(led_set_patterns[led] == PATTERN_ALT_FUNC)
        return ESP_ERR_INVALID_STATE;
    return LED_setPattern(led, pattern_off);
}


/*---------------------------------------------------------------------
 * Function: LED_toggle
 * 
 * Description: Simplified interface for LED toggles
 * 
 * Parameters: led     - specific led to attach the pattern to.
 *             pattern - specific pattern to attach.
 * Returns: ESP_OK on success, or an error code on failure.
 * 
 * Note: Calling this function on an LED set to PATTERN_ALT_FUNC will
 *       return ESP_ERR_INVALID_STATE. LED_setPattern() must be called
 *       to force ALT_FUNC override
 *---------------------------------------------------------------------*/
esp_err_t LED_toggle(led_index_t led)
{
    if(led_set_patterns[led] == PATTERN_ALT_FUNC)
        return ESP_ERR_INVALID_STATE;
    else if(led_set_patterns[led] == PATTERN_ON)
        return LED_setPattern(led, pattern_off);
    else
        return LED_setPattern(led, pattern_on);
}


/*---------------------------------------------------------------------
 * Function: Pyro_Task
 * 
 * Description: Manages pyro channels to perform requested event
 *              sequences and trigger callbacks.
 * 
 * Parameters: handle - board specific configurations out of BSP
 * Returns: None
 * 
 * Note: Task creation relies on board hardware being initialized first
 *---------------------------------------------------------------------*/
void Pyro_Task( board_handle_t *handle )
{
    // Set up varaibles controlling pyro channels
    uint32_t wakeNotification = 0;
    uint8_t requestedChannel;

    for(;;)
    {
        // Wait for event request
        xTaskNotifyWait(0x00, ULONG_MAX, &wakeNotification, portMAX_DELAY);

        // Get GPIO_NUM from handle and requested flag
        requestedChannel = pyro_channel_max;
        for(uint8_t chan = 0; chan < pyro_channel_max; chan++)
        {
            if((wakeNotification >> chan) & 0x0001)
            {
                requestedChannel = chan;
            }
        }

        if(requestedChannel < pyro_channel_max)
        {
            // loop for number of retries configured
            for(uint8_t i = 0; i < PYRO_MAX_ATTEMPTS; i++)
            {
                // Turn on channel for burst, then turn off
                gpio_set_level((*handle)->pyro_gpio_nums[requestedChannel], 1); // on
                vTaskDelay(pdMS_TO_TICKS(PYRO_DRIVE_TIME));
                gpio_set_level((*handle)->pyro_gpio_nums[requestedChannel], 0); // off

                // If success break loop, no more retries - TODO: not implemented)
                if(true)
                    break;
                
                // Cool off between firing to protect hardware for ematch short
                vTaskDelay(pdMS_TO_TICKS(PYRO_COOL_TIME));
            }
        }
    }
}