#include "led_blink_test.h"
#include "delay.h"

/**
 * @brief Initialize LED pin (already configured in MX_GPIO_Init)
 */
void LED_Blink_Init(void)
{
    /* PA4 is already configured as output in MX_GPIO_Init */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

/**
 * @brief Blink LED continuously with specified interval
 * @param interval_ms: Interval in milliseconds (total period = 2 * interval_ms)
 */
void LED_Blink(uint16_t interval_ms)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    delay_ms(interval_ms);
}

/**
 * @brief Blink LED N times
 * @param times: Number of times to blink
 * @param interval_ms: Interval in milliseconds for each on/off period
 */
void LED_Blink_N_Times(uint16_t times, uint16_t interval_ms)
{
    for(uint16_t i = 0; i < times * 2; i++)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        delay_ms(interval_ms);
    }
    /* Ensure LED is OFF after blinking */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
