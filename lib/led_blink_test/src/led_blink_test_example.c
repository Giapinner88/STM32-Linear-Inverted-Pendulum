/**
 * @file led_blink_test_example.c
 * @brief Example of how to use LED blink test functions
 * 
 * To use this test in your main.c:
 * 1. Include the header:
 *    #include "led_blink_test.h"
 * 
 * 2. Add the library to platformio.ini:
 *    lib_extra_dirs = lib/led_blink_test
 * 
 * 3. In main(), after MX_GPIO_Init():
 *    LED_Blink_Init();
 * 
 * 4. Choose one of the following in the main loop or a test section:
 */

#include "main.h"
#include "delay.h"
#include "led_blink_test.h"

/* Example 1: Simple continuous blinking (500ms interval) */
void test_continuous_blink(void)
{
    LED_Blink_Init();
    
    while(1)
    {
        LED_Blink(500);  /* Toggle LED every 500ms (1Hz frequency) */
    }
}

/* Example 2: Blink LED 10 times with 300ms interval */
void test_blink_n_times(void)
{
    LED_Blink_Init();
    
    LED_Blink_N_Times(10, 300);  /* 10 blinks with 300ms intervals */
}

/* Example 3: Custom blink pattern in main loop */
void test_custom_pattern(void)
{
    LED_Blink_Init();
    
    while(1)
    {
        /* Fast blink - 5 times */
        LED_Blink_N_Times(5, 100);
        delay_ms(500);
        
        /* Slow blink - 3 times */
        LED_Blink_N_Times(3, 500);
        delay_ms(1000);
    }
}

/**
 * QUICK START:
 * 
 * In your src/main.c, modify the main() function:
 * 
 * int main(void)
 * {
 *   HAL_Init();
 *   SystemClock_Config();
 *   MX_GPIO_Init();
 *   
 *   // Add other initializations
 *   MX_TIM3_Init();
 *   MX_TIM4_Init();
 *   MX_USART1_UART_Init();
 *   MX_ADC1_Init();
 *   MX_TIM1_Init();
 *   MX_I2C1_Init();
 *   OLED_Init();
 *   
 *   // Test LED blinking on PA4
 *   LED_Blink_Init();
 *   
 *   while(1)
 *   {
 *       LED_Blink(500);  // Blink with 500ms interval
 *   }
 *   
 *   return 0;
 * }
 */
