/**
 * @file test_main.c
 * @brief Simple test file to demonstrate PA4 LED blinking
 * 
 * This is a minimal example that can be used to test LED blinking on PA4.
 * To use this test:
 * 
 * 1. Compile with the project
 * 2. Flash to STM32F103C8TX
 * 3. The LED on PA4 should blink at 1Hz (500ms interval)
 * 
 * Expected behavior:
 * - LED turns ON for 500ms
 * - LED turns OFF for 500ms
 * - Repeats continuously
 */

#include "main.h"
#include "delay.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "adc.h"
#include "i2c.h"
#include "led_blink_test.h"

/* ============================================================================
   TEST 1: Simple continuous blinking at 1Hz
   ============================================================================ */
void test_led_blink_1hz(void)
{
    LED_Blink_Init();
    
    while(1)
    {
        LED_Blink(500);  /* 500ms ON, 500ms OFF = 1Hz frequency */
    }
}

/* ============================================================================
   TEST 2: Blink LED 10 times with 300ms interval
   ============================================================================ */
void test_led_blink_n_times(void)
{
    LED_Blink_Init();
    
    LED_Blink_N_Times(10, 300);  /* Blink 10 times, 300ms interval */
    
    /* After blinking, keep LED OFF */
    while(1)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        delay_ms(1000);
    }
}

/* ============================================================================
   TEST 3: Various blinking patterns
   ============================================================================ */
void test_led_blink_patterns(void)
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

/* ============================================================================
   TEST 4: Real-time blinking in main loop with other operations
   ============================================================================ */
void test_led_blink_with_operations(void)
{
    LED_Blink_Init();
    
    uint32_t counter = 0;
    
    while(1)
    {
        /* Toggle LED every 500ms */
        if(counter % 500 == 0)
        {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }
        
        counter++;
        delay_ms(1);
    }
}

/* ============================================================================
   MAIN - Choose which test to run
   ============================================================================ */
int main(void)
{
    /* Initialize MCU */
    HAL_Init();
    SystemClock_Config();
    
    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_I2C1_Init();
    
    /* ====================================================================
       Select which test to run by uncommenting one of these:
       ==================================================================== */
    
    test_led_blink_1hz();              /* 1Hz blinking */
    // test_led_blink_n_times();          /* Blink N times */
    // test_led_blink_patterns();         /* Multiple patterns */
    // test_led_blink_with_operations();  /* Blinking in main loop */
    
    return 0;
}

/* ============================================================================
   This example assumes:
   - PA4 has an LED connected (LED anode to PA4, cathode to GND via resistor)
   - delay_ms() is available from the delay library
   - All HAL peripheral initializations are working
   ============================================================================ */
