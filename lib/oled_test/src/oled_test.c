/**
 * @file oled_test.c
 * @brief Simple OLED test program
 * 
 * This program tests only OLED display functionality
 * LED and Motor are disabled for debugging purposes
 */

#include "main.h"
#include "delay.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "adc.h"
#include "i2c.h"
#include "oled.h"

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void);

/**
 * @brief Main test entry point
 */
int main(void)
{
  /* MCU Configuration */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Initialize delay function (SYSCLK = 72MHz) */
  delay_init(72);
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();

  /* ========== OLED TEST SECTION ========== */
  
  /* Long delay after GPIO init */
  delay_ms(200);
  
  /* Initialize OLED */
  OLED_Init();
  
  /* Wait for OLED reset and stabilization */
  delay_ms(1000);
  
  /* Clear OLED display */
  OLED_Clear();
  
  /* Wait for clear operation to complete */
  delay_ms(200);

  /* Main loop - Simple OLED test patterns */
  while (1)
  {
    /* ===== Pattern 1: Show text ===== */
    OLED_Clear();
    delay_ms(100);
    
    OLED_ShowString(0, 0, (uint8_t*)"OLED TEST");
    OLED_ShowString(0, 2, (uint8_t*)"Display ON");
    OLED_ShowString(0, 4, (uint8_t*)"Pattern 1");
    OLED_ShowString(0, 6, (uint8_t*)"Working!");
    
    delay_ms(2000);  /* Display for 2 seconds */

    /* ===== Pattern 2: Different text ===== */
    OLED_Clear();
    delay_ms(100);
    
    OLED_ShowString(0, 1, (uint8_t*)"STM32 Test");
    OLED_ShowString(0, 3, (uint8_t*)"I2C - 0x78");
    OLED_ShowString(0, 5, (uint8_t*)"SSD1306");
    
    delay_ms(2000);  /* Display for 2 seconds */

    /* ===== Pattern 3: Simple counter ===== */
    OLED_Clear();
    delay_ms(100);
    
    OLED_ShowString(0, 0, (uint8_t*)"Counter:");
    
    for(int i = 0; i < 10; i++)
    {
      OLED_ShowNumber(80, 0, i, 1, 16);
      delay_ms(500);
    }

    /* ===== Pattern 4: All clear (test if display works) ===== */
    OLED_Clear();
    delay_ms(2000);  /* Dark screen for 2 seconds */

    /* ===== Pattern 5: Fill all (all pixels on) ===== */
    for(int x = 0; x < 128; x++)
    {
      for(int y = 0; y < 64; y++)
      {
        OLED_DrawPoint(x, y, 1);
      }
    }
    OLED_Refresh_Gram();
    delay_ms(2000);

    /* ===== Pattern 6: Checkerboard pattern ===== */
    OLED_Clear();
    for(int x = 0; x < 128; x += 2)
    {
      for(int y = 0; y < 64; y += 2)
      {
        OLED_DrawPoint(x, y, 1);
      }
    }
    OLED_Refresh_Gram();
    delay_ms(2000);
  }

  return 0;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
