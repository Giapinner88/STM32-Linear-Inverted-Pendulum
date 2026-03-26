/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "oled.h"
#include "show.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"

/* Motor control variables */
extern TIM_HandleTypeDef htim4;  /* PWMB on PB1 - TIM4_CH2 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Initialize delay function (SYSCLK = 72MHz) */
  delay_init(72);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  
  /* Wait for GPIO to stabilize */
  delay_ms(200);
  
  /* Initialize OLED display - with extra debug delays */
  OLED_Init();
  
  /* Wait for OLED to fully initialize and reset */
  delay_ms(1000);
  
  /* Clear display */
  OLED_Clear();
  
  /* Wait for clear to complete */
  delay_ms(200);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    /* ===== OLED Test Pattern 1: Text Display ===== */
    OLED_Clear();
    delay_ms(100);
    OLED_ShowString(0, 0, (uint8_t*)"OLED TEST");
    OLED_ShowString(0, 2, (uint8_t*)"Display: ON");
    OLED_ShowString(0, 4, (uint8_t*)"I2C: 0x78");
    OLED_ShowString(0, 6, (uint8_t*)"SSD1306");
    delay_ms(2000);

    /* ===== OLED Test Pattern 2: Different Text ===== */
    OLED_Clear();
    delay_ms(100);
    OLED_ShowString(0, 1, (uint8_t*)"STM32F103C8");
    OLED_ShowString(0, 3, (uint8_t*)"Testing...");
    OLED_ShowString(0, 5, (uint8_t*)"Pattern 2");
    delay_ms(2000);

    /* ===== OLED Test Pattern 3: Number Display ===== */
    OLED_Clear();
    delay_ms(100);
    OLED_ShowString(0, 0, (uint8_t*)"Counter:");
    for(uint8_t i = 0; i < 16; i++)
    {
      OLED_ShowNumber(80, 0, i, 2, 16);
      delay_ms(300);
    }

    /* ===== OLED Test Pattern 4: Clear screen ===== */
    OLED_Clear();
    delay_ms(2000);  /* Show black screen for 2 seconds */

    /* ===== OLED Test Pattern 5: All pixels ON ===== */
    for(uint16_t x = 0; x < 128; x++)
    {
      for(uint8_t y = 0; y < 64; y++)
      {
        OLED_DrawPoint(x, y, 1);
      }
    }
    OLED_Refresh_Gram();
    delay_ms(2000);  /* Show filled screen for 2 seconds */

    /* ===== OLED Test Pattern 6: Checkerboard ===== */
    OLED_Clear();
    for(uint16_t x = 0; x < 128; x += 2)
    {
      for(uint8_t y = 0; y < 64; y += 2)
      {
        OLED_DrawPoint(x, y, 1);
      }
    }
    OLED_Refresh_Gram();
    delay_ms(2000);

    /* ===== OLED Test Pattern 7: Back to text ===== */
    OLED_Clear();
    delay_ms(100);
    OLED_ShowString(0, 2, (uint8_t*)"All Tests Done!");
    OLED_ShowString(0, 4, (uint8_t*)"Looping...");
    delay_ms(1000);
  }
  /* USER CODE END 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

