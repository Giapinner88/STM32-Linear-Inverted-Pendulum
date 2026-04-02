/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

/* Motor control variables */
extern TIM_HandleTypeDef htim4;  /* PWMB on PB1 - TIM4_CH2 */
extern UART_HandleTypeDef huart1;  /* For serial output */
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

/* Debug log structure */
#define MAX_LOG_ENTRIES 100
typedef struct {
  uint32_t timestamp_ms;
  int32_t encoder_value;
  int32_t stable_count;
  char phase;  /* 'L' for LEFT, 'R' for RIGHT */
} DebugLog_t;

DebugLog_t debug_log[MAX_LOG_ENTRIES];
int log_index = 0;

/* Global variables for results */
int32_t encoder_left = 0;
int32_t encoder_right = 0;
int32_t total_range = 0;

void add_log(int32_t enc, int32_t stab, char phase)
{
  if(log_index < MAX_LOG_ENTRIES)
  {
    debug_log[log_index].timestamp_ms = HAL_GetTick();
    debug_log[log_index].encoder_value = enc;
    debug_log[log_index].stable_count = stab;
    debug_log[log_index].phase = phase;
    log_index++;
  }
}

void serial_printf(const char *fmt, ...)
{
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
}

void display_log(void)
{
  /* Print to serial */
  serial_printf("\r\n\r\n========== TEST RESULTS ==========\r\n");
  serial_printf("LEFT LIMIT (encoder):  %d\r\n", encoder_left);
  serial_printf("RIGHT LIMIT (encoder): %d\r\n", encoder_right);
  serial_printf("RANGE (pulses):        %d\r\n", total_range);
  serial_printf("==================================\r\n\r\n");
  
  /* Display on OLED */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t*)"===RESULTS===");
  OLED_Refresh_Gram();
  delay_ms(1000);
  
  /* Display LEFT value */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t*)"LEFT:");
  OLED_ShowNumber(0, 2, encoder_left, 5, 16);
  OLED_ShowNumber(0, 4, encoder_left, 5, 16);  /* Display twice for visibility */
  OLED_Refresh_Gram();
  delay_ms(2000);
  
  /* Display RIGHT value */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t*)"RIGHT:");
  OLED_ShowNumber(0, 2, encoder_right, 5, 16);
  OLED_ShowNumber(0, 4, encoder_right, 5, 16);  /* Display twice for visibility */
  OLED_Refresh_Gram();
  delay_ms(2000);
  
  /* Display RANGE */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t*)"RANGE:");
  OLED_ShowNumber(0, 2, total_range, 5, 16);
  OLED_ShowNumber(0, 4, total_range, 5, 16);  /* Display twice for visibility */
  OLED_ShowString(0, 6, (uint8_t*)"pulses");
  OLED_Refresh_Gram();
  delay_ms(2000);
}

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
  
  /* Initialize OLED display */
  OLED_Init();
  delay_ms(1000);
  OLED_Clear();
  
  /* Start PWM on TIM3 Channel 4 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  
  /* Start encoder on TIM4 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    /* ========== WAIT FOR USER BUTTON PRESS ========== */
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"READY");
    OLED_ShowString(0, 2, (uint8_t*)"Press USER btn");
    OLED_Refresh_Gram();
    
    /* Wait for User button press (schematic: PA7 / User_key) */
    /* Poll button - it's active LOW due to pull-up */
    while(HAL_GPIO_ReadPin(User_key_GPIO_Port, User_key_Pin) == GPIO_PIN_SET)
    {
      delay_ms(50);  /* Debounce delay */
    }
    
    /* Button detected! Flash LED 3 times */
    for(int i = 0; i < 3; i++)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      delay_ms(200);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      delay_ms(200);
    }
    
    /* Wait a bit before starting test */
    delay_ms(1000);
    
    /* ========== MOTOR TRAVEL LIMIT TEST ========== */
    
    uint16_t motor_pwm = 3500;  /* ~50% power for steady movement */
    uint32_t move_duration = 6000;  /* 6 seconds to reach limit */
    int32_t last_enc = -999;  /* For collision detection */
    int32_t stable_count = 0;  /* For collision detection */
    
    /* ===== MOVE LEFT ===== */
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"GO LEFT");
    OLED_Refresh_Gram();
    delay_ms(500);
    
    log_index = 0;  /* Reset log */
    add_log(-999, -1, 'L');  /* Marker: start LEFT */
    
    /* Set direction LEFT - FIRST before applying power */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); /* BIN1 = 0 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   /* BIN2 = 1 */
    delay_ms(100);  /* Let motor driver settle */
    
    /* Reset encoder */
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    delay_ms(200);
    
    /* Move left with constant power */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, motor_pwm);
    uint32_t start_time = HAL_GetTick();
    last_enc = -999;
    stable_count = 0;
    
    while(1)
    {
      uint32_t now = HAL_GetTick();
      uint32_t elapsed = now - start_time;
      int32_t curr_enc = __HAL_TIM_GET_COUNTER(&htim4);
      
      /* Check if encoder is stable (not changing) */
      if(curr_enc == last_enc)
      {
        stable_count++;
      }
      else
      {
        stable_count = 0;  /* Reset if any motion detected */
        last_enc = curr_enc;
      }
      
      /* Exit if: timeout (6s) OR encoder stable for 3 reads (~900ms) */
      if(elapsed >= move_duration || stable_count >= 3)
      {
        add_log(curr_enc, stable_count, 'L');  /* Log exit condition */
        encoder_left = curr_enc;  /* Save before exiting loop */
        break;
      }
      
      add_log(curr_enc, stable_count, 'L');
      
      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t*)"LEFT");
      OLED_ShowString(0, 1, (uint8_t*)"Enc:");
      OLED_ShowNumber(35, 1, curr_enc, 5, 16);
      OLED_ShowString(0, 2, (uint8_t*)"Stb:");
      OLED_ShowNumber(35, 2, stable_count, 1, 16);
      OLED_ShowString(0, 3, (uint8_t*)"Tm:");
      uint32_t secs = elapsed / 1000;
      OLED_ShowNumber(35, 3, secs, 1, 16);
      OLED_ShowString(50, 3, (uint8_t*)"s");
      OLED_Refresh_Gram();
      delay_ms(300);
    }
    
    /* STOP motor completely */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    /* Clear direction pins too */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    delay_ms(1000);  /* Wait for motor to stop */
    
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"LEFT done");
    OLED_ShowString(0, 1, (uint8_t*)"Enc:");
    OLED_ShowNumber(35, 1, encoder_left, 5, 16);
    OLED_ShowString(0, 3, (uint8_t*)"Wait...");
    OLED_Refresh_Gram();
    delay_ms(2000);
    
    /* ===== MOVE RIGHT ===== */
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"GO RIGHT");
    OLED_Refresh_Gram();
    delay_ms(500);
    
    add_log(-999, -2, 'R');  /* Marker: start RIGHT */
    
    /* Set direction RIGHT - FIRST before applying power */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   /* BIN1 = 1 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); /* BIN2 = 0 */
    delay_ms(100);  /* Let motor driver settle */
    
    /* DON'T reset encoder - keep measuring from origin */
    
    /* Move right with constant power */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, motor_pwm);
    start_time = HAL_GetTick();
    last_enc = __HAL_TIM_GET_COUNTER(&htim4);  /* Start from current position */
    stable_count = 0;
    
    while(1)
    {
      uint32_t now = HAL_GetTick();
      uint32_t elapsed = now - start_time;
      int32_t curr_enc = __HAL_TIM_GET_COUNTER(&htim4);
      
      /* Check if encoder is stable (not changing) */
      if(curr_enc == last_enc)
      {
        stable_count++;
      }
      else
      {
        stable_count = 0;  /* Reset if any motion detected */
        last_enc = curr_enc;
      }
      
      /* Exit if: timeout (6s) OR encoder stable for 3 reads (~900ms) */
      if(elapsed >= move_duration || stable_count >= 3)
      {
        encoder_right = curr_enc;  /* Save before exiting loop */
        add_log(curr_enc, stable_count, 'R');  /* Log exit condition */
        break;
      }
      
      add_log(curr_enc, stable_count, 'R');
      
      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t*)"RIGHT");
      OLED_ShowString(0, 1, (uint8_t*)"Enc:");
      OLED_ShowNumber(35, 1, curr_enc, 5, 16);
      OLED_ShowString(0, 2, (uint8_t*)"Stb:");
      OLED_ShowNumber(35, 2, stable_count, 1, 16);
      OLED_ShowString(0, 3, (uint8_t*)"Tm:");
      uint32_t secs = elapsed / 1000;
      OLED_ShowNumber(35, 3, secs, 1, 16);
      OLED_ShowString(50, 3, (uint8_t*)"s");
      OLED_Refresh_Gram();
      delay_ms(300);
    }
    
    /* STOP motor completely */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    /* Clear direction pins too */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    delay_ms(1000);  /* Wait for motor to stop */
    
    /* Calculate range */
    total_range = encoder_right - encoder_left;  /* Use global, not local */
    if(total_range < 0) total_range = -total_range;
    
    /* Display results */
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"RESULTS");
    OLED_Refresh_Gram();
    delay_ms(1000);
    
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"L:");
    OLED_ShowNumber(20, 0, encoder_left, 5, 16);
    OLED_ShowString(0, 2, (uint8_t*)"R:");
    OLED_ShowNumber(20, 2, encoder_right, 5, 16);
    OLED_ShowString(0, 4, (uint8_t*)"RNG:");
    OLED_ShowNumber(35, 4, total_range, 5, 16);
    OLED_ShowString(0, 6, (uint8_t*)"Press btn");
    OLED_Refresh_Gram();
    
    /* Wait for button press */
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET)
    {
      delay_ms(50);
    }
    
    /* Debounce */
    delay_ms(500);
    
    /* Show log after test */
    display_log();
    
    /* Wait for button again */
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET)
    {
      delay_ms(50);
    }
    delay_ms(500);;
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

