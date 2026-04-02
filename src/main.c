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
#include "motor.h"
#include "comms.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include <math.h>

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
#define CONTROL_PERIOD_MS 5U
#define OLED_PERIOD_MS 80U
#define COMMAND_TIMEOUT_MS 250U
#define UPRIGHT_ANGLE_RAD 3.1415926f

#define UPRIGHT_ENTER_TH_RAD 0.30f
#define UPRIGHT_EXIT_TH_RAD 0.45f
#define UPRIGHT_ENTER_DTH_RAD_S 4.5f
#define UPRIGHT_EXIT_DTH_RAD_S 7.0f
#define UPRIGHT_ENTER_TICKS 5U
#define UPRIGHT_EXIT_TICKS 1U

#define ANGLE_ZERO_DEFAULT_ADC 1024.0f
#define ADC_TO_RAD (2.0f * 3.1415926f / 4096.0f)

/* If enabled, MENU accepts zero calibration only when ADC is in [1010, 1030]. */
#define POT_ZERO_STRICT_WINDOW 1U
#define POT_ZERO_MIN_ADC 1010U
#define POT_ZERO_MAX_ADC 1030U

#define ENC_SOFT_LIMIT_POS 18000
#define ENC_SOFT_LIMIT_NEG -18000
#define JOG_U_CMD 0.18f

#define GRAVITY_MPS2 9.81f
#define PENDULUM_LENGTH_M 0.200f
#define CART_MASS_KG 0.100f

#define SWINGUP_U_SIGN 1.0f
#define SWINGUP_MIN_PUMP_U 0.16f
#define SWINGUP_ENERGY_TO_U 0.14f
#define SWINGUP_KICK_TH_RAD 0.35f
#define SWINGUP_KICK_DTH_RAD_S 1.0f
#define SWINGUP_KICK_U 0.26f
#define SWINGUP_MAX_U 0.72f
#define SWINGUP_WINDOW_HALF_TICKS 3200
#define SWINGUP_RETURN_U_MAX 0.62f
#define SWINGUP_RETURN_KX 0.00020f
#define SWINGUP_RETURN_KDX 0.00180f

/* Update with measured Phase 1 dead-zone values (PWM units out of 7199). */
#define MOTOR_DEADZONE_FORWARD_PWM 80U
#define MOTOR_DEADZONE_REVERSE_PWM 80U

static float g_angle_zero_adc = ANGLE_ZERO_DEFAULT_ADC;
static float g_prev_theta_rad = 0.0f;
static float g_curr_dtheta_rad_s = 0.0f;
static float g_filt_dtheta_rad_s = 0.0f;
static float g_curr_theta_rad = 0.0f;
static float g_curr_u = 0.0f;
static float g_prev_u = 0.0f;
static uint16_t g_curr_adc = 0U;
static int32_t g_curr_enc = 0;
static uint16_t g_prev_enc_raw = 0U;
static float g_curr_x_vel = 0.0f;
static int32_t g_x_ref_enc = 0;
static uint32_t g_frame_count = 0U;
static uint8_t g_run_enabled = 0U;
static uint8_t g_user_latch = 0U;
static uint8_t g_menu_latch = 0U;
static uint8_t g_x_latch = 0U;
static uint8_t g_capture_mode = 0U;
static uint8_t g_capture_hold_count = 0U;
static uint8_t g_capture_lost_count = 0U;
static uint8_t g_upright_locked = 0U;
static uint8_t g_zero_set_ok = 1U;
static ControlMode g_remote_mode = CONTROL_MODE_IDLE;
static float g_remote_u = 0.0f;
static uint8_t g_remote_active_dbg = 0U;
static uint8_t g_rail_blocked = 0U;
static uint8_t g_jog_active_dbg = 0U;

static void BootStageBlink(uint8_t times)
{
  uint8_t i;

  for (i = 0U; i < times; i++) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    delay_ms(180);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    delay_ms(180);
  }
}

static uint16_t ReadAngleAdc(void)
{
  uint32_t adc = 0U;

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 5U) == HAL_OK) {
    adc = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);

  if (adc > 4095U) {
    adc = 4095U;
  }

  return (uint16_t)adc;
}

static uint8_t ButtonPressedLatched(GPIO_TypeDef *port, uint16_t pin, uint8_t *latch)
{
  GPIO_PinState key_state = HAL_GPIO_ReadPin(port, pin);

  if (key_state == GPIO_PIN_RESET) {
    if (*latch == 0U) {
      *latch = 1U;
      return 1U;
    }
  } else {
    *latch = 0U;
  }

  return 0U;
}

static float ClampFloat(float v, float vmin, float vmax)
{
  if (v < vmin) {
    return vmin;
  }
  if (v > vmax) {
    return vmax;
  }
  return v;
}

static float WrapPi(float x)
{
  while (x > 3.1415926f) {
    x -= 2.0f * 3.1415926f;
  }
  while (x < -3.1415926f) {
    x += 2.0f * 3.1415926f;
  }
  return x;
}

static float ThetaErrFromUpright(float theta)
{
  return WrapPi(theta - UPRIGHT_ANGLE_RAD);
}

static float SignWithFallback(float v, float fallback)
{
  if (v > 0.0f) {
    return 1.0f;
  }
  if (v < 0.0f) {
    return -1.0f;
  }
  return (fallback >= 0.0f) ? 1.0f : -1.0f;
}

static float ApplySwingWindowClamp(float u_cmd, float x_err, float x_dot)
{
  float x_over;
  float u_return;

  if (x_err > (float)SWINGUP_WINDOW_HALF_TICKS) {
    x_over = x_err - (float)SWINGUP_WINDOW_HALF_TICKS;
    u_return = -(SWINGUP_RETURN_KX * x_over + SWINGUP_RETURN_KDX * x_dot);
    return ClampFloat(u_return, -SWINGUP_RETURN_U_MAX, 0.0f);
  }

  if (x_err < -(float)SWINGUP_WINDOW_HALF_TICKS) {
    x_over = x_err + (float)SWINGUP_WINDOW_HALF_TICKS;
    u_return = -(SWINGUP_RETURN_KX * x_over + SWINGUP_RETURN_KDX * x_dot);
    return ClampFloat(u_return, 0.0f, SWINGUP_RETURN_U_MAX);
  }

  return ClampFloat(u_cmd, -SWINGUP_MAX_U, SWINGUP_MAX_U);
}

static float ComputeSwingUpU(float x_err, float x_dot, float theta, float dtheta)
{
  const float k_x = 0.000010f;
  const float k_dx = 0.00110f;
  const float energy_target = 2.0f * GRAVITY_MPS2 * PENDULUM_LENGTH_M;
  float energy;
  float energy_err;
  float phase;
  float phase_sign;
  float pump_u;
  float u_energy;
  float u_center;
  float u;

  /* Physical energy shaping with theta=0 at hanging-down and theta=pi at upright. */
  energy = (0.5f * PENDULUM_LENGTH_M * PENDULUM_LENGTH_M * dtheta * dtheta)
         + (GRAVITY_MPS2 * PENDULUM_LENGTH_M * (1.0f - cosf(theta)));
  energy_err = energy_target - energy;

  phase = dtheta * cosf(theta);
  phase_sign = SignWithFallback(phase, theta);

  /* Strong pump: acceleration follows energy error with a minimum drive to overcome stiction. */
  pump_u = (SWINGUP_ENERGY_TO_U * energy_err);
  if (energy_err > 0.0f) {
    pump_u += SWINGUP_MIN_PUMP_U;
  }
  pump_u = ClampFloat(pump_u, -SWINGUP_MAX_U, SWINGUP_MAX_U);
  u_energy = SWINGUP_U_SIGN * phase_sign * pump_u;

  /* Kick-start near hanging-down where dtheta can be too small for energy pumping to begin. */
  if ((fabsf(theta) < SWINGUP_KICK_TH_RAD) && (fabsf(dtheta) < SWINGUP_KICK_DTH_RAD_S)) {
    u_energy += SWINGUP_U_SIGN * SWINGUP_KICK_U * SignWithFallback(theta, 1.0f);
  }

  /* CART_MASS_KG placeholder kept explicit for next stage model-based tuning. */
  (void)CART_MASS_KG;

  /* Keep cart near x_ref while pumping energy. */
  u_center = -(k_x * x_err + k_dx * x_dot);

  u = u_energy + u_center;
  return ApplySwingWindowClamp(u, x_err, x_dot);
}

static float ApplySlewRate(float target_u)
{
  const float du_max = 0.35f;
  float du = target_u - g_prev_u;

  if (du > du_max) {
    du = du_max;
  } else if (du < -du_max) {
    du = -du_max;
  }

  g_prev_u += du;
  return g_prev_u;
}

static uint8_t IsSoftRailBlocked(int32_t enc_pos, float u_cmd)
{
  if ((enc_pos >= ENC_SOFT_LIMIT_POS) && (u_cmd > 0.0f)) {
    return 1U;
  }

  if ((enc_pos <= ENC_SOFT_LIMIT_NEG) && (u_cmd < 0.0f)) {
    return 1U;
  }

  return 0U;
}

static uint8_t TrySetZero(uint16_t adc_now)
{
#if POT_ZERO_STRICT_WINDOW
  if ((adc_now < POT_ZERO_MIN_ADC) || (adc_now > POT_ZERO_MAX_ADC)) {
    g_zero_set_ok = 0U;
    return 0U;
  }
#endif

  g_angle_zero_adc = (float)adc_now;
  g_zero_set_ok = 1U;
  return 1U;
}

static void BlinkSetZeroResult(uint8_t ok)
{
  if (ok != 0U) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    delay_ms(35);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    delay_ms(25);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    delay_ms(25);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    delay_ms(25);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
}

static void UpdateUprightLock(float theta_err_abs, float dtheta_abs)
{
  if (g_upright_locked == 0U) {
    if ((theta_err_abs < UPRIGHT_ENTER_TH_RAD) && (dtheta_abs < UPRIGHT_ENTER_DTH_RAD_S)) {
      if (g_capture_hold_count < 255U) {
        g_capture_hold_count++;
      }
    } else {
      g_capture_hold_count = 0U;
    }

    if (g_capture_hold_count >= UPRIGHT_ENTER_TICKS) {
      g_upright_locked = 1U;
      g_capture_lost_count = 0U;
    }
  } else {
    if ((theta_err_abs > UPRIGHT_EXIT_TH_RAD) || (dtheta_abs > UPRIGHT_EXIT_DTH_RAD_S)) {
      if (g_capture_lost_count < 255U) {
        g_capture_lost_count++;
      }
    } else {
      g_capture_lost_count = 0U;
    }

    if (g_capture_lost_count >= UPRIGHT_EXIT_TICKS) {
      g_upright_locked = 0U;
      g_capture_hold_count = 0U;
    }
  }

  g_capture_mode = g_upright_locked;
}

static void UpdateOledAngle(void)
{
  int32_t th_deg = (int32_t)(g_curr_theta_rad * (180.0f / 3.1415926f));
  int32_t dth_deg = (int32_t)(g_curr_dtheta_rad_s * (180.0f / 3.1415926f));
  int32_t u_pct = (int32_t)(g_curr_u * 100.0f);
  int32_t x_err = g_curr_enc - g_x_ref_enc;
  uint16_t th_abs = (uint16_t)((th_deg >= 0) ? th_deg : -th_deg);
  uint16_t dth_abs = (uint16_t)((dth_deg >= 0) ? dth_deg : -dth_deg);
  uint16_t u_abs = (uint16_t)((u_pct >= 0) ? u_pct : -u_pct);
  uint16_t x_abs = (uint16_t)((x_err >= 0) ? x_err : -x_err);

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"LQR ");
  OLED_ShowString(28, 0, (uint8_t *)(g_remote_active_dbg ? "R" : "L"));
  OLED_ShowString(40, 0, (uint8_t *)"R:");
  OLED_ShowNumber(56, 0, g_run_enabled, 1, 12);
  OLED_ShowString(72, 0, (uint8_t *)"C:");
  OLED_ShowNumber(88, 0, g_capture_mode, 1, 12);
  OLED_ShowString(100, 0, (uint8_t *)"M:");
  OLED_ShowNumber(116, 0, (uint16_t)g_remote_mode, 1, 12);

  OLED_ShowString(0, 12, (uint8_t *)"ADC:");
  OLED_ShowNumber(28, 12, g_curr_adc, 4, 12);
  OLED_ShowString(70, 12, (uint8_t *)"Z:");
  OLED_ShowNumber(84, 12, (uint16_t)g_angle_zero_adc, 4, 12);

  OLED_ShowString(0, 24, (uint8_t *)"TH:");
  OLED_ShowChar(20, 24, (th_deg >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(28, 24, th_abs, 3, 12);
  OLED_ShowString(56, 24, (uint8_t *)"d:");
  OLED_ShowChar(72, 24, (dth_deg >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(80, 24, dth_abs, 3, 12);

  OLED_ShowString(0, 36, (uint8_t *)"U%:");
  OLED_ShowChar(20, 36, (u_pct >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(28, 36, u_abs, 3, 12);
  OLED_ShowString(56, 36, (uint8_t *)"X:");
  OLED_ShowChar(72, 36, (x_err >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(80, 36, x_abs, 4, 12);

  OLED_ShowString(0, 48, (uint8_t *)"J:");
  OLED_ShowNumber(12, 48, g_jog_active_dbg, 1, 12);
  OLED_ShowString(24, 48, (uint8_t *)"RL:");
  OLED_ShowNumber(42, 48, g_rail_blocked, 1, 12);
  OLED_ShowString(54, 48, (uint8_t *)(POT_ZERO_STRICT_WINDOW ? "S1" : "S0"));
  OLED_ShowString(70, 48, (uint8_t *)(g_zero_set_ok ? "OK" : "ER"));
  OLED_ShowString(86, 48, (uint8_t *)"XREF");
  OLED_Refresh_Gram();
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

  /* Visible proof that firmware reached post-GPIO stage. */
  BootStageBlink(4U);
  
  /* Initialize OLED display */
  OLED_Init();
  delay_ms(1000);
  OLED_Clear();
  
  /* Start motor PWM and quadrature encoder */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /* Calibrate ADC once on startup for reliable potentiometer readings. */
  HAL_ADCEx_Calibration_Start(&hadc1);

  Motor_Init(&htim3, TIM_CHANNEL_4, BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin);
  Motor_SetDeadzonePwm(MOTOR_DEADZONE_FORWARD_PWM, MOTOR_DEADZONE_REVERSE_PWM);
  Motor_Stop();

  g_curr_enc = 0;
  g_prev_enc_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

  Comms_Init(&huart1);

  g_angle_zero_adc = ANGLE_ZERO_DEFAULT_ADC;

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"BOOT OK");
  OLED_ShowString(0, 12, (uint8_t *)"SWUP READY");
  OLED_ShowString(0, 24, (uint8_t *)"USER:RUN/STOP");
  OLED_ShowString(0, 36, (uint8_t *)"MENU:SET ZERO");
  OLED_Refresh_Gram();
  delay_ms(1200);

  OLED_Clear();
  OLED_ShowString(0, 12, (uint8_t *)"USER RUN/STOP");
  OLED_ShowString(0, 24, (uint8_t *)"MENU SET ZERO");
  OLED_ShowString(0, 36, (uint8_t *)"+/- JOG  X:XREF");
  OLED_Refresh_Gram();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint8_t oled_div = 0U;
    static uint8_t local_run_latched = 0U;
    static uint8_t user_force_stop = 0U;
    static uint16_t remote_miss_ticks = 0xFFFFU;
    static uint8_t run_prev = 0U;
    float u_cmd = 0.0f;
    float theta_err_abs;
    uint16_t enc_raw;
    int16_t enc_delta;
    ControlCommandData cmd;
    RobotStateData telem;
    uint8_t remote_active = 0U;
    uint8_t jog_plus_held;
    uint8_t jog_minus_held;
    uint8_t jog_active = 0U;

    Comms_Process();

    g_curr_adc = ReadAngleAdc();
    g_curr_theta_rad = (((float)g_curr_adc) - g_angle_zero_adc) * ADC_TO_RAD;
    g_curr_dtheta_rad_s = (g_curr_theta_rad - g_prev_theta_rad) / ((float)CONTROL_PERIOD_MS / 1000.0f);
    g_prev_theta_rad = g_curr_theta_rad;
    g_filt_dtheta_rad_s = 0.55f * g_filt_dtheta_rad_s + 0.45f * g_curr_dtheta_rad_s;
    theta_err_abs = fabsf(ThetaErrFromUpright(g_curr_theta_rad));


    UpdateUprightLock(theta_err_abs, fabsf(g_filt_dtheta_rad_s));

    enc_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    enc_delta = (int16_t)(enc_raw - g_prev_enc_raw);
    g_prev_enc_raw = enc_raw;
    g_curr_enc += (int32_t)enc_delta;
    g_curr_x_vel = ((float)enc_delta) / ((float)CONTROL_PERIOD_MS / 1000.0f);

    if (Comms_GetLatestCommand(&cmd) != 0U) {
      g_remote_mode = cmd.mode;
      g_remote_u = ClampFloat(cmd.control_effort, -1.0f, 1.0f);
      remote_miss_ticks = 0U;
    } else if (remote_miss_ticks < 0xFFFFU) {
      remote_miss_ticks++;
    }

    remote_active = (remote_miss_ticks <= (COMMAND_TIMEOUT_MS / CONTROL_PERIOD_MS)) ? 1U : 0U;
    g_remote_active_dbg = remote_active;

    jog_plus_held = (HAL_GPIO_ReadPin(pid_plus_GPIO_Port, pid_plus_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    jog_minus_held = (HAL_GPIO_ReadPin(pid_reduce_GPIO_Port, pid_reduce_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if (ButtonPressedLatched(User_key_GPIO_Port, User_key_Pin, &g_user_latch) != 0U) {
      user_force_stop = (user_force_stop == 0U) ? 1U : 0U;

      if (user_force_stop != 0U) {
        local_run_latched = 0U;
        g_remote_mode = CONTROL_MODE_IDLE;
        g_remote_u = 0.0f;
        remote_miss_ticks = 0xFFFFU;
        g_remote_active_dbg = 0U;
        g_run_enabled = 0U;
        g_curr_u = 0.0f;
        g_prev_u = 0.0f;
        Motor_Stop();
      } else {
        /* Resume from USER button with local run mode. */
        local_run_latched = 1U;
        g_remote_mode = CONTROL_MODE_IDLE;
        g_remote_u = 0.0f;
        remote_miss_ticks = 0xFFFFU;
        g_remote_active_dbg = 0U;
      }
    }

    if (ButtonPressedLatched(menu_key_GPIO_Port, menu_key_Pin, &g_menu_latch) != 0U) {
      BlinkSetZeroResult(TrySetZero(g_curr_adc));
    }

    if (ButtonPressedLatched(reserved_key_GPIO_Port, reserved_key_Pin, &g_x_latch) != 0U) {
      /* X key: store current cart position as LQR target x_ref. */
      g_x_ref_enc = g_curr_enc;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      delay_ms(20);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }

    if (user_force_stop != 0U) {
      g_run_enabled = 0U;
      g_curr_u = 0.0f;
      g_prev_u = 0.0f;
      g_upright_locked = 0U;
      g_capture_mode = 0U;
      g_capture_hold_count = 0U;
      g_capture_lost_count = 0U;
      Motor_Stop();
    } else if (remote_active != 0U) {
      g_run_enabled = (g_remote_mode != CONTROL_MODE_IDLE) ? 1U : 0U;

      if (g_run_enabled == 0U) {
        g_curr_u = 0.0f;
        g_prev_u = 0.0f;
        g_upright_locked = 0U;
        g_capture_mode = 0U;
        g_capture_hold_count = 0U;
        g_capture_lost_count = 0U;
        Motor_Stop();
      } else {
        if ((g_remote_mode == CONTROL_MODE_SWINGUP) || (g_remote_mode == CONTROL_MODE_LQR)) {
          /* Swing-up only phase: ignore LQR branch until identified model/gains are available. */
          u_cmd = ComputeSwingUpU((float)(g_curr_enc - g_x_ref_enc), g_curr_x_vel, g_curr_theta_rad, g_filt_dtheta_rad_s);
        } else {
          u_cmd = g_remote_u;
        }

        g_curr_u = ApplySlewRate(u_cmd);
      }
    } else {
      g_run_enabled = local_run_latched;

      if (g_run_enabled == 0U) {
        g_curr_u = 0.0f;
        g_prev_u = 0.0f;
        g_upright_locked = 0U;
        g_capture_mode = 0U;
        g_capture_hold_count = 0U;
        g_capture_lost_count = 0U;
        Motor_Stop();
      } else {
        u_cmd = ComputeSwingUpU((float)(g_curr_enc - g_x_ref_enc), g_curr_x_vel, g_curr_theta_rad, g_filt_dtheta_rad_s);

        g_curr_u = ApplySlewRate(u_cmd);
      }
    }

    if ((remote_active == 0U) && (g_run_enabled == 0U) && (user_force_stop == 0U)) {
      /* Local jog mode: hold +/- to move cart before balancing. */
      if (jog_plus_held != jog_minus_held) {
        jog_active = 1U;
        u_cmd = (jog_plus_held != 0U) ? JOG_U_CMD : -JOG_U_CMD;
        g_curr_u = ApplySlewRate(u_cmd);
      } else {
        g_curr_u = 0.0f;
        g_prev_u = 0.0f;
      }
    }

    g_jog_active_dbg = jog_active;

    if ((g_run_enabled != 0U) || (jog_active != 0U)) {
      g_rail_blocked = IsSoftRailBlocked(g_curr_enc, g_curr_u);
      if (g_rail_blocked != 0U) {
        g_curr_u = 0.0f;
        g_prev_u = 0.0f;
        Motor_Stop();
      } else {
        Motor_SetTorque(g_curr_u);
      }
    } else {
      g_rail_blocked = 0U;
    }

    if ((g_run_enabled != 0U) && (run_prev == 0U)) {
      g_x_ref_enc = g_curr_enc;
    }
    run_prev = g_run_enabled;

    telem.x_pos = (float)g_curr_enc;
    telem.x_vel = g_curr_x_vel;
    telem.theta = g_curr_theta_rad;
    telem.theta_vel = g_filt_dtheta_rad_s;
    Comms_SendTelemetry(&telem);

    g_frame_count++;
    if ((g_frame_count % 10U) == 0U) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    oled_div++;
    if (oled_div >= (OLED_PERIOD_MS / CONTROL_PERIOD_MS)) {
      oled_div = 0U;
      UpdateOledAngle();
    }

    delay_ms(CONTROL_PERIOD_MS);
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

