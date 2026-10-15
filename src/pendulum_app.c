#include "pendulum_app.h"

#include "main.h"
#include "cartpole_config.h"
#include "cartpole_math.h"
#include "cartpole_safety.h"
#include "comms.h"
#include "delay.h"
#include "drive_state.h"
#include "motor.h"
#include "pendulum_sequence.h"
#include "pendulum_ui.h"

#include <math.h>

static float g_angle_zero_adc = CARTPOLE_ANGLE_ZERO_DEFAULT_ADC;
static float g_curr_dtheta_rad_s = 0.0f;
static float g_filt_dtheta_rad_s = 0.0f;
static float g_curr_theta_rad = 0.0f;
static float g_prev_theta_wrapped_rad = 0.0f;
static float g_theta_unwrapped_rad = 0.0f;
static uint8_t g_theta_unwrap_init = 0U;
static float g_curr_u = 0.0f;
static float g_peak_abs_u = 0.0f;
static uint16_t g_curr_adc = 0U;
static int32_t g_curr_enc = 0;
static uint16_t g_prev_enc_raw = 0U;
static float g_curr_x_vel = 0.0f;
static float g_theta_history_rad[CARTPOLE_RATE_WINDOW_SAMPLES];
static int32_t g_enc_history[CARTPOLE_RATE_WINDOW_SAMPLES];
static uint32_t g_tick_history_ms[CARTPOLE_RATE_WINDOW_SAMPLES];
static uint8_t g_history_index = 0U;
static uint8_t g_history_count = 0U;
static uint32_t g_frame_count = 0U;
static uint8_t g_run_enabled = 0U;
static uint8_t g_user_latch = 0U;
static uint8_t g_menu_latch = 0U;
static uint8_t g_calibration_mode = 1U;
static int32_t g_right_home_enc = CARTPOLE_ENCODER_RIGHT_WALL_TICKS;
static uint8_t g_home_return_active = 0U;
static ControlMode g_remote_mode = CONTROL_MODE_IDLE;
static float g_remote_u = 0.0f;
static uint16_t g_remote_miss_frames = 0xFFFFU;
static float g_direct_u = 0.0f;
static uint16_t g_direct_age_frames = 0xFFFFU;
static uint8_t g_direct_in_use = 0U;
static ControlMode g_direct_mode = CONTROL_MODE_IDLE;
static uint8_t g_last_cmd_seq = 0U;
static uint8_t g_rail_blocked = 0U;
static int32_t g_control_center_enc = 0;
static uint8_t g_control_center_valid = 0U;
static DriveState g_controller;
static PendulumSequence g_sequence;

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;

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
  uint32_t adc_sum = 0U;
  uint16_t adc;
  uint8_t valid_samples = 0U;
  uint8_t i;

  for (i = 0U; i < CARTPOLE_ANGLE_ADC_AVERAGE_SAMPLES; ++i) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5U) == HAL_OK) {
      adc = (uint16_t)HAL_ADC_GetValue(&hadc1);
      adc_sum += (uint32_t)((adc > 4095U) ? 4095U : adc);
      valid_samples++;
    }
    HAL_ADC_Stop(&hadc1);
  }

  return (valid_samples != 0U)
       ? (uint16_t)(adc_sum / (uint32_t)valid_samples) : 0U;
}

static uint8_t ButtonPressedLatched(GPIO_TypeDef *port, uint16_t pin,
                                    uint8_t *latch)
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

static float UnwrapThetaSample(float theta_wrapped)
{
  float delta;

  if (g_theta_unwrap_init == 0U) {
    g_prev_theta_wrapped_rad = theta_wrapped;
    g_theta_unwrapped_rad = theta_wrapped;
    g_theta_unwrap_init = 1U;
    return g_theta_unwrapped_rad;
  }

  delta = theta_wrapped - g_prev_theta_wrapped_rad;
  if (delta > CARTPOLE_PI_F) {
    delta -= 2.0f * CARTPOLE_PI_F;
  } else if (delta < -CARTPOLE_PI_F) {
    delta += 2.0f * CARTPOLE_PI_F;
  }

  g_theta_unwrapped_rad += delta;
  g_prev_theta_wrapped_rad = theta_wrapped;
  return g_theta_unwrapped_rad;
}

static uint8_t TrySetZero(uint16_t adc_now)
{
#if CARTPOLE_POT_ZERO_STRICT_WINDOW
  if ((adc_now < CARTPOLE_POT_ZERO_MIN_ADC)
      || (adc_now > CARTPOLE_POT_ZERO_MAX_ADC)) {
    return 0U;
  }
#endif
  g_angle_zero_adc = (float)adc_now;
  return 1U;
}

static void BlinkSetZeroResult(uint8_t ok)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  delay_ms((ok != 0U) ? 35U : 25U);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  if (ok == 0U) {
    delay_ms(25U);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    delay_ms(25U);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
}

/* Differencing across a multi-frame window keeps the old 20 ms resolution
 * on ADC/encoder quantisation while refreshing the estimate every frame. */
static void UpdateRateEstimates(uint32_t now_ms)
{
  uint8_t oldest;
  uint32_t elapsed_ms;
  float elapsed_s;

  g_curr_dtheta_rad_s = 0.0f;
  g_curr_x_vel = 0.0f;
  if (g_history_count != 0U) {
    oldest = (g_history_count < CARTPOLE_RATE_WINDOW_SAMPLES)
           ? 0U : g_history_index;
    elapsed_ms = now_ms - g_tick_history_ms[oldest];
    if (elapsed_ms != 0U) {
      elapsed_s = (float)elapsed_ms / 1000.0f;
      g_curr_dtheta_rad_s = (g_curr_theta_rad - g_theta_history_rad[oldest])
                          / elapsed_s;
      g_curr_x_vel = (float)(g_curr_enc - g_enc_history[oldest]) / elapsed_s;
    }
  }

  g_theta_history_rad[g_history_index] = g_curr_theta_rad;
  g_enc_history[g_history_index] = g_curr_enc;
  g_tick_history_ms[g_history_index] = now_ms;
  g_history_index = (uint8_t)((g_history_index + 1U)
                              % CARTPOLE_RATE_WINDOW_SAMPLES);
  if (g_history_count < CARTPOLE_RATE_WINDOW_SAMPLES) {
    g_history_count++;
  }

  g_filt_dtheta_rad_s = 0.5f * g_filt_dtheta_rad_s
                      + 0.5f * g_curr_dtheta_rad_s;
}

/* Fixed-rate scheduling: frame work time does not stretch the period. */
static void WaitForNextFrame(void)
{
  static uint32_t next_frame_ms = 0U;
  static uint8_t scheduler_started = 0U;
  uint32_t now_ms = HAL_GetTick();

  if (scheduler_started == 0U) {
    next_frame_ms = now_ms;
    scheduler_started = 1U;
  }
  next_frame_ms += CARTPOLE_CONTROL_PERIOD_MS;
  if ((int32_t)(next_frame_ms - now_ms) <= 0) {
    /* Overrun (e.g. blocking LED feedback): resync, no catch-up burst. */
    next_frame_ms = now_ms;
    return;
  }
  while ((int32_t)(next_frame_ms - HAL_GetTick()) > 0) {
  }
}

static void ResetMotionEstimates(void)
{
  g_history_index = 0U;
  g_history_count = 0U;
  g_curr_dtheta_rad_s = 0.0f;
  g_filt_dtheta_rad_s = 0.0f;
  g_curr_theta_rad = 0.0f;
  g_prev_theta_wrapped_rad = 0.0f;
  g_theta_unwrapped_rad = 0.0f;
  g_theta_unwrap_init = 0U;
  g_prev_enc_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  g_curr_x_vel = 0.0f;
}

static void StopAutomaticSequence(void)
{
  PendulumSequence_Stop(&g_sequence);
  g_calibration_mode = 1U;
  g_run_enabled = 0U;
  g_curr_u = 0.0f;
  DriveState_Reset(&g_controller);
  Motor_Stop();
}

static void StartAutomaticCalibration(void)
{
  PendulumSequence_StartCalibration(&g_sequence);
  g_calibration_mode = 1U;
  g_run_enabled = 0U;
  g_control_center_valid = 0U;
  g_curr_u = 0.0f;
  DriveState_Reset(&g_controller);
  Motor_Stop();
}

static void UpdateAutomaticCalibration(void)
{
  PendulumSequenceEvent event;

  event = PendulumSequence_Update(&g_sequence, g_curr_adc, g_curr_enc);
  if (event == PENDULUM_SEQUENCE_EVENT_CALIBRATED) {
    g_angle_zero_adc = g_sequence.angle_zero_adc;
    g_control_center_enc = g_sequence.control_center_encoder;
    g_control_center_valid = 1U;
    g_calibration_mode = 0U;
    g_run_enabled = 1U;
    g_curr_u = 0.0f;
    DriveState_Reset(&g_controller);
    ResetMotionEstimates();
  } else if (event == PENDULUM_SEQUENCE_EVENT_CAL_TIMEOUT) {
    g_calibration_mode = 1U;
    g_run_enabled = 0U;
    g_curr_u = 0.0f;
    DriveState_Reset(&g_controller);
    Motor_Stop();
  }
}

static void SetCalibrationMode(uint8_t enabled)
{
  g_calibration_mode = (enabled != 0U) ? 1U : 0U;
  if (g_calibration_mode != 0U) {
    PendulumSequence_Stop(&g_sequence);
  }
  g_run_enabled = 0U;
  g_curr_u = 0.0f;
  g_rail_blocked = 0U;
  DriveState_Reset(&g_controller);
  Motor_Stop();
}

static void StartRightWallHoming(void)
{
  PendulumSequence_Stop(&g_sequence);
  g_control_center_valid = 0U;
  g_home_return_active = 1U;
  g_calibration_mode = 0U;
  g_run_enabled = 1U;
  g_curr_u = 0.0f;
  g_rail_blocked = 0U;
  DriveState_Reset(&g_controller);
}

static float ComputeRightWallHomeU(int32_t encoder_position)
{
  float remaining_ticks;
  float normalized_distance;

  remaining_ticks = (float)(encoder_position - g_right_home_enc);
  if (remaining_ticks <= (float)CARTPOLE_RAIL_MARGIN_TICKS) {
    return 0.0f;
  }

  normalized_distance = (remaining_ticks - (float)CARTPOLE_RAIL_MARGIN_TICKS)
                      / CARTPOLE_HOMING_DECAY_TICKS;
  normalized_distance = CartpoleMath_Clamp(normalized_distance, 0.0f, 1.0f);
  normalized_distance *= normalized_distance;

  return CARTPOLE_HOMING_MIN_U
       + ((CARTPOLE_HOMING_MAX_U - CARTPOLE_HOMING_MIN_U)
          * normalized_distance);
}

static int32_t GetCartPositionFromControlCenter(void)
{
  int32_t reference = (g_control_center_valid != 0U)
                    ? g_control_center_enc : g_right_home_enc;
  return g_curr_enc - reference;
}

static PendulumUiMode GetUiMode(uint8_t remote_active)
{
  if (g_home_return_active != 0U) {
    return PENDULUM_UI_HOMING;
  }
  if (g_sequence.state == PENDULUM_SEQUENCE_CALIBRATING) {
    return PENDULUM_UI_CALIBRATING;
  }
  if (g_sequence.state == PENDULUM_SEQUENCE_CAL_ERROR) {
    return PENDULUM_UI_CAL_ERROR;
  }
  if ((g_sequence.state == PENDULUM_SEQUENCE_RUNNING)
      || ((remote_active != 0U) && (g_run_enabled != 0U))) {
    return (DriveState_IsCaptured(&g_controller) != 0U)
         ? PENDULUM_UI_BALANCE : PENDULUM_UI_SWINGUP;
  }
  return PENDULUM_UI_IDLE;
}

static void RenderUi(uint8_t remote_active)
{
  PendulumUiSnapshot snapshot;

  snapshot.mode = GetUiMode(remote_active);
  snapshot.remote_active = remote_active;
  snapshot.capture_active = DriveState_IsCaptured(&g_controller);
  snapshot.remote_mode = (uint8_t)g_remote_mode;
  snapshot.angle_adc = g_curr_adc;
  snapshot.angle_zero_adc = g_angle_zero_adc;
  snapshot.angle_rad = g_curr_theta_rad;
  snapshot.angle_rate_rad_s = g_curr_dtheta_rad_s;
  snapshot.motor_command = g_curr_u;
  snapshot.peak_motor_command = g_peak_abs_u;
  snapshot.cart_position_ticks = GetCartPositionFromControlCenter();
  snapshot.control_center_ticks = (g_control_center_valid != 0U)
                                ? g_control_center_enc : g_right_home_enc;
  snapshot.cart_velocity_ticks_s = g_curr_x_vel;
  PendulumUi_Render(&snapshot);
  g_peak_abs_u = 0.0f;
}

void PendulumApp_Init(void)
{
  delay_ms(200);
  BootStageBlink(4U);
  PendulumUi_Init();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_ADCEx_Calibration_Start(&hadc1);

  Motor_Init(&htim3, TIM_CHANNEL_4,
             BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin);
  Motor_SetDeadzonePwm(CARTPOLE_MOTOR_DEADZONE_FORWARD_PWM,
                       CARTPOLE_MOTOR_DEADZONE_REVERSE_PWM);
  Motor_Stop();

  g_curr_enc = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  g_prev_enc_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  DriveState_Init(&g_controller);
  PendulumSequence_Init(&g_sequence);
  Comms_Init(&huart1);
  PendulumUi_ShowStartup();
}

/* Samples the angle pot and encoder, then refreshes rates and capture. */
static void ReadSensors(void)
{
  uint16_t encoder_raw;
  int16_t encoder_delta;

  g_curr_adc = ReadAngleAdc();
  g_curr_theta_rad = UnwrapThetaSample(
    ((float)g_curr_adc - g_angle_zero_adc) * CARTPOLE_ADC_TO_RAD);

  encoder_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  encoder_delta = (int16_t)(encoder_raw - g_prev_enc_raw);
  g_prev_enc_raw = encoder_raw;
  g_curr_enc += (int32_t)encoder_delta;

  UpdateRateEstimates(HAL_GetTick());
  DriveState_UpdateCapture(&g_controller, g_curr_theta_rad,
                                  g_filt_dtheta_rad_s);
}

static void PollRemoteCommand(void)
{
  ControlCommandData command;

  if (Comms_GetLatestCommand(&command) != 0U) {
    g_remote_mode = command.mode;
    g_remote_u = (isfinite(command.control_effort) != 0)
               ? CartpoleMath_Clamp(command.control_effort, -1.0f, 1.0f)
               : 0.0f;
    g_remote_miss_frames = 0U;
    g_last_cmd_seq = command.seq;
    if ((command.mode == CONTROL_MODE_FULL)
        && (isfinite(command.control_effort) != 0)) {
      g_direct_mode = command.mode;
      g_direct_u = g_remote_u;
      g_direct_age_frames = 0U;
      return;
    }
    /* IDLE, unsupported modes and non-finite commands revoke the old command. */
    g_direct_age_frames = 0xFFFFU;
    g_direct_u = 0.0f;
  }
  if (g_remote_miss_frames < 0xFFFFU) {
    g_remote_miss_frames++;
  }
  if (g_direct_age_frames < 0xFFFFU) {
    g_direct_age_frames++;
  }
}

static void DropRemoteControl(void)
{
  g_remote_mode = CONTROL_MODE_IDLE;
  g_remote_u = 0.0f;
  g_remote_miss_frames = 0xFFFFU;
}

static uint8_t IsSequenceBusy(void)
{
  return ((g_sequence.state == PENDULUM_SEQUENCE_CALIBRATING)
          || (g_sequence.state == PENDULUM_SEQUENCE_RUNNING)) ? 1U : 0U;
}

/* The on-board sequence always has priority over a PC link. */
static uint8_t IsRemoteActive(void)
{
  return ((g_remote_miss_frames
           <= (CARTPOLE_COMMAND_TIMEOUT_MS / CARTPOLE_CONTROL_PERIOD_MS))
          && (IsSequenceBusy() == 0U)) ? 1U : 0U;
}

/* USER starts/stops the automatic sequence; M1 re-zeroes the angle. */
static void HandleUserAndMenuKeys(void)
{
  if (ButtonPressedLatched(User_key_GPIO_Port, User_key_Pin,
                           &g_user_latch) != 0U) {
    DropRemoteControl();
    if (IsSequenceBusy() != 0U) {
      StopAutomaticSequence();
    } else {
      StartAutomaticCalibration();
    }
  }

  if ((ButtonPressedLatched(menu_key_GPIO_Port, menu_key_Pin,
                            &g_menu_latch) != 0U)
      && (IsSequenceBusy() == 0U)) {
    BlinkSetZeroResult(TrySetZero(g_curr_adc));
  }
}

/* Reserved key: short press toggles calibration mode, long press homes. */
static void HandleReservedKey(void)
{
  static uint16_t hold_frames = 0U;

  if (HAL_GPIO_ReadPin(reserved_key_GPIO_Port,
                       reserved_key_Pin) == GPIO_PIN_RESET) {
    if (hold_frames < 0xFFFFU) {
      hold_frames++;
    }
  } else if (hold_frames > 0U) {
    if (hold_frames >= CARTPOLE_RESERVED_KEY_LONG_PRESS_SAMPLES) {
      StartRightWallHoming();
    } else {
      SetCalibrationMode((g_calibration_mode == 0U) ? 1U : 0U);
    }
    hold_frames = 0U;
  }
}

static void StopDrive(void)
{
  g_curr_u = 0.0f;
  DriveState_Reset(&g_controller);
  Motor_Stop();
}

/* The laptop owns all control phases. An expired command stops the drive;
 * capture remains telemetry/slew state only, never a controller switch. */
static uint8_t HasFreshNmpcCommand(void)
{
  return ((g_direct_mode == CONTROL_MODE_FULL)
      && (g_direct_age_frames
          <= (CARTPOLE_DIRECT_TIMEOUT_MS / CARTPOLE_CONTROL_PERIOD_MS))) ? 1U : 0U;
}

/* Jog buttons move the cart only while nothing else owns the motor.
 * Returns 1 while a jog is in progress. */
static uint8_t UpdateJog(void)
{
  uint8_t plus_held = (HAL_GPIO_ReadPin(pid_plus_GPIO_Port, pid_plus_Pin)
                       == GPIO_PIN_RESET) ? 1U : 0U;
  uint8_t minus_held = (HAL_GPIO_ReadPin(pid_reduce_GPIO_Port, pid_reduce_Pin)
                        == GPIO_PIN_RESET) ? 1U : 0U;

  if (plus_held == minus_held) {
    g_curr_u = 0.0f;
    DriveState_Reset(&g_controller);
    return 0U;
  }
  g_curr_u = DriveState_ApplySlew(
    &g_controller, (plus_held != 0U) ? CARTPOLE_JOG_U : -CARTPOLE_JOG_U);
  return 1U;
}

/* Chooses the command source by priority: homing, calibration, PC link,
 * then the automatic sequence. Returns 1 when one of them drives the motor. */
static uint8_t SelectControllerCommand(uint8_t remote_active)
{
  g_direct_in_use = 0U;
  if (g_home_return_active != 0U) {
    if (g_curr_enc <= (g_right_home_enc + CARTPOLE_RAIL_MARGIN_TICKS)) {
      g_home_return_active = 0U;
      g_run_enabled = 0U;
      StopDrive();
      return 0U;
    }
    g_curr_u = DriveState_ApplySlew(
      &g_controller, ComputeRightWallHomeU(g_curr_enc));
    return 1U;
  }

  if ((g_sequence.state == PENDULUM_SEQUENCE_CALIBRATING)
      || (g_calibration_mode != 0U)) {
    g_run_enabled = 0U;
    g_rail_blocked = 0U;
    StopDrive();
    return 0U;
  }

  if (remote_active != 0U) {
    g_run_enabled = 0U;  /* Commands cannot bypass the USER start sequence. */
  } else {
    g_run_enabled = (g_sequence.state == PENDULUM_SEQUENCE_RUNNING) ? 1U : 0U;
  }
  if (g_run_enabled != 0U) {
    if (HasFreshNmpcCommand() == 0U) {
      StopDrive();  /* Immediate output zero, not a slew toward zero. */
      return 1U;
    }
    g_direct_in_use = 1U;
    g_curr_u = DriveState_ApplySlew(&g_controller, g_direct_u);
    return 1U;
  }

  StopDrive();
  return 0U;
}

/* Manual jog is allowed whenever no controller owns the motor, including
 * calibration mode, but not during the PC link or auto-calibration. */
static uint8_t UpdateMotorCommand(uint8_t remote_active)
{
  if (SelectControllerCommand(remote_active) != 0U) {
    return 1U;
  }
  if ((remote_active == 0U)
      && (g_sequence.state != PENDULUM_SEQUENCE_CALIBRATING)) {
    return UpdateJog();
  }
  return 0U;
}

/* Rail guard sits between every command source and the H-bridge. */
static void DriveMotor(uint8_t drive_active)
{
  if (drive_active == 0U) {
    g_rail_blocked = 0U;
    return;
  }

  g_rail_blocked = CartpoleSafety_IsRailBlocked(
    g_curr_enc, g_curr_u, g_control_center_enc, g_control_center_valid);
  if (g_rail_blocked != 0U) {
    StopDrive();
  } else {
    Motor_SetTorque(g_curr_u);
  }
}

static void PublishTelemetry(void)
{
  RobotStateData telemetry;

  telemetry.x_pos = (float)g_curr_enc;
  telemetry.x_vel = g_curr_x_vel;
  telemetry.theta = g_curr_theta_rad;
  telemetry.theta_vel = g_filt_dtheta_rad_s;
  telemetry.control_effort = g_curr_u;
  telemetry.frame = (uint16_t)g_frame_count;
  telemetry.flags = (uint8_t)(
    ((DriveState_IsCaptured(&g_controller) != 0U)
       ? TELEMETRY_FLAG_CAPTURED : 0U)
    | ((g_run_enabled != 0U) ? TELEMETRY_FLAG_RUNNING : 0U)
    | ((g_rail_blocked != 0U) ? TELEMETRY_FLAG_RAIL_BLOCKED : 0U)
    | ((g_direct_in_use != 0U) ? TELEMETRY_FLAG_DIRECT_ACTIVE : 0U));
  telemetry.cmd_seq = g_last_cmd_seq;
  telemetry.center_ticks = (g_control_center_valid != 0U)
                         ? (int16_t)g_control_center_enc : 0;
  Comms_SendTelemetry(&telemetry);
}

static void UpdateIndicators(uint8_t remote_active)
{
  static uint8_t oled_div = 0U;

  if (fabsf(g_curr_u) > g_peak_abs_u) {
    g_peak_abs_u = fabsf(g_curr_u);
  }

  g_frame_count++;
  if ((g_frame_count % CARTPOLE_LED_TOGGLE_SAMPLES) == 0U) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }

  oled_div++;
  if (oled_div >= (CARTPOLE_OLED_PERIOD_MS / CARTPOLE_CONTROL_PERIOD_MS)) {
    oled_div = 0U;
    RenderUi(remote_active);
  }
  PendulumUi_FlushStep();
}

void PendulumApp_RunFrame(void)
{
  uint8_t remote_active;
  uint8_t drive_active;

  Comms_Process();
  ReadSensors();
  PollRemoteCommand();
  HandleUserAndMenuKeys();
  UpdateAutomaticCalibration();
  remote_active = IsRemoteActive();
  HandleReservedKey();

  drive_active = UpdateMotorCommand(remote_active);
  DriveMotor(drive_active);

  PublishTelemetry();
  UpdateIndicators(remote_active);
  WaitForNextFrame();
}
