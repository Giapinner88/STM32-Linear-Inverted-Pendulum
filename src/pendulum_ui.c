#include "pendulum_ui.h"

#include "cartpole_config.h"
#include "delay.h"
#include "oled.h"

static const uint8_t *ModeLabel(PendulumUiMode mode)
{
  switch (mode) {
    case PENDULUM_UI_CALIBRATING:
      return (uint8_t *)"CAL";
    case PENDULUM_UI_SWINGUP:
      return (uint8_t *)"SWG";
    case PENDULUM_UI_BALANCE:
      return (uint8_t *)"BAL";
    case PENDULUM_UI_CAL_ERROR:
      return (uint8_t *)"ERR";
    case PENDULUM_UI_HOMING:
      return (uint8_t *)"HOM";
    case PENDULUM_UI_IDLE:
    default:
      return (uint8_t *)"IDL";
  }
}

void PendulumUi_Init(void)
{
  OLED_Init();
  delay_ms(1000);
  OLED_Clear();
}

void PendulumUi_ShowStartup(void)
{
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"BOOT OK");
  OLED_ShowString(0, 12, (uint8_t *)"CART AT CENTRE");
  OLED_ShowString(0, 24, (uint8_t *)"PENDULUM DOWN");
  OLED_ShowString(0, 36, (uint8_t *)"USER:AUTO START");
  OLED_Refresh_Gram();
  delay_ms(1200);

  OLED_Clear();
  OLED_ShowString(0, 12, (uint8_t *)"CART AT CENTRE");
  OLED_ShowString(0, 24, (uint8_t *)"DOWN ADC 990-1060");
  OLED_ShowString(0, 36, (uint8_t *)"PRESS USER");
  OLED_Refresh_Gram();
}

void PendulumUi_Render(const PendulumUiSnapshot *snapshot)
{
  int32_t angle_deg;
  int32_t angle_rate_deg_s;
  int32_t command_percent;
  int32_t peak_command_percent;
  int32_t cart_velocity;

  if (snapshot == 0) {
    return;
  }

  angle_deg = (int32_t)(snapshot->angle_rad
              * (180.0f / CARTPOLE_PI_F));
  angle_rate_deg_s = (int32_t)(snapshot->angle_rate_rad_s
                     * (180.0f / CARTPOLE_PI_F));
  command_percent = (int32_t)(snapshot->motor_command * 100.0f);
  peak_command_percent = (int32_t)(snapshot->peak_motor_command * 100.0f);
  cart_velocity = (int32_t)snapshot->cart_velocity_ticks_s;

  OLED_ClearGram();
  OLED_ShowString(0, 0, (uint8_t *)ModeLabel(snapshot->mode));
  OLED_ShowString(28, 0, (uint8_t *)(snapshot->remote_active ? "R" : "L"));
  OLED_ShowString(40, 0, (uint8_t *)"C:");
  OLED_ShowNumber(56, 0, snapshot->capture_active, 1, 12);
  OLED_ShowString(72, 0, (uint8_t *)"P:");
  OLED_ShowNumber(88, 0, (uint16_t)peak_command_percent, 3, 12);

  OLED_ShowString(0, 12, (uint8_t *)"ADC:");
  OLED_ShowNumber(28, 12, snapshot->angle_adc, 4, 12);
  OLED_ShowString(70, 12, (uint8_t *)"Z:");
  OLED_ShowNumber(84, 12, (uint16_t)snapshot->angle_zero_adc, 4, 12);

  OLED_ShowString(0, 24, (uint8_t *)"TH:");
  OLED_ShowChar(20, 24, (angle_deg >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(28, 24, (uint16_t)((angle_deg >= 0) ? angle_deg : -angle_deg), 3, 12);
  OLED_ShowString(56, 24, (uint8_t *)"d:");
  OLED_ShowChar(72, 24, (angle_rate_deg_s >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(80, 24,
                  (uint16_t)((angle_rate_deg_s >= 0)
                             ? angle_rate_deg_s : -angle_rate_deg_s),
                  3, 12);

  OLED_ShowString(0, 36, (uint8_t *)"U%:");
  OLED_ShowChar(20, 36, (command_percent >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(28, 36,
                  (uint16_t)((command_percent >= 0)
                             ? command_percent : -command_percent),
                  3, 12);
  OLED_ShowString(56, 36, (uint8_t *)"X:");
  OLED_ShowChar(72, 36, (snapshot->cart_position_ticks >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(80, 36,
                  (uint16_t)((snapshot->cart_position_ticks >= 0)
                             ? snapshot->cart_position_ticks
                             : -snapshot->cart_position_ticks),
                  5, 12);

  OLED_ShowString(0, 48, (uint8_t *)"CTR:");
  OLED_ShowChar(20, 48, (snapshot->control_center_ticks >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(28, 48,
                  (uint16_t)((snapshot->control_center_ticks >= 0)
                             ? snapshot->control_center_ticks
                             : -snapshot->control_center_ticks),
                  5, 12);
  OLED_ShowString(72, 48, (uint8_t *)"dX:");
  OLED_ShowChar(96, 48, (cart_velocity >= 0) ? '+' : '-', 12, 1);
  OLED_ShowNumber(104, 48,
                  (uint16_t)((cart_velocity >= 0)
                             ? cart_velocity : -cart_velocity),
                  4, 12);
}

void PendulumUi_FlushStep(void)
{
  static uint8_t page = 0U;

  OLED_Refresh_Page(page);
  page = (uint8_t)((page + 1U) & 7U);
}
