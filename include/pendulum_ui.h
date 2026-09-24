#ifndef PENDULUM_UI_H
#define PENDULUM_UI_H

#include <stdint.h>

typedef enum {
  PENDULUM_UI_IDLE = 0,
  PENDULUM_UI_CALIBRATING,
  PENDULUM_UI_SWINGUP,
  PENDULUM_UI_BALANCE,
  PENDULUM_UI_CAL_ERROR,
  PENDULUM_UI_HOMING
} PendulumUiMode;

typedef struct {
  PendulumUiMode mode;
  uint8_t remote_active;
  uint8_t capture_active;
  uint8_t remote_mode;
  uint16_t angle_adc;
  float angle_zero_adc;
  float angle_rad;
  float angle_rate_rad_s;
  float motor_command;
  float peak_motor_command;
  int32_t cart_position_ticks;
  int32_t control_center_ticks;
  float cart_velocity_ticks_s;
} PendulumUiSnapshot;

void PendulumUi_Init(void);
void PendulumUi_ShowStartup(void);
void PendulumUi_Render(const PendulumUiSnapshot *snapshot);
/* Sends one OLED page per call so a full refresh never blocks a frame. */
void PendulumUi_FlushStep(void);

#endif /* PENDULUM_UI_H */
