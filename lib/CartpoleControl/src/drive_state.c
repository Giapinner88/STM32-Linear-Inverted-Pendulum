#include "drive_state.h"

#include "cartpole_config.h"
#include "cartpole_math.h"

#include <math.h>

void DriveState_Init(DriveState *controller)
{
  DriveState_Reset(controller);
}

void DriveState_Reset(DriveState *controller)
{
  if (controller == 0) {
    return;
  }
  controller->previous_command = 0.0f;
  controller->upright_locked = 0U;
  controller->capture_hold_count = 0U;
  controller->capture_lost_count = 0U;
}

void DriveState_UpdateCapture(DriveState *controller,
                                    float angle_rad,
                                    float angle_rate_rad_s)
{
  float angle_error;
  float angle_error_abs;
  float rate_abs;

  if (controller == 0) {
    return;
  }

  angle_error = CartpoleMath_ThetaErrorFromUpright(angle_rad);
  angle_error_abs = fabsf(angle_error);
  rate_abs = fabsf(angle_rate_rad_s);

  if (controller->upright_locked == 0U) {
    if ((angle_error_abs < CARTPOLE_CAPTURE_ENTER_ANGLE_RAD)
        && (rate_abs < CARTPOLE_CAPTURE_ENTER_RATE_RAD_S)
        && (fabsf(angle_error + CARTPOLE_CAPTURE_LEAD_S * angle_rate_rad_s)
            < CARTPOLE_CAPTURE_ENTER_PREDICT_RAD)) {
      if (controller->capture_hold_count < 255U) {
        controller->capture_hold_count++;
      }
    } else {
      controller->capture_hold_count = 0U;
    }

    if (controller->capture_hold_count >= CARTPOLE_CAPTURE_ENTER_SAMPLES) {
      controller->upright_locked = 1U;
      controller->capture_lost_count = 0U;
    }
  } else {
    if ((angle_error_abs > CARTPOLE_CAPTURE_EXIT_ANGLE_RAD)
        || (rate_abs > CARTPOLE_CAPTURE_EXIT_RATE_RAD_S)) {
      if (controller->capture_lost_count < 255U) {
        controller->capture_lost_count++;
      }
    } else {
      controller->capture_lost_count = 0U;
    }

    if (controller->capture_lost_count >= CARTPOLE_CAPTURE_EXIT_SAMPLES) {
      controller->upright_locked = 0U;
      controller->capture_hold_count = 0U;
    }
  }
}

uint8_t DriveState_IsCaptured(const DriveState *controller)
{
  return (controller != 0) ? controller->upright_locked : 0U;
}

float DriveState_ApplySlew(DriveState *controller,
                                 float target_command)
{
  float delta;
  float slew_limit;

  if (controller == 0) {
    return 0.0f;
  }

  slew_limit = (controller->upright_locked != 0U)
             ? CARTPOLE_BALANCE_SLEW_PER_SAMPLE
             : CARTPOLE_SWINGUP_SLEW_PER_SAMPLE;
  delta = target_command - controller->previous_command;
  delta = CartpoleMath_Clamp(delta, -slew_limit, slew_limit);
  controller->previous_command += delta;
  return controller->previous_command;
}

