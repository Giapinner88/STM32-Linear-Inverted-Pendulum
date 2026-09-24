#include "hybrid_controller.h"

#include "cartpole_config.h"
#include "cartpole_math.h"
#include "lqr_controller.h"
#include "swingup_controller.h"

#include <math.h>

void HybridController_Init(HybridController *controller)
{
  HybridController_Reset(controller);
}

void HybridController_Reset(HybridController *controller)
{
  if (controller == 0) {
    return;
  }
  controller->previous_command = 0.0f;
  SwingupController_Reset(&controller->swingup);
  controller->upright_locked = 0U;
  controller->capture_hold_count = 0U;
  controller->capture_lost_count = 0U;
}

void HybridController_UpdateCapture(HybridController *controller,
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

uint8_t HybridController_IsCaptured(const HybridController *controller)
{
  return (controller != 0) ? controller->upright_locked : 0U;
}

float HybridController_Compute(HybridController *controller,
                               float cart_position_ticks,
                               float cart_velocity_ticks_s,
                               float angle_rad,
                               float angle_rate_rad_s)
{
  if (HybridController_IsCaptured(controller) != 0U) {
    return LqrController_Compute(cart_position_ticks, cart_velocity_ticks_s,
                                 angle_rad, angle_rate_rad_s);
  }
  return HybridController_ComputeSwingup(controller, cart_position_ticks,
                                         cart_velocity_ticks_s, angle_rad,
                                         angle_rate_rad_s);
}

float HybridController_ComputeSwingup(HybridController *controller,
                                      float cart_position_ticks,
                                      float cart_velocity_ticks_s,
                                      float angle_rad,
                                      float angle_rate_rad_s)
{
  if (controller == 0) {
    return 0.0f;
  }

  return SwingupController_Compute(&controller->swingup,
                                   cart_position_ticks,
                                   cart_velocity_ticks_s,
                                   angle_rad,
                                   angle_rate_rad_s);
}

float HybridController_ApplySlew(HybridController *controller,
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
