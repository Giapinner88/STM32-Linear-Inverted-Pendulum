#include "swingup_controller.h"

#include "cartpole_config.h"
#include "cartpole_coordinates.h"
#include "cartpole_math.h"

#include <math.h>

static float SignFloat(float value)
{
  if (value > 0.0f) {
    return 1.0f;
  }
  if (value < 0.0f) {
    return -1.0f;
  }
  return 0.0f;
}

static float ComputeCartReturn(SwingupController *controller,
                               float position_ticks,
                               float velocity_ticks_s)
{
  float return_command;

  if (controller->return_direction < 0) {
    return_command = -(CARTPOLE_SWINGUP_RETURN_K_X * position_ticks
                     + CARTPOLE_SWINGUP_RETURN_K_V * velocity_ticks_s);
    return_command = CartpoleMath_Clamp(
      return_command, -CARTPOLE_SWINGUP_RETURN_MAX_U, 0.0f);
    if ((return_command < 0.0f)
        && (return_command > -CARTPOLE_SWINGUP_RETURN_MIN_U)) {
      return_command = -CARTPOLE_SWINGUP_RETURN_MIN_U;
    }
    return return_command;
  }

  if (controller->return_direction > 0) {
    return_command = -(CARTPOLE_SWINGUP_RETURN_K_X * position_ticks
                     + CARTPOLE_SWINGUP_RETURN_K_V * velocity_ticks_s);
    return_command = CartpoleMath_Clamp(
      return_command, 0.0f, CARTPOLE_SWINGUP_RETURN_MAX_U);
    if ((return_command > 0.0f)
        && (return_command < CARTPOLE_SWINGUP_RETURN_MIN_U)) {
      return_command = CARTPOLE_SWINGUP_RETURN_MIN_U;
    }
    return return_command;
  }

  return 0.0f;
}

void SwingupController_Init(SwingupController *controller)
{
  SwingupController_Reset(controller);
}

void SwingupController_Reset(SwingupController *controller)
{
  if (controller == 0) {
    return;
  }

  controller->pump_direction = 1;
  controller->return_direction = 0;
}

float SwingupController_Compute(SwingupController *controller,
                                float cart_position_ticks,
                                float cart_velocity_ticks_s,
                                float angle_rad,
                                float angle_rate_rad_s)
{
  float acceleration;
  float energy;
  float motor_voltage;
  float phase_signal;
  float reference_angle_rad;
  float reference_angle_rate_rad_s;
  float reference_position_m;
  float reference_position_ticks;
  float reference_velocity_m_s;
  float reference_velocity_ticks_s;
  float position_ratio;

  if (controller == 0) {
    return 0.0f;
  }

  reference_position_ticks = CartpoleCoordinates_ToReferencePosition(
                               cart_position_ticks);
  reference_velocity_ticks_s = CartpoleCoordinates_ToReferenceVelocity(
                                 cart_velocity_ticks_s);
  reference_angle_rad = -CartpoleMath_ThetaErrorFromUpright(angle_rad);
  reference_angle_rate_rad_s = -angle_rate_rad_s;
  reference_position_m = reference_position_ticks
                       * CARTPOLE_ENCODER_METERS_PER_TICK;
  reference_velocity_m_s = reference_velocity_ticks_s
                         * CARTPOLE_ENCODER_METERS_PER_TICK;

  energy = 0.0032f * reference_angle_rate_rad_s * reference_angle_rate_rad_s
         + 0.1764f * (cosf(reference_angle_rad) - 1.0f);

  if (controller->return_direction == 0) {
    if (reference_position_ticks
        > (float)CARTPOLE_SWINGUP_WINDOW_HALF_TICKS) {
      controller->return_direction = -1;
    } else if (reference_position_ticks
               < -(float)CARTPOLE_SWINGUP_WINDOW_HALF_TICKS) {
      controller->return_direction = 1;
    }
  } else if ((controller->return_direction < 0)
             && (reference_position_ticks
                 <= (float)CARTPOLE_SWINGUP_RETURN_EXIT_HALF_TICKS)) {
    controller->return_direction = 0;
  } else if ((controller->return_direction > 0)
             && (reference_position_ticks
                 >= -(float)CARTPOLE_SWINGUP_RETURN_EXIT_HALF_TICKS)) {
    controller->return_direction = 0;
  }

  if (controller->return_direction != 0) {
    return ComputeCartReturn(controller, reference_position_ticks,
                             reference_velocity_ticks_s);
  }

  if (energy >= 0.0f) {
    return 0.0f;
  }

  /* Vendor relay: accel = 98 * Sign(E * angle_speed * cos(angle)).
   * The dead band only keeps ADC derivative noise from toggling the sign. */
  phase_signal = reference_angle_rate_rad_s * cosf(reference_angle_rad);
  if (fabsf(phase_signal) > CARTPOLE_SWINGUP_PHASE_HYST_RAD_S) {
    controller->pump_direction = (int8_t)SignFloat(energy * phase_signal);
  }

  acceleration = 98.0f * (float)controller->pump_direction;
  position_ratio = CartpoleMath_Clamp(
    fabsf(reference_position_m) / 0.15f, 0.0f, 0.95f);
  if (reference_position_m != 0.0f) {
    acceleration += SignFloat(reference_position_m)
                  * logf(1.0f - position_ratio);
  }

  motor_voltage = 0.0651f * acceleration
                + 11.5556f * reference_velocity_m_s
                + (0.0028f * cosf(reference_angle_rad)
                   * (0.1764f * sinf(reference_angle_rad)
                      - 0.0180f * acceleration)) / 0.0012f
                - 0.8440f * sinf(reference_angle_rad)
                  * reference_angle_rate_rad_s * reference_angle_rate_rad_s;

  return CartpoleMath_Clamp(
    motor_voltage / CARTPOLE_SWINGUP_SUPPLY_VOLTAGE,
    -CARTPOLE_SWINGUP_MAX_U, CARTPOLE_SWINGUP_MAX_U);
}
