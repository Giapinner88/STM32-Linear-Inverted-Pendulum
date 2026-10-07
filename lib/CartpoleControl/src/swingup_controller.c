#include "swingup_controller.h"

#include "cartpole_config.h"
#include "cartpole_coordinates.h"
#include "cartpole_math.h"

#include <math.h>

/* Wheeltec energy-shaping model, in the reference frame (x right, theta
 * from upright). Kept verbatim from the supplied firmware. */
#define SWINGUP_KINETIC_COEFF 0.0032f       /* E term on theta_dot^2 */
#define SWINGUP_POTENTIAL_COEFF 0.1764f     /* m*g*l */
#define SWINGUP_PUMP_ACCEL_M_S2 98.0f       /* relay acceleration */
#define SWINGUP_CENTER_RANGE_M 0.15f        /* log barrier half-width */
#define SWINGUP_CENTER_RATIO_MAX 0.95f
#define SWINGUP_VOLT_PER_ACCEL 0.0651f
#define SWINGUP_VOLT_PER_SPEED 11.5556f
#define SWINGUP_REACTION_GAIN 0.0028f
#define SWINGUP_PEND_ML 0.0180f
#define SWINGUP_PEND_INERTIA 0.0012f
#define SWINGUP_CENTRIPETAL_VOLT 0.8440f

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

/* PD pull toward the centre along return_direction only. A command that
 * would push outward is dropped (a cart moving inward fast may coast), and
 * an inward command is raised to MIN_U to overcome static friction. */
static float ComputeCartReturn(int8_t return_direction,
                               float position_ticks,
                               float velocity_ticks_s)
{
  float direction = (float)return_direction;
  float inward_command;

  inward_command = -direction * (CARTPOLE_SWINGUP_RETURN_K_X * position_ticks
                                 + CARTPOLE_SWINGUP_RETURN_K_V
                                   * velocity_ticks_s);
  inward_command = CartpoleMath_Clamp(inward_command, 0.0f,
                                      CARTPOLE_SWINGUP_RETURN_MAX_U);
  if ((inward_command > 0.0f)
      && (inward_command < CARTPOLE_SWINGUP_RETURN_MIN_U)) {
    inward_command = CARTPOLE_SWINGUP_RETURN_MIN_U;
  }
  return direction * inward_command;
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

  energy = SWINGUP_KINETIC_COEFF
           * reference_angle_rate_rad_s * reference_angle_rate_rad_s
         + SWINGUP_POTENTIAL_COEFF * (cosf(reference_angle_rad) - 1.0f);

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
    return ComputeCartReturn(controller->return_direction,
                             reference_position_ticks,
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

  acceleration = SWINGUP_PUMP_ACCEL_M_S2 * (float)controller->pump_direction;
  position_ratio = CartpoleMath_Clamp(
    fabsf(reference_position_m) / SWINGUP_CENTER_RANGE_M,
    0.0f, SWINGUP_CENTER_RATIO_MAX);
  if (reference_position_m != 0.0f) {
    acceleration += SignFloat(reference_position_m)
                  * logf(1.0f - position_ratio);
  }

  motor_voltage = SWINGUP_VOLT_PER_ACCEL * acceleration
                + SWINGUP_VOLT_PER_SPEED * reference_velocity_m_s
                + (SWINGUP_REACTION_GAIN * cosf(reference_angle_rad)
                   * (SWINGUP_POTENTIAL_COEFF * sinf(reference_angle_rad)
                      - SWINGUP_PEND_ML * acceleration))
                  / SWINGUP_PEND_INERTIA
                - SWINGUP_CENTRIPETAL_VOLT * sinf(reference_angle_rad)
                  * reference_angle_rate_rad_s * reference_angle_rate_rad_s;

  return CartpoleMath_Clamp(
    motor_voltage / CARTPOLE_SWINGUP_SUPPLY_VOLTAGE,
    -CARTPOLE_SWINGUP_MAX_U, CARTPOLE_SWINGUP_MAX_U);
}
