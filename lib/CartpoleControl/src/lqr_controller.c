#include "lqr_controller.h"

#include "cartpole_config.h"
#include "cartpole_coordinates.h"
#include "cartpole_math.h"

float LqrController_Compute(float cart_position_ticks,
                            float cart_velocity_ticks_s,
                            float angle_rad,
                            float angle_rate_rad_s)
{
  float position_m;
  float velocity_m_s;
  float reference_angle_rad;
  float reference_angle_rate_rad_s;
  float motor_voltage;

  /* Reference firmware uses position and angle signs opposite to this app. */
  position_m = CartpoleCoordinates_ToReferencePosition(cart_position_ticks)
             * CARTPOLE_ENCODER_METERS_PER_TICK;
  velocity_m_s = CartpoleCoordinates_ToReferenceVelocity(
                   cart_velocity_ticks_s) * CARTPOLE_ENCODER_METERS_PER_TICK;
  reference_angle_rad = -CartpoleMath_ThetaErrorFromUpright(angle_rad);
  reference_angle_rate_rad_s = -angle_rate_rad_s;

  motor_voltage = -((CARTPOLE_LQR_K_X * position_m)
                  + (CARTPOLE_LQR_K_DX * velocity_m_s)
                  + (CARTPOLE_LQR_K_THETA * reference_angle_rad)
                  + (CARTPOLE_LQR_K_DTHETA * reference_angle_rate_rad_s));

  return CartpoleMath_Clamp(motor_voltage / CARTPOLE_LQR_SUPPLY_VOLTAGE,
                            -CARTPOLE_LQR_MAX_U,
                            CARTPOLE_LQR_MAX_U);
}
