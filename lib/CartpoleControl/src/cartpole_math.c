#include "cartpole_math.h"

#include "cartpole_config.h"

float CartpoleMath_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

float CartpoleMath_WrapPi(float angle_rad)
{
  while (angle_rad > CARTPOLE_PI_F) {
    angle_rad -= 2.0f * CARTPOLE_PI_F;
  }
  while (angle_rad < -CARTPOLE_PI_F) {
    angle_rad += 2.0f * CARTPOLE_PI_F;
  }
  return angle_rad;
}

float CartpoleMath_ThetaErrorFromUpright(float angle_rad)
{
  return CartpoleMath_WrapPi(angle_rad - CARTPOLE_UPRIGHT_ANGLE_RAD);
}
