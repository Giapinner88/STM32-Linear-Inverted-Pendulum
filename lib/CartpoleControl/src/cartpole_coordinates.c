#include "cartpole_coordinates.h"

float CartpoleCoordinates_ToReferencePosition(float encoder_ticks)
{
  return -encoder_ticks;
}

float CartpoleCoordinates_ToReferenceVelocity(float encoder_ticks_per_second)
{
  return -encoder_ticks_per_second;
}
