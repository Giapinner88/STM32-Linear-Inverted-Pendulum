#include "cartpole_safety.h"

#include "cartpole_config.h"

uint8_t CartpoleSafety_IsRailBlocked(int32_t encoder_position,
                                     float command,
                                     int32_t control_center,
                                     uint8_t control_center_valid)
{
  int32_t right_limit = CARTPOLE_ENCODER_RIGHT_WALL_TICKS;
  int32_t left_limit = CARTPOLE_ENCODER_LEFT_WALL_TICKS;

  if (control_center_valid != 0U) {
    right_limit = control_center - CARTPOLE_ENCODER_HALF_TRAVEL_TICKS;
    left_limit = control_center + CARTPOLE_ENCODER_HALF_TRAVEL_TICKS;
  }

  /* Raw encoder ticks increase toward the left, but positive motor command
   * follows the Wheeltec reference convention and moves toward the right. */
  if ((encoder_position >= (left_limit - CARTPOLE_RAIL_MARGIN_TICKS))
      && (command < 0.0f)) {
    return 1U;
  }
  if ((encoder_position <= (right_limit + CARTPOLE_RAIL_MARGIN_TICKS))
      && (command > 0.0f)) {
    return 1U;
  }
  return 0U;
}
