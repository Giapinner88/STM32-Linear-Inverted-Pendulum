#ifndef CARTPOLE_SAFETY_H
#define CARTPOLE_SAFETY_H

#include <stdint.h>

uint8_t CartpoleSafety_IsRailBlocked(int32_t encoder_position,
                                     float command,
                                     int32_t control_center,
                                     uint8_t control_center_valid);

#endif /* CARTPOLE_SAFETY_H */
