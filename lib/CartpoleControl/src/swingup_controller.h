#ifndef SWINGUP_CONTROLLER_H
#define SWINGUP_CONTROLLER_H

#include <stdint.h>

typedef struct {
  int8_t pump_direction;
  int8_t return_direction;
} SwingupController;

void SwingupController_Init(SwingupController *controller);
void SwingupController_Reset(SwingupController *controller);
float SwingupController_Compute(SwingupController *controller,
                                float cart_position_ticks,
                                float cart_velocity_ticks_s,
                                float angle_rad,
                                float angle_rate_rad_s);

#endif /* SWINGUP_CONTROLLER_H */
