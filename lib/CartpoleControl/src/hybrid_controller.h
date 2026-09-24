#ifndef HYBRID_CONTROLLER_H
#define HYBRID_CONTROLLER_H

#include <stdint.h>
#include "swingup_controller.h"

typedef struct {
  float previous_command;
  SwingupController swingup;
  uint8_t upright_locked;
  uint8_t capture_hold_count;
  uint8_t capture_lost_count;
} HybridController;

void HybridController_Init(HybridController *controller);
void HybridController_Reset(HybridController *controller);
void HybridController_UpdateCapture(HybridController *controller,
                                    float angle_rad,
                                    float angle_rate_rad_s);
uint8_t HybridController_IsCaptured(const HybridController *controller);
float HybridController_Compute(HybridController *controller,
                               float cart_position_ticks,
                               float cart_velocity_ticks_s,
                               float angle_rad,
                               float angle_rate_rad_s);
float HybridController_ComputeSwingup(HybridController *controller,
                                      float cart_position_ticks,
                                      float cart_velocity_ticks_s,
                                      float angle_rad,
                                      float angle_rate_rad_s);
float HybridController_ApplySlew(HybridController *controller,
                                 float target_command);

#endif /* HYBRID_CONTROLLER_H */
