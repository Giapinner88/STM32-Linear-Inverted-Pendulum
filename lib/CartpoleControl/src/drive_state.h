#ifndef DRIVE_STATE_H
#define DRIVE_STATE_H

#include <stdint.h>

typedef struct {
  float previous_command;
  uint8_t upright_locked;
  uint8_t capture_hold_count;
  uint8_t capture_lost_count;
} DriveState;

void DriveState_Init(DriveState *controller);
void DriveState_Reset(DriveState *controller);
void DriveState_UpdateCapture(DriveState *controller,
                                    float angle_rad,
                                    float angle_rate_rad_s);
uint8_t DriveState_IsCaptured(const DriveState *controller);
float DriveState_ApplySlew(DriveState *controller,
                                 float target_command);

#endif /* DRIVE_STATE_H */

