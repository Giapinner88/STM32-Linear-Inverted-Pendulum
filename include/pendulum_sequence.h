#ifndef PENDULUM_SEQUENCE_H
#define PENDULUM_SEQUENCE_H

#include <stdint.h>

typedef enum {
  PENDULUM_SEQUENCE_IDLE = 0,
  PENDULUM_SEQUENCE_CALIBRATING,
  PENDULUM_SEQUENCE_RUNNING,
  PENDULUM_SEQUENCE_CAL_ERROR
} PendulumSequenceState;

typedef enum {
  PENDULUM_SEQUENCE_EVENT_NONE = 0,
  PENDULUM_SEQUENCE_EVENT_CALIBRATED,
  PENDULUM_SEQUENCE_EVENT_CAL_TIMEOUT
} PendulumSequenceEvent;

typedef struct {
  PendulumSequenceState state;
  uint16_t valid_samples;
  uint16_t consecutive_rejects;
  uint16_t elapsed_samples;
  uint32_t adc_sum;
  float angle_zero_adc;
  int32_t control_center_encoder;
} PendulumSequence;

void PendulumSequence_Init(PendulumSequence *sequence);
void PendulumSequence_StartCalibration(PendulumSequence *sequence);
void PendulumSequence_Stop(PendulumSequence *sequence);
PendulumSequenceEvent PendulumSequence_Update(PendulumSequence *sequence,
                                              uint16_t angle_adc,
                                              int32_t encoder_position);

#endif /* PENDULUM_SEQUENCE_H */
