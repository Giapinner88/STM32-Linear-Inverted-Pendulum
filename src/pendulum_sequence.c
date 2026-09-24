#include "pendulum_sequence.h"

#include "cartpole_config.h"

void PendulumSequence_Init(PendulumSequence *sequence)
{
  if (sequence == 0) {
    return;
  }
  sequence->state = PENDULUM_SEQUENCE_IDLE;
  sequence->valid_samples = 0U;
  sequence->consecutive_rejects = 0U;
  sequence->elapsed_samples = 0U;
  sequence->adc_sum = 0U;
  sequence->angle_zero_adc = CARTPOLE_ANGLE_ZERO_DEFAULT_ADC;
  sequence->control_center_encoder = 0;
}

void PendulumSequence_StartCalibration(PendulumSequence *sequence)
{
  if (sequence == 0) {
    return;
  }
  sequence->state = PENDULUM_SEQUENCE_CALIBRATING;
  sequence->valid_samples = 0U;
  sequence->consecutive_rejects = 0U;
  sequence->elapsed_samples = 0U;
  sequence->adc_sum = 0U;
}

void PendulumSequence_Stop(PendulumSequence *sequence)
{
  if (sequence == 0) {
    return;
  }
  sequence->state = PENDULUM_SEQUENCE_IDLE;
  sequence->valid_samples = 0U;
  sequence->consecutive_rejects = 0U;
  sequence->elapsed_samples = 0U;
  sequence->adc_sum = 0U;
}

PendulumSequenceEvent PendulumSequence_Update(PendulumSequence *sequence,
                                              uint16_t angle_adc,
                                              int32_t encoder_position)
{
  if ((sequence == 0)
      || (sequence->state != PENDULUM_SEQUENCE_CALIBRATING)) {
    return PENDULUM_SEQUENCE_EVENT_NONE;
  }

  if (sequence->elapsed_samples < 0xFFFFU) {
    sequence->elapsed_samples++;
  }

  if ((angle_adc >= CARTPOLE_POT_ZERO_MIN_ADC)
      && (angle_adc <= CARTPOLE_POT_ZERO_MAX_ADC)) {
    sequence->adc_sum += (uint32_t)angle_adc;
    if (sequence->valid_samples < 0xFFFFU) {
      sequence->valid_samples++;
    }
    sequence->consecutive_rejects = 0U;
  } else {
    if (sequence->consecutive_rejects < 0xFFFFU) {
      sequence->consecutive_rejects++;
    }
    if (sequence->consecutive_rejects
        >= CARTPOLE_AUTO_CAL_MAX_CONSECUTIVE_REJECTS) {
      sequence->adc_sum = 0U;
      sequence->valid_samples = 0U;
    }
  }

  if (sequence->valid_samples >= CARTPOLE_AUTO_CAL_REQUIRED_SAMPLES) {
    sequence->angle_zero_adc = (float)sequence->adc_sum
                             / (float)sequence->valid_samples;
    sequence->control_center_encoder = encoder_position;
    sequence->state = PENDULUM_SEQUENCE_RUNNING;
    return PENDULUM_SEQUENCE_EVENT_CALIBRATED;
  }

  if (sequence->elapsed_samples >= CARTPOLE_AUTO_CAL_TIMEOUT_SAMPLES) {
    sequence->state = PENDULUM_SEQUENCE_CAL_ERROR;
    return PENDULUM_SEQUENCE_EVENT_CAL_TIMEOUT;
  }

  return PENDULUM_SEQUENCE_EVENT_NONE;
}
