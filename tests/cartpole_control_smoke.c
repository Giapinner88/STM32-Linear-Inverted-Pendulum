#include "cartpole_config.h"
#include "cartpole_coordinates.h"
#include "cartpole_math.h"
#include "cartpole_safety.h"
#include "hybrid_controller.h"
#include "lqr_controller.h"
#include "pendulum_sequence.h"
#include "swingup_controller.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static int NearlyEqual(float a, float b, float tolerance)
{
  return fabsf(a - b) <= tolerance;
}

static void TestMathAndControllers(void)
{
  HybridController hybrid;
  SwingupController swingup;
  float command;
  unsigned int i;

  assert(NearlyEqual(fabsf(CartpoleMath_WrapPi(3.0f * CARTPOLE_PI_F)),
                     CARTPOLE_PI_F, 1.0e-5f));
  assert(NearlyEqual(LqrController_Compute(0.0f, 0.0f,
                                           CARTPOLE_UPRIGHT_ANGLE_RAD, 0.0f),
                     0.0f, 1.0e-6f));

  SwingupController_Init(&swingup);
  command = SwingupController_Compute(&swingup, 0.0f, 0.0f, 0.0f, 0.0f);
  assert(command > 0.87f);
  assert(command < 0.89f);
  command = SwingupController_Compute(&swingup, 0.0f, 0.0f, 0.0f, 0.20f);
  assert(command > 0.80f);

  SwingupController_Reset(&swingup);
  command = SwingupController_Compute(&swingup, 0.0f, 0.0f, 0.0f, 1.0f);
  assert(command < -0.80f);
  command = SwingupController_Compute(&swingup, 0.0f, 0.0f, 0.0f, 0.20f);
  assert(command < -0.80f);
  command = SwingupController_Compute(&swingup, 0.0f, 0.0f, 0.0f, 20.0f);
  assert(fabsf(command) <= CARTPOLE_SWINGUP_MAX_U);

  SwingupController_Reset(&swingup);
  command = SwingupController_Compute(&swingup, 2000.0f, 0.0f, 0.0f, 0.0f);
  assert(command >= CARTPOLE_SWINGUP_RETURN_MIN_U);
  assert(command <= CARTPOLE_SWINGUP_RETURN_MAX_U);
  assert(swingup.return_direction > 0);
  command = SwingupController_Compute(&swingup, 1600.0f, 0.0f, 0.0f, 0.0f);
  assert(command >= CARTPOLE_SWINGUP_RETURN_MIN_U);
  assert(swingup.return_direction > 0);
  (void)SwingupController_Compute(&swingup, 1000.0f, 0.0f, 0.0f, 0.0f);
  assert(swingup.return_direction == 0);

  SwingupController_Reset(&swingup);
  command = SwingupController_Compute(&swingup, -2000.0f, 0.0f, 0.0f, 0.0f);
  assert(command <= -CARTPOLE_SWINGUP_RETURN_MIN_U);
  assert(command >= -CARTPOLE_SWINGUP_RETURN_MAX_U);

  HybridController_Init(&hybrid);
  for (i = 0U; i < CARTPOLE_CAPTURE_ENTER_SAMPLES; ++i) {
    HybridController_UpdateCapture(&hybrid,
                                   CARTPOLE_UPRIGHT_ANGLE_RAD, 0.0f);
  }
  assert(HybridController_IsCaptured(&hybrid) != 0U);
  for (i = 0U; i < CARTPOLE_CAPTURE_EXIT_SAMPLES; ++i) {
    assert(HybridController_IsCaptured(&hybrid) != 0U);
    HybridController_UpdateCapture(&hybrid,
                                   CARTPOLE_UPRIGHT_ANGLE_RAD
                                     + CARTPOLE_CAPTURE_EXIT_ANGLE_RAD + 0.01f,
                                   0.0f);
  }
  assert(HybridController_IsCaptured(&hybrid) == 0U);

  HybridController_Reset(&hybrid);
  command = HybridController_ApplySlew(&hybrid, 1.0f);
  assert(NearlyEqual(command, CARTPOLE_SWINGUP_SLEW_PER_SAMPLE, 1.0e-6f));

  hybrid.upright_locked = 1U;
  hybrid.previous_command = 0.0f;
  command = HybridController_ApplySlew(&hybrid, 1.0f);
  assert(NearlyEqual(command, CARTPOLE_BALANCE_SLEW_PER_SAMPLE, 1.0e-6f));
}

static void TestRailSafety(void)
{
  assert(CartpoleCoordinates_ToReferencePosition(100.0f) == -100.0f);
  assert(CartpoleCoordinates_ToReferenceVelocity(-50.0f) == 50.0f);
  assert(CartpoleSafety_IsRailBlocked(4010, -0.2f, 0, 0U) != 0U);
  assert(CartpoleSafety_IsRailBlocked(4010, 0.2f, 0, 0U) == 0U);
  assert(CartpoleSafety_IsRailBlocked(120, 0.2f, 0, 0U) != 0U);
  assert(CartpoleSafety_IsRailBlocked(120, -0.2f, 0, 0U) == 0U);
  assert(CartpoleSafety_IsRailBlocked(2000, 0.2f, 0, 0U) == 0U);
  assert(CartpoleSafety_IsRailBlocked(4450, -0.2f, 2500, 1U) != 0U);
}

static void TestCalibrationSequence(void)
{
  PendulumSequence sequence;
  PendulumSequenceEvent event = PENDULUM_SEQUENCE_EVENT_NONE;
  unsigned int i;

  PendulumSequence_Init(&sequence);
  PendulumSequence_StartCalibration(&sequence);
  for (i = 0U; i < (CARTPOLE_AUTO_CAL_REQUIRED_SAMPLES / 2U); ++i) {
    event = PendulumSequence_Update(&sequence, 1024U, 2070);
  }
  event = PendulumSequence_Update(&sequence, 900U, 2070);
  assert(event == PENDULUM_SEQUENCE_EVENT_NONE);
  for (; i < CARTPOLE_AUTO_CAL_REQUIRED_SAMPLES; ++i) {
    event = PendulumSequence_Update(&sequence, 1024U, 2070);
  }
  assert(event == PENDULUM_SEQUENCE_EVENT_CALIBRATED);
  assert(sequence.state == PENDULUM_SEQUENCE_RUNNING);
  assert(NearlyEqual(sequence.angle_zero_adc, 1024.0f, 1.0e-6f));
  assert(sequence.control_center_encoder == 2070);

  PendulumSequence_StartCalibration(&sequence);
  for (i = 0U; i < CARTPOLE_AUTO_CAL_TIMEOUT_SAMPLES; ++i) {
    event = PendulumSequence_Update(&sequence, 900U, 0);
  }
  assert(event == PENDULUM_SEQUENCE_EVENT_CAL_TIMEOUT);
  assert(sequence.state == PENDULUM_SEQUENCE_CAL_ERROR);
}

int main(void)
{
  TestMathAndControllers();
  TestRailSafety();
  TestCalibrationSequence();
  puts("cartpole_control_smoke: PASS");
  return 0;
}
