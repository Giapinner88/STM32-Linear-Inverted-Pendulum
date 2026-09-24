# Phase 1: Motor Travel Range Characterization

## Test Results

**Date**: March 26, 2026  
**Platform**: STM32F103C8T6 @ 72MHz  
**Motor**: DC Motor with Quadrature Encoder (TIM4 encoder mode)  
**PWM Duty**: 50% (3500/7199)

### Measurements

| Parameter | Value | Unit |
|-----------|-------|------|
| **LEFT WALL Position** | 4040 | encoder pulses |
| **RIGHT WALL Position** | 0 | encoder pulses |
| **Usable Travel Range** | 4040 | encoder pulses |

### Test Procedure

The encoder is treated as a **relative position from the right-wall home reference**.
In the current setup, the reserved button is used to command a return to the fixed right-wall home before closed-loop testing.

### Homing Procedure

1. Drive the cart to the right wall.
2. From away from the right wall, hold the reserved PA2 button to command the cart back to the fixed right-wall home.
3. Verify on OLED that the state shows homing while the cart is moving back.
4. The cart will stop automatically when it reaches the home threshold.

1. **LEFT Phase**:
   - Motor runs at 50% PWM toward left wall
   - Collision detection: encoder stable for 3 readings (~900ms) = stops
   - Captures: 4040 pulses

2. **RIGHT Phase**:
   - Motor continues from left position, reverses direction
   - Runs toward right wall at 50% PWM
   - Collision detection: encoder stable for 3 readings = stops
   - Captures: 0 pulses

### Key Findings

- Motor spans **4040 encoder pulses** from right wall to left wall in the current convention
- Encoder should be treated as relative-per-home data, with right wall as the home reference in this measurement setup
- Collision detection via stability counting works reliably
- This characterization predates the hybrid controller; direction signs must be verified again after controller changes.

### Active controller direction convention

- Positive command follows the manufacturer driver and moves the cart right while encoder ticks decrease.
- Negative command moves the cart left while encoder ticks increase.
- Controllers use `x_ref = -encoder_ticks`, matching the source firmware's `lastEncoder - Encoder` velocity definition.
- Before a full swing-up test, verify `+` jog moves right and `-` jog moves left.

### Next Steps

1. **Calibration**: Measure wall-to-wall distance (meters) to get conversion factor
   - **Conversion**: [distance in meters] / 4040 pulses = meters/pulse

2. **Control Workflow**: Put the cart at the physical centre, hold the pendulum down at ADC 990–1060 (target 1024), then press USER (PA5) once. Do not press M1 first. Expect `CAL -> SWG -> BAL`; press USER again to stop.

3. **Dead-zone Testing**: Determine minimum PWM needed for motor to start moving

4. **LQR Controller Design**: Use travel range for state boundaries and control limits

---

## Hardware Configuration

- **Motor Control**: TIM3 Channel 4 (PWM), GPIO PB13/PB12 (direction)
- **Encoder Input**: TIM4 in encoder mode, GPIO PB6/PB7
- **Communication**: USART1 @ 128000 baud
- **Display**: SSD1306 OLED 128x64
