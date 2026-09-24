# Firmware architecture

The firmware is split so that board I/O, operating sequence, control laws, and
display code can be inspected independently.

## Runtime flow

```text
USER button
    -> PendulumSequence (IDLE -> CALIBRATING -> RUNNING)
    -> PendulumApp reads ADC and encoder
    -> HybridController
         |-- SwingupController while the pendulum is away from upright
         `-- LqrController after upright capture
    -> CartpoleSafety rail guard
    -> Motor_SetTorque

PendulumApp -> PendulumUi -> OLED
```

The physical USER button is active-low on `PA5`. M1/menu is on `PA7` and is not
required for the automatic sequence. The control frame runs in the cooperative
main loop on a fixed 5 ms deadline (the supplied firmware rate); TIM1 does not
dispatch the controller. Angle and cart rates are differenced over a 4-frame
(20 ms) window.
Each frame averages ten ADC conversions, matching the supplied firmware.
Calibration requires 500 ms of accepted averaged samples in the 990–1060 range, tolerates
short rejected bursts, and reports `ERR` after approximately five seconds.

The Wheeltec reference coordinate is positive toward the right, while raw TIM4
ticks increase toward the left. `cartpole_coordinates.c` performs this sign
conversion for both swing-up and LQR. Motor commands keep the manufacturer
driver convention: positive goes right and negative goes left. Rail safety uses
raw ticks plus that motor-command convention explicitly.

## Module map

| Path | Responsibility |
| --- | --- |
| `include/cartpole_config.h` | All controller gains, geometry, thresholds, timing, and limits |
| `src/pendulum_app.c` | Nominal 20 ms main-loop orchestration, sensor acquisition, homing, remote commands, and actuator dispatch |
| `src/pendulum_sequence.c` | One-button calibration/start/stop state machine |
| `src/pendulum_ui.c` | OLED startup and runtime views |
| `lib/CartpoleControl/src/swingup_controller.c` | Energy-shaping swing-up and cart return window |
| `lib/CartpoleControl/src/lqr_controller.c` | Upright LQR control law and reference coordinate conversion |
| `lib/CartpoleControl/src/hybrid_controller.c` | Swing-up/LQR capture hysteresis and output slew limiting |
| `lib/CartpoleControl/src/cartpole_safety.c` | Dynamic and absolute rail-limit interlock |
| `lib/CartpoleControl/src/cartpole_math.c` | Angle wrapping and scalar clamp helpers |
| `lib/CartpoleControl/src/cartpole_coordinates.c` | Raw encoder to Wheeltec reference-coordinate conversion |
| `lib/Hardware/src/motor.c` | PWM and H-bridge adapter |
| `lib/Protocol/src/comms.c` | UART packet adapter |
| `src/setup/` | STM32Cube-generated peripheral initialization and interrupts |

`PendulumSequence` and every file in `CartpoleControl` are independent of the
STM32 HAL. They can therefore be compiled and tested on the host using
`tests/cartpole_control_smoke.c`.

## Safe modification points

- Tune only constants in `include/cartpole_config.h` first.
- Change the swing-up law only in `swingup_controller.c`.
- Change the balancing law only in `lqr_controller.c`.
- Keep rail direction, motor polarity, and angle reference conversions explicit;
  do not bury them in new gains.
- Run both the host smoke test and `pio run` before flashing.
