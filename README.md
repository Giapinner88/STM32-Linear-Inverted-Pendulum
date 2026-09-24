# STM32 Linear Inverted Pendulum — Custom Controller Platform

A hardware abstraction layer for the **Wheeltec IP570 linear inverted pendulum**, extracted and ported to STM32CubeIDE (HAL) as a clean foundation for implementing custom control algorithms.

The original manufacturer firmware (Wheeltec / MiniBalance) came bundled with a proprietary PID controller in a Keil-only project. This repository strips away the control logic entirely and replaces it with a buildable, open STM32CubeIDE project — leaving the control layer as a blank slate where any algorithm can be dropped in.

Inspired by [playfultechnology/pid-invertedpendulum](https://github.com/playfultechnology/pid-invertedpendulum), which went through a similar reverse-engineering process on the same hardware.

---

## Hardware

- **Platform:** Wheeltec IP570 linear inverted pendulum
- **MCU:** STM32F103C8 (ARM Cortex-M3, 72 MHz)
- **Angle sensor:** WDD35D4-5K rotary potentiometer → ADC (replaces IMU entirely)
- **Position sensor:** Linear encoder via TIM4 quadrature input
- **Actuator:** DC motor with H-bridge, driven by 10 kHz PWM
- **Display:** 128×64 OLED (I2C)
- **Programmer:** ST-Link v2

---

## What this repo contains

This repository is now organized in **PlatformIO standard layout**.

```
.
├── platformio.ini
├── src/
│   ├── main.c                  # entry point and hardware init
│   ├── pendulum_app.c          # real-time orchestration
│   ├── pendulum_sequence.c     # one-button calibration/start state machine
│   ├── pendulum_ui.c           # OLED presentation
│   └── setup/                  # Cube-generated peripheral setup and IRQs
├── include/
│   ├── cartpole_config.h       # gains, geometry, thresholds and timing
│   └── ...                     # application and Cube-generated headers
├── lib/
│   ├── CartpoleControl/src/    # swing-up, LQR, hybrid handoff and safety
│   ├── Hardware/src/           # motor output adapter
│   ├── Protocol/src/           # UART command/telemetry protocol
│   ├── OLED/src/               # oled.c, oled.h, oledfont.h
│   ├── delay/src/              # delay.c, delay.h
│   └── ...
├── tests/                      # host-side controller smoke tests
├── docs/firmware-architecture.md
├── method.md                   # PID, energy-shaping and LQR design record
└── project/                    # CubeMX/CubeIDE reference files (.ioc, linker, Drivers)
```

See [firmware architecture](docs/firmware-architecture.md) for the runtime flow
and the ownership of each module.

`PendulumApp_RunFrame()` runs from the main loop with a nominal 20 ms delay and
uses `HAL_GetTick()` to calculate the measured step time. Sensor reads, state
updates, safety checks, and motor output are executed there. TIM1 remains
initialized in the CubeMX project, but it does not drive the active controller.
Reading the angle and position, and writing the PWM, requires only standard HAL
calls:

```c
// Read pendulum angle (ADC, 12-bit, 0–4095)
HAL_ADC_Start(&hadc1);
HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
uint32_t angle_raw = HAL_ADC_GetValue(&hadc1);

// Read cart position (quadrature encoder, TIM4)
int32_t position = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

// Set motor PWM (TIM3 CH1/CH2, range ±7199)
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
```

---

## Sensor calibration

Before running any control algorithm, the angle sensor must be calibrated so that the pendulum hanging straight down reads a known ADC value (~1024).

1. Power on the board and let it sit idle
2. Hold the pendulum pointing straight down
3. Loosen the top screw connecting the pendulum rod to the encoder shaft
4. Slowly twist the encoder knob until the ADC value on the OLED is close to 1024. Automatic calibration accepts 990–1060.
5. Retighten the screw

The OLED screen shows live sensor readings during this process — ADC (angle), P (position), VOL (battery voltage).

### Runtime mode and homing

The firmware uses the reserved button on `PA2` for two functions:

- Short press: toggle between `RUN` and `CAL`
- Long press: start homing to the fixed right-wall reference

In `CAL`, the motor output is disabled while telemetry and OLED updates remain active. Use this mode for open-loop checks, sign verification, and sensor calibration.

### One-button automatic sequence

The USER button runs the local hybrid controller without a PC:

1. Put the cart at the physical centre of the rail and put the pendulum at the hanging-down reference (`ADC 990-1060`, target 1024).
2. From `IDL`, press the physical USER button once. Do not press M1 first. `CAL` keeps the motor off and accepts 25 averaged in-window samples while tolerating short noise bursts.
3. Calibration completion automatically changes the display to `SWG` and starts energy-shaping swing-up.
4. A valid upright capture automatically changes the display to `BAL` and hands control to the 40 cm LQR ported from `/home/giap/Downloads/whell fake`.
5. Press USER again at any time to stop the motor and return to idle. `ERR` means calibration timed out; correct the hanging reference and press USER to retry.

The LQR and encoder metre conversion are sourced from the reference firmware (`1040` counts per `0.036*pi` metres, with 12 V output scaling). The firmware builds successfully, but this controller has not been validated on the physical unit. Keep the rail clear and the power disconnect reachable during the first run.

The hardware swing-up preset uses a near-saturation `0.88` startup/energy-pump
command, matching the approximate `10.50 V` near-down result of the supplied
firmware. Its active cart window is limited to ±1700 encoder ticks to leave
braking margin.

The OLED `U%` field is the current applied command. `P%` is the peak absolute
applied command observed since the previous 80 ms display refresh, so short
near-saturation pulses remain visible.

Right-wall homing is a command to return to the fixed right-wall reference, not a free-form reference capture.

The cart position is homed from the right wall. If the cart is at the right wall and the travel span is the full usable range, the midpoint is simply half that span. With the current encoder limits, the midpoint is approximately halfway between the right-wall home and the left limit.

---

## Getting started

**Requirements:**
- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) or VS Code + PlatformIO IDE extension
- ST-Link v2 USB programmer (or serial uploader if configured)

**Steps (PlatformIO):**
1. Clone this repo
2. Build firmware: `pio run`
3. Upload firmware: `pio run -t upload`
4. Tune constants in `include/cartpole_config.h`; edit individual control laws
   under `lib/CartpoleControl/src/`

### Button map

- `User_key` on `PA5`: one-button calibration/start and immediate stop
- `menu_key` on `PA7`: set angle zero while idle
- `reserved_key` on `PA2`: short press toggles `RUN/CAL`, long press starts homing to the fixed right-wall reference
- `pid_plus` / `pid_reduce`: manual jog / torque trim during open-loop checks when enabled

### Phase 2 Serial Link Test (PC <-> STM32)

Use the binary protocol bridge script to validate telemetry + command packets:

```bash
python3 tools/serial_link.py --port /dev/ttyUSB0 --mode 0
```

Send a sine-wave command (test in `SWINGUP` mode):

```bash
python3 tools/serial_link.py --port /dev/ttyUSB0 --mode 1 --sine --amp 0.25 --freq 0.5
```

Install dependency once:

```bash
python3 -m pip install pyserial
```

**Optional (CubeMX/CubeIDE reference):**
- Original CubeMX metadata and linker script are kept in `project/` for pin/peripheral regeneration and reference.

---

## Algorithms planned / in progress

- [ ] PD balance + position control (baseline)
- [x] Experimental energy-shaping to LQR handoff (hardware validation pending)
- [ ] Lyapunov-stable neural control (main research goal)

---

## Acknowledgements

Hardware documentation and reverse-engineering notes from [playfultechnology/pid-invertedpendulum](https://github.com/playfultechnology/pid-invertedpendulum) were essential in understanding how this board actually works — particularly the ADC-based angle sensing and calibration procedure.

Original firmware by Wheeltec / MiniBalance. HAL port and project restructuring by this repository's author.
