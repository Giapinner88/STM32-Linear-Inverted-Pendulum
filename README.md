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
├── src/                        # STM32Cube generated application sources
│   ├── main.c                  # entry point, hardware init
│   ├── adc.c                   # angle sensor (WDD35D4 potentiometer)
│   ├── tim.c                   # PWM output + 5 ms timer interrupt
│   ├── usart.c                 # UART debug output
│   ├── gpio.c                  # buttons, LEDs
│   ├── i2c.c                   # OLED communication
│   └── stm32f1xx_it.c          # interrupt handlers
├── include/                    # STM32Cube generated headers (main.h, tim.h, ...)
├── lib/
│   ├── OLED/src/               # oled.c, oled.h, oledfont.h
│   ├── delay/src/              # delay.c, delay.h
│   └── show/src/               # show.c, show.h, control.c
└── project/                    # CubeMX/CubeIDE reference files (.ioc, linker, Drivers)
```

The 5 ms control interrupt fires in `stm32f1xx_it.c`. All sensor reads and motor output happen there. Reading the angle and position, and writing the PWM, requires only standard HAL calls:

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
4. Slowly twist the encoder knob until the ADC value on the OLED reads between 1010–1030 (target: 1024)
5. Retighten the screw

The OLED screen shows live sensor readings during this process — ADC (angle), P (position), VOL (battery voltage).

---

## Getting started

**Requirements:**
- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) or VS Code + PlatformIO IDE extension
- ST-Link v2 USB programmer (or serial uploader if configured)

**Steps (PlatformIO):**
1. Clone this repo
2. Build firmware: `pio run`
3. Upload firmware: `pio run -t upload`
4. Edit your control algorithm in `lib/show/src/control.c`

**Optional (CubeMX/CubeIDE reference):**
- Original CubeMX metadata and linker script are kept in `project/` for pin/peripheral regeneration and reference.

---

## Algorithms planned / in progress

- [ ] PD balance + position control (baseline)
- [ ] LQR full-state feedback
- [ ] Lyapunov-stable neural control (main research goal)

---

## Acknowledgements

Hardware documentation and reverse-engineering notes from [playfultechnology/pid-invertedpendulum](https://github.com/playfultechnology/pid-invertedpendulum) were essential in understanding how this board actually works — particularly the ADC-based angle sensing, the calibration procedure, and the 5 ms interrupt-driven control loop structure.

Original firmware by Wheeltec / MiniBalance. HAL port and project restructuring by this repository's author.