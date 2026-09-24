# Pin assignment — STM32F103C8 / IP570 Wheeltec

> Source: current `project/InvertedPendulum.ioc`, HAL setup, and supplied Wheeltec reference firmware
> MCU: STM32F103C8Tx · LQFP48 · 72 MHz (HSE × PLL × 9)

---

## Angle sensor — WDD35D4-5K rotary potentiometer (ADC)

| Pin | Signal | Peripheral | Notes |
|-----|--------|------------|-------|
| PA3 | `ADC1_IN3` | ADC1 channel 3 | Pendulum angle — 12-bit, 0–4095. ~1024 = vertical down |
| PA6 | `ADC1_IN6` | ADC1 channel 6 | Battery voltage monitor |

---

## Linear encoder — cart position (quadrature)

| Pin | Signal | Peripheral | Notes |
|-----|--------|------------|-------|
| PB6 | `ENC_A` | TIM4_CH1 | Encoder channel A — quadrature mode TI12 |
| PB7 | `ENC_B` | TIM4_CH2 | Encoder channel B — IC filter = 10 |

The active application accumulates encoder motion relative to startup. Its
configured right-home/left-wall convention is 0 to 4130 ticks.

---

## Motor driver — H-bridge (PWM + direction)

| Pin | Label | Notes |
|-----|-------|-------|
| PB1  | `PWMB` | TIM3_CH4 · 10 kHz PWM · period = 7199. Range ±7199 |
| PB12 | `BIN2` | H-bridge direction bit 2 (GPIO output) |
| PB13 | `BIN1` | H-bridge direction bit 1 (GPIO output) |

The manufacturer convention is retained: `u > 0` drives right and `u < 0`
drives left. Raw encoder ticks have the opposite sign: they increase toward the
left. Control modules convert raw position/velocity with `x_ref = -ticks`.

---

## Buttons — external interrupts

All buttons are active-low with internal pull-up. The active firmware detects
press edges by polling and latching them in the nominal 20 ms application loop.

| Pin | Label | EXTI | Physical location | Function |
|-----|-------|------|-------------------|----------|
| PA5  | `User_key`   | EXTI5  (EXTI9_5_IRQn)   | Physical **USER** | One click: start automatic calibration/swing-up/balance; next click: stop |
| PA7  | `menu_key`   | EXTI7  (EXTI9_5_IRQn)   | **M1/menu** | Set angle zero while the automatic sequence is idle |
| PA11 | `pid_plus`   | EXTI11 (EXTI15_10_IRQn) | Back of board, labelled **+**   | Positive manual jog while idle |
| PA12 | `pid_reduce` | EXTI12 (EXTI15_10_IRQn) | Back of board, labelled **−**   | Negative manual jog while idle |
| PA2  | `reserved_key` | — (GPIO input, pull-up) | Auxiliary key | Short press: toggle manual RUN/CAL inhibit; long press: home to the fixed right-wall reference |

> Note: Current firmware polls button states in the control loop; EXTI is configured for compatibility.

The `+` and `-` keys currently provide low-authority manual jog while idle; they
do not edit PID gains. Automatic operation requires only USER on PA5.

---

## LED

| Pin | Label | PCB label | Notes |
|-----|-------|-----------|-------|
| PA4 | `LED` | **L1** — user LED | GPIO output, pull-up, active-low (`LED=0` = on) |
| — | — | **L2** — power LED | Hardwired to VCC via resistor. Always on when powered. No GPIO |

L1 blinks four times during boot, then toggles every ten application frames as
a liveness indicator. It is not a controller-mode indicator.

---

## OLED display — 128×64 (dual interface)

The OLED is wired to both a software bit-bang SPI and I2C1 (hardware). Check `oled.c` for which interface is active.

| Pin | Label | Notes |
|-----|-------|-------|
| PA15 | `OLED_DC`  | Data/Command select (GPIO output) |
| PB3  | `OLED_RSE` | Reset (GPIO output) |
| PB4  | `OLED_SDA` | Bit-bang SDA (GPIO output) |
| PB5  | `OLED_SCL` | Bit-bang SCL (GPIO output) |
| PB8  | `I2C1_SCL` | I2C1 fast mode (hardware) |
| PB9  | `I2C1_SDA` | I2C1 fast mode (hardware) |

---

## UART — debug / DataScope

| Pin | Signal | Config |
|-----|--------|--------|
| PA9  | `USART1_TX` | 128000 baud, async |
| PA10 | `USART1_RX` | 128000 baud, async |

---

## SWD — ST-Link programmer

| Pin | Signal |
|-----|--------|
| PA13 | `SWDIO` |
| PA14 | `SWCLK` |

---

## Crystal — HSE

| Pin | Signal |
|-----|--------|
| PD0 | `OSC_IN`  |
| PD1 | `OSC_OUT` |

8 MHz crystal → PLL × 9 → **72 MHz** system clock.

---

## Timers summary

| Timer | Mode | Period | Use |
|-------|------|--------|-----|
| TIM1  | Up interrupt | prescaler 7199, period 49 | Initialized by CubeMX; no active controller callback |
| TIM3  | PWM CH4 | period 7199 (10 kHz) | Motor PWM output on PB1 |
| TIM4  | Encoder TI12 | — | Linear encoder on PB6/PB7 |
| SysTick | — | — | HAL timebase (1 ms) |

---

## NVIC interrupt priorities (priority group 2)

| Interrupt | Preempt | Sub | Source |
|-----------|---------|-----|--------|
| TIM1_UP   | 1 | 3 | Reserved/unused by current control path |
| EXTI9_5   | 2 | 2 | PA5 (USER), PA7 (M1/menu) |
| EXTI15_10 | 2 | 2 | PA11 (pid_plus), PA12 (pid_reduce) |
| SysTick   | 0 | 0 | HAL timebase |

---

## Clock tree

```
HSE (8 MHz crystal)
  └── PLL × 9 = 72 MHz (SYSCLK)
        ├── AHB  = 72 MHz (HCLK)
        ├── APB2 = 72 MHz  → TIM1, ADC1, USART1, GPIO
        ├── APB1 = 36 MHz  → TIM3, TIM4, I2C1
        └── ADC  = 12 MHz  (APB2 / 6)
```

---

## Hardware specifications

### System overview

| Parameter | Value |
|-----------|-------|
| Product name | Wheeltec IP570 linear inverted pendulum |
| MCU | STM32F103C8T6 — ARM Cortex-M3, 72 MHz, 64 KB flash, 20 KB SRAM |
| Supply voltage | 12 V DC (battery pack) |
| Control cycle | Nominal 20 ms cooperative main-loop frame; measured `dt` from HAL tick; 10-sample ADC average |
| Motor driver | TB6612FNG (integrated on board), VM max 15 V, output avg 1.2 A, peak 3.2 A |
| Communication | USART1 at 128000 baud — program download via MicroUSB, DataScope waveform display |
| Display | 0.96" OLED 128×64 (I2C / bit-bang SPI) |

---

### Motor — MG513P20 12V

DC brushed gear motor with Hall encoder.

| Parameter | Value |
|-----------|-------|
| Model | MG513P20 (P = plastic gearbox, 20 = reduction ratio 1:20) |
| Rated voltage | 12 V |
| No-load speed (output shaft) | ~550 RPM @ 12 V |
| Reduction ratio | 1 : 20 |
| Hall encoder lines | 13 lines per motor revolution |
| Quadrature counts per output shaft revolution | 13 × 4 × 20 = **1040 counts/rev** (4× counting, TIM4 encoder mode TI12) |
| Encoder supply voltage | 5 V |
| Output shaft diameter | 6 mm, D-type flat |
| Gearbox length | 23 mm |
| Motor diameter | 37 mm |

> Source: comment in `control.c` — *"产品使用的是13线霍尔传感器电机，20减速比，程序上用的是4倍频，电机转一圈编码器上的数值是13×4×20 = 1040"*

---

### Angle sensor — WDD35D4-5K

Single-turn conductive plastic rotary potentiometer, used as angular displacement sensor for the pendulum rod.

| Parameter | Value |
|-----------|-------|
| Model | WDD35D4-5K |
| Resistance | 5 kΩ |
| Resistance tolerance | ±15% |
| Electrical rotation range | 345° ± 2° |
| Mechanical rotation | 360° continuous |
| Linearity (independent) | 0.1% ~ 1% (model-dependent) |
| Rated power | 2 W @ 70°C |
| Temperature coefficient | 400 ppm/°C |
| Bearing | Dual ball bearing (stainless steel shaft, aluminium alloy housing) |
| Mechanical life | 50,000,000 revolutions |
| Operating temperature | −55°C to +125°C |
| ADC reading at vertical-down | ~1024 (12-bit, 0–4095 full range) |
| ADC reading at vertical-up (balance point) | ~3100 (`ZHONGZHI = 3100` in firmware) |
| Automatic calibration acceptance | 990–1060 (vertical down; target 1024) |

---

### Linear encoder (cart position)

The cart position is measured by the motor's own Hall encoder read via TIM4 in quadrature mode.

| Parameter | Value | Source |
|-----------|-------|--------|
| Encoder type | Hall effect, 2-channel quadrature | hardware |
| Counts at left end relative to current right home | 4130 | `cartpole_config.h` |
| Counts at right home | 0 | `cartpole_config.h` |
| Nominal center | 2065 ticks from right home | Derived from configured travel |
| Rail safety margin | 120 ticks | `cartpole_config.h` |
| One full motor revolution | 1040 counts | Supplied reference firmware |

---

### Legacy PID parameters from the supplied reference firmware

These values are retained only for provenance. They are not compiled into the
current energy-shaping/LQR control path.

| Parameter | Default value | Physical meaning |
|-----------|--------------|-----------------|
| `Balance_KP` | 400 | Angle proportional gain |
| `Balance_KD` | 400 | Angle derivative gain |
| `Position_KP` | 20 | Position proportional gain |
| `Position_KD` | 300 | Position derivative gain |
| PWM limit | ±6900 / 7199 | ~96% duty cycle max |
| Low-voltage cutoff | 700 (ADC units) | ~8.4 V |
| Angle protection range | ZHONGZHI ± 500 = 2600–3600 | ~±87° from upright |

---

### Pendulum rod — physical parameters

Uniform solid cylindrical rod, stainless steel (inox 304/316).

| Parameter | Symbol | Value | Notes |
|-----------|--------|-------|-------|
| Length | L | 390 mm | Measured from pivot axis to tip |
| Diameter | d | 6 mm | Circular cross-section |
| Material | — | Stainless steel (inox) | ρ ≈ 7900 kg/m³ |
| Volume | V | 11.03 cm³ | π·r²·L |
| Mass | m | **87.1 g** | ρ·V (note: 800g is the full assembly including encoder/mount) |
| Center of mass | l_c | 195 mm | L/2 from pivot (uniform rod assumption) |

#### Rotational inertia

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Moment of inertia about pivot (end) | I | 4.417 × 10⁻³ kg·m² |
| Moment of inertia about center of mass | I_cm | 1.104 × 10⁻³ kg·m² |
| Gravitational torque term | m·g·l_c | 0.1666 N·m |

#### Linearized dynamics (unstable equilibrium — upright)

Eigenvalue of the uncontrolled pendulum: `λ = ±√(m·g·l_c / I)`

| Parameter | Value |
|-----------|-------|
| ω² = m·g·l_c / I | 37.73 rad²/s² |
| Unstable pole | ±6.14 rad/s |
| Natural frequency | 0.978 Hz |
| Pendulum period | 1.02 s |

#### State-space model parameters (for LQR / MPC)

```
m  = 0.08711  kg     (rod mass)
L  = 0.390    m      (rod length)
l  = 0.1950   m      (distance pivot → CoM)
I  = 0.004417 kg·m²  (inertia about pivot)
g  = 9.81     m/s²
```

> ⚠️ These values assume a uniform solid rod. In practice, the pivot-end cap, encoder coupling, and end tip add mass asymmetrically — l_c and I should be verified experimentally if high-fidelity modelling is needed (hang test or frequency response identification).
