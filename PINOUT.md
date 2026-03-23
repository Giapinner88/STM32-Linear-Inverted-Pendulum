# Pin assignment — STM32F103C8 / IP570 Wheeltec

> Source: `InvertedPendulum.ioc` (STM32CubeMX) + `exti.c` + `key.h` + `LED.H`  
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

Counter range: ~5850 (right end) to ~10000 (left end).

---

## Motor driver — H-bridge (PWM + direction)

| Pin | Label | Notes |
|-----|-------|-------|
| PB1  | `PWMB` | TIM3_CH4 · 10 kHz PWM · period = 7199. Range ±7199 |
| PB12 | `BIN2` | H-bridge direction bit 2 (GPIO output) |
| PB13 | `BIN1` | H-bridge direction bit 1 (GPIO output) |

---

## Buttons — external interrupts

All buttons are active-low with internal pull-up. Debounce: 5 ms delay in ISR.

| Pin | Label | EXTI | Physical location | Function |
|-----|-------|------|-------------------|----------|
| PA5  | `User_key`   | EXTI5  (EXTI9_5_IRQn)   | Top of board, labelled **USER** | Single click: start / stop balancing (`Flag_Stop` toggle). On start: triggers swing-up sequence |
| PA7  | `menu_key`   | EXTI7  (EXTI9_5_IRQn)   | Front of board, labelled **M**  | Single click: cycle selected PID parameter (B-KP → B-KD → P-KP → P-KD → repeat, shown as Y on OLED). Long press (~1 s): show help text |
| PA11 | `pid_plus`   | EXTI11 (EXTI15_10_IRQn) | Back of board, labelled **+**   | Increase selected PID parameter by its amplitude step |
| PA12 | `pid_reduce` | EXTI12 (EXTI15_10_IRQn) | Back of board, labelled **−**   | Decrease selected PID parameter by its amplitude step |
| PA2  | `reserved_key` | — (GPIO input, pull-up) | — | KEY2: single click moves cart one revolution forward/backward. Long press (~2 s): toggle auto swing-up mode (`auto_run`) |

> Note: PA2 (`KEY2`) is polled in the 5 ms control loop via `Key()`, not handled by EXTI.

### PID parameter selection (menu_key cycles through)

| Menu value | Parameter | Amplitude step variable |
|------------|-----------|------------------------|
| 1 | `Balance_KP` | `Amplitude1` (default 5) |
| 2 | `Balance_KD` | `Amplitude2` (default 20) |
| 3 | `Position_KP` | `Amplitude3` (default 1) |
| 4 | `Position_KD` | `Amplitude4` (default 10) |

---

## LED

| Pin | Label | PCB label | Notes |
|-----|-------|-----------|-------|
| PA4 | `LED` | **L1** — user LED | GPIO output, pull-up, active-low (`LED=0` = on) |
| — | — | **L2** — power LED | Hardwired to VCC via resistor. Always on when powered. No GPIO |

L1 behaviour from `Key()` in control loop:
- `auto_run=1` and `Flag_Stop=1`: L1 on solid (auto swing-up mode active)
- `auto_run=0` and `Flag_Stop=1`: L1 off (manual mode, stopped)
- `Flag_Stop=0` (balancing): L1 blinks via `Led_Flash(100)` every 500 ms

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
| TIM1  | Up interrupt | prescaler 7199, period 49 | **5 ms control loop** (TIM1_UP_IRQn, priority 1/3) |
| TIM3  | PWM CH4 | period 7199 (10 kHz) | Motor PWM output on PB1 |
| TIM4  | Encoder TI12 | — | Linear encoder on PB6/PB7 |
| SysTick | — | — | HAL timebase (1 ms) |

---

## NVIC interrupt priorities (priority group 2)

| Interrupt | Preempt | Sub | Source |
|-----------|---------|-----|--------|
| TIM1_UP   | 1 | 3 | 5 ms control loop |
| EXTI9_5   | 2 | 2 | PA5 (User_key), PA7 (menu_key) |
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