# DEBUG LOG - STM32 Linear Inverted Pendulum Project

## 26/03/2026 - Initial Setup & Testing

### 🔴 Issue 1: USB/ST-Link Access Permission Error
**Time:** Initial upload attempt  
**Error:** `libusb_open() failed with LIBUSB_ERROR_ACCESS`  
**Root Cause:** User doesn't have permission to access ST-Link V2 USB device on Linux

**Solution:**
```bash
# Created udev rule for ST-Link V2
sudo nano /etc/udev/rules.d/49-stlinkv2.rules

# Added rules:
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666"  # ST-Link V2
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666"  # ST-Link V2.1
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374d", MODE="0666"  # ST-Link V3

# Reload udev rules
sudo udevadm control --reload
sudo udevadm trigger
```

**Status:** ✅ FIXED - Upload now works without sudo

---

### 🔴 Issue 2: LED PA4 - Always ON / No Blinking
**Time:** After first successful upload  
**Observation:** LED stayed constantly lit, not blinking

**Root Cause:** `delay_init(72)` was never called, so `delay_ms()` wasn't working. LED was toggling extremely fast (microseconds) making it appear permanently ON.

**Solution:**
```c
int main(void)
{
  HAL_Init();

  /* USER CODE BEGIN Init */
  delay_init(72);  // <<< ADDED THIS - Initialize delay with 72MHz system clock
  /* USER CODE END Init */
  
  SystemClock_Config();
  // ... rest of init
}
```

**Status:** ✅ FIXED - LED now blinks at 500ms interval

---

### 🔴 Issue 3: Motor Not Responding
**Time:** When motor test was added to main.c  
**Observation:** Motor showed no movement despite GPIO and PWM signals

**Root Cause:** `HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4)` wasn't called before using `__HAL_TIM_SET_COMPARE()` macro

**Solution:**
```c
/* After MX_TIM3_Init() */
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  // <<< START PWM before using it

while(1)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);  // Now works
}
```

**Status:** ✅ FIXED - Motor runs forward/reverse pattern

---

### 🔴 Issue 4: OLED Display - No Display Output
**Time:** After adding OLED to combined LED+Motor+OLED test  
**Observation:** OLED remained completely dark/blank

**Root Cause:** Missing initialization delays. OLED needs time to:
1. Reset (RST pin)
2. Power up internal voltage regulators (DCDC)
3. Clear display buffer

**Solution:**
```c
/* After MX_GPIO_Init() */
delay_ms(200);  // GPIO stabilization

OLED_Init();    // Initialize OLED
delay_ms(1000); // Wait for OLED reset + DCDC startup
OLED_Clear();
delay_ms(100);  // Wait for clear operation
```

**Status:** ⚠️ PARTIALLY FIXED - OLED powers on but text still not visible

---

### 🔴 Issue 5: OLED Display - Blinking but No Text
**Time:** After adding initialization delays  
**Observation:** OLED blinks On/Off but no text displayed (no artifacts, just blank refresh)

**Root Cause:** `OLED_ShowString()` writes to internal RAM buffer (`OLED_GRAM[128][8]`) but **doesn't call `OLED_Refresh_Gram()`** to send buffer to hardware display

**Solution:**
```c
// WRONG - Text written to RAM but not sent to display:
OLED_Clear();
OLED_ShowString(0, 0, (uint8_t*)"Hello");
delay_ms(2000);

// CORRECT - Add Refresh after ShowString:
OLED_Clear();
OLED_ShowString(0, 0, (uint8_t*)"Hello");
OLED_Refresh_Gram();  // <<< SEND buffer to hardware
delay_ms(2000);
```

**Key Functions Requiring Refresh:**
- `OLED_ShowString()` - needs refresh
- `OLED_ShowChar()` - needs refresh  
- `OLED_ShowNumber()` - needs refresh
- `OLED_DrawPoint()` - needs refresh
- `OLED_Clear()` - **automatically calls refresh** (exception)

**Status:** ✅ FIXED - OLED now displays text and graphics correctly

---

## Summary of Test Results

| Component | Status | Initiative | Notes |
|-----------|--------|-----------|-------|
| **LED (PA4)** | ✅ Working | 500ms blink pattern | Requires `delay_init(72)` |
| **Motor (PB12/13)** | ✅ Working | Forward→Stop→Reverse | Requires `HAL_TIM_PWM_Start()` |
| **OLED (SSD1306, I2C)** | ✅ Working | Text + Graphics | Requires `OLED_Refresh_Gram()` after writes |

---

## Key Findings & Best Practices

### 1. Delay Function Initialization
- **ALWAYS** call `delay_init(SYSCLK)` early in `main()`
- For STM32F103C8 @ 72MHz: `delay_init(72)`
- Must happen BEFORE any `delay_ms()` or `delay_us()` calls

### 2. PWM Setup Order
1. Call `MX_TIMx_Init()` → Configures timer and channel
2. Call `HAL_TIM_PWM_Start()` → Enables PWM output
3. Use `__HAL_TIM_SET_COMPARE()` → Set duty cycle

### 3. OLED Display Pattern
```c
// Initialization sequence:
delay_ms(200);           // GPIO stable
OLED_Init();
delay_ms(1000);          // OLED startup
OLED_Clear();
delay_ms(100);

// Display update pattern:
OLED_Clear();            // Auto-refresh
[draw operations]
OLED_ShowString/ShowNumber/DrawPoint(...);
OLED_Refresh_Gram();     // Send to hardware (required!)
delay_ms(timing);
```

### 4. GPIO Bit-Band Access
OLED library uses bit-band macros (`PBout(n)`, `PAout(n)`) instead of HAL functions:
- Hardware: Directly toggles GPIO pins via bitband access
- Allows bit-level control without affecting other pins

---

## Files Modified

- ✅ `/home/giapinner88/Project/STM32-Linear-Inverted-Pendulum/src/main.c`
- ✅ `/home/giapinner88/Project/STM32-Linear-Inverted-Pendulum/lib/led_blink_test/src/`
- ✅ `/home/giapinner88/Project/STM32-Linear-Inverted-Pendulum/lib/oled_test/src/`
- ✅ `/etc/udev/rules.d/49-stlinkv2.rules` (system)

---

## Next Steps / Open Items

- [ ] Test full system with LQR controller
- [ ] Implement real-time display on OLED (sensor readings, motor commands)
- [ ] Optimize refresh rate vs. system responsiveness
- [ ] Test encoder feedback (TIM4 - currently in encoder mode, not used)

---

**Last Updated:** 26/03/2026 23:45 UTC  
**Status:** ✅ All basic hardware components tested and working

---

## 26/03/2026 - Phase 2 Bring-up + Swing-up Integration Notes

### 1) Firmware Architecture Pivot (From test scripts -> runtime control loop)
**Goal:** Move from one-off motor/OLED tests to a structure usable for swing-up.

**Implemented:**
- Added motor abstraction with dead-zone compensation (`Motor_SetTorque(float u)`).
- Added binary communication module (`Telemetry AA BB`, `Command CC DD`) and UART interrupt handlers.
- Refactored main loop toward periodic state update and control command execution.

**Build/Upload:** ✅ Successful via `~/.platformio/penv/bin/pio run -t upload`.

---

### 2) Serial Port Mismatch Investigation
**Symptom:** No telemetry bytes received despite successful flashing.

**Findings:**
- ST-Link present as `0483:3748`.
- `/dev/ttyACM0` identified as QinHeng (`1a86:55d4`, USB single serial), not ST-Link VCP.
- Raw read tests at 115200 and 128000 both returned 0 bytes.

**Conclusion:**
- Programming path is valid (ST-Link).
- UART runtime visibility depends on physical routing/wiring to PA9/PA10 serial path.

---

### 3) Potentiometer-only Validation Stage (Safety-first)
**Reason:** Prioritize verified angle sensing before full swing-up.

**Changes made during iterations:**
- Motor forced OFF.
- Added OLED live display for ADC, derived angle, derivative, and button status.
- Added boot heartbeat and LED blink patterns to verify code progress.

**Critical UI bug fixed:**
- OLED text coordinates were treated as line index instead of pixel Y.
- This caused overlapped text like `MOTOR OFFZERO` and misleading "freeze" perception.

**Outcome:**
- Potentiometer test confirmed working by user.

---

### 4) Swing-up First Integration and Observed Failure Mode
**User-observed behavior:**
- `R` toggled to run.
- `U%` stuck around +38.
- Cart repeatedly hit left wall and rebounded.

**Root causes identified/fixed in sequence:**
1. Encoder sign/width conversion mistake at one point (`int16_t` path around 16-bit counter wrap domain).
2. OLED long bottom string at Y=58 could trigger clear side effects in font drawing path.
3. Rail safety forcing logic could dominate control when encoder reference quality was poor.

---

### 5) Controller Smoothing Improvements
To reduce bang-bang behavior:
- Low-pass filter on `dtheta`.
- Slew-rate limit on control output `u`.
- Capture hysteresis/hold counter around upright region.
- Reduced/retuned force limits for safer experimental runs.

Result reported by user: swing behavior improved and pendulum started oscillatory motion rather than immediate hard rail forcing.

---

### 6) Zero Calibration Policy (User Requirement)
**Requested behavior:**
- Operator manually rotates pendulum to hanging-down reference near ADC 1010-1030.
- Press `M` should set zero only if ADC already in this valid window.
- Press `M` outside window must be rejected.

**Documented policy:**
- "Strict zero window" mode introduced conceptually and iterated in firmware updates.
- UX feedback added via OLED status (`OK/ER`) and LED acknowledgement patterns.

---

## Current Practical Lessons
- Validate sensing first (ADC + sign conventions) before trusting energy-based swing-up terms.
- Keep OLED strings short and Y positions explicit (12px rows) for this library.
- Avoid letting rail-protection logic become a permanent command source.
- When behavior seems "stuck", prioritize instrumentation (run flag, command %, capture flag, zero status).

## Suggested Next Step
- Freeze one "known-good" version of `main.c` with:
  - strict zero-set window,
  - smoothed swing-up,
  - capture hysteresis,
  - minimal rail override.
- Then tune only 2-3 gains (`kE`, `kp`, `kd`) with controlled runs.
