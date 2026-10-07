"""Cart-pole controller for running on the laptop.

A line-by-line port of lib/CartpoleControl (swingup_controller.c,
hybrid_controller.c capture logic, lqr_controller.c) plus the firmware's
command slew limiter. Keep it in step with the C constants in
include/cartpole_config.h; tools/check_control_port.py compares the two.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

PI_F = 3.1415926
UPRIGHT_OFFSET_RAD = 0.127
UPRIGHT_ANGLE_RAD = PI_F + UPRIGHT_OFFSET_RAD
METERS_PER_TICK = (0.036 * PI_F) / 1040.0

# Capture hysteresis. Sample counts are control frames of 5 ms.
CAPTURE_ENTER_ANGLE = 0.30
CAPTURE_EXIT_ANGLE = 0.45
CAPTURE_ENTER_RATE = 3.5
CAPTURE_EXIT_RATE = 7.0
CAPTURE_LEAD_S = 0.20
CAPTURE_ENTER_PREDICT = 0.25
CAPTURE_ENTER_SAMPLES = 2
CAPTURE_EXIT_SAMPLES = 4

# Swing-up (Wheeltec energy shaping) and cart return.
SWINGUP_MAX_U = 6900.0 / 7199.0
SWINGUP_PHASE_HYST = 0.45
SWINGUP_SUPPLY_V = 12.0
# Laptop tuning: the firmware uses 1700/1100. The board limits command changes
# to 0.1125 per frame, so reversing from +0.96 to -0.96 takes ~85 ms and the
# cart travels ~7 cm meanwhile; a 1700 tick window then ends in the end stop
# (hardware and simulation agree). check_control_port.py restores 1700/1100.
WINDOW_HALF_TICKS = 1000
RETURN_EXIT_HALF_TICKS = 600
FIRMWARE_WINDOW_HALF_TICKS = 1700
FIRMWARE_RETURN_EXIT_HALF_TICKS = 1100
RETURN_MAX_U = 6900.0 / 7199.0
RETURN_MIN_U = 0.80
RETURN_K_X = 0.00020
RETURN_K_V = 0.00180
KINETIC_COEFF = 0.0032
POTENTIAL_COEFF = 0.1764
PUMP_ACCEL = 98.0
CENTER_RANGE_M = 0.15
CENTER_RATIO_MAX = 0.95
VOLT_PER_ACCEL = 0.0651
VOLT_PER_SPEED = 11.5556
REACTION_GAIN = 0.0028
PEND_ML = 0.0180
PEND_INERTIA = 0.0012
CENTRIPETAL_VOLT = 0.8440

# 40 cm LQR from the supplied firmware.
LQR_K = (-36.4232, -40.0996, -101.4914, -15.8376)
LQR_SUPPLY_V = 12.0
LQR_MAX_U = 6900.0 / 7199.0

# Firmware slew limits, per 5 ms frame.
SWINGUP_SLEW = 0.1125
BALANCE_SLEW = 0.10


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def sign(value: float) -> float:
    return 1.0 if value > 0.0 else -1.0 if value < 0.0 else 0.0


def wrap_pi(angle: float) -> float:
    while angle > PI_F:
        angle -= 2.0 * PI_F
    while angle < -PI_F:
        angle += 2.0 * PI_F
    return angle


def theta_error(angle: float) -> float:
    return wrap_pi(angle - UPRIGHT_ANGLE_RAD)


def lqr_command(x_ticks: float, x_vel_ticks_s: float,
                angle: float, angle_rate: float) -> float:
    """x is relative to the control centre, raw encoder direction."""
    x = -x_ticks * METERS_PER_TICK
    x_dot = -x_vel_ticks_s * METERS_PER_TICK
    ref_angle = -theta_error(angle)
    ref_rate = -angle_rate
    volts = -(LQR_K[0] * x + LQR_K[1] * x_dot
              + LQR_K[2] * ref_angle + LQR_K[3] * ref_rate)
    return clamp(volts / LQR_SUPPLY_V, -LQR_MAX_U, LQR_MAX_U)


@dataclass
class Swingup:
    pump_direction: int = 1
    return_direction: int = 0

    def reset(self) -> None:
        self.pump_direction = 1
        self.return_direction = 0

    def _cart_return(self, position_ticks: float, velocity_ticks_s: float) -> float:
        direction = float(self.return_direction)
        inward = -direction * (RETURN_K_X * position_ticks
                               + RETURN_K_V * velocity_ticks_s)
        inward = clamp(inward, 0.0, RETURN_MAX_U)
        if 0.0 < inward < RETURN_MIN_U:
            inward = RETURN_MIN_U
        return direction * inward

    def compute(self, x_ticks: float, x_vel_ticks_s: float,
                angle: float, angle_rate: float) -> float:
        pos_ticks = -x_ticks
        vel_ticks_s = -x_vel_ticks_s
        ref_angle = -theta_error(angle)
        ref_rate = -angle_rate
        pos_m = pos_ticks * METERS_PER_TICK
        vel_m_s = vel_ticks_s * METERS_PER_TICK

        energy = (KINETIC_COEFF * ref_rate * ref_rate
                  + POTENTIAL_COEFF * (math.cos(ref_angle) - 1.0))

        if self.return_direction == 0:
            if pos_ticks > WINDOW_HALF_TICKS:
                self.return_direction = -1
            elif pos_ticks < -WINDOW_HALF_TICKS:
                self.return_direction = 1
        elif self.return_direction < 0 and pos_ticks <= RETURN_EXIT_HALF_TICKS:
            self.return_direction = 0
        elif self.return_direction > 0 and pos_ticks >= -RETURN_EXIT_HALF_TICKS:
            self.return_direction = 0

        if self.return_direction != 0:
            return self._cart_return(pos_ticks, vel_ticks_s)

        if energy >= 0.0:
            return 0.0

        # Relay accel = 98 * Sign(E * angle_speed * cos(angle)); the dead band
        # keeps derivative noise from toggling the sign.
        phase = ref_rate * math.cos(ref_angle)
        if abs(phase) > SWINGUP_PHASE_HYST:
            self.pump_direction = int(sign(energy * phase))

        accel = PUMP_ACCEL * self.pump_direction
        ratio = clamp(abs(pos_m) / CENTER_RANGE_M, 0.0, CENTER_RATIO_MAX)
        if pos_m != 0.0:
            accel += sign(pos_m) * math.log(1.0 - ratio)

        volts = (VOLT_PER_ACCEL * accel
                 + VOLT_PER_SPEED * vel_m_s
                 + (REACTION_GAIN * math.cos(ref_angle)
                    * (POTENTIAL_COEFF * math.sin(ref_angle) - PEND_ML * accel))
                 / PEND_INERTIA
                 - CENTRIPETAL_VOLT * math.sin(ref_angle) * ref_rate * ref_rate)
        return clamp(volts / SWINGUP_SUPPLY_V, -SWINGUP_MAX_U, SWINGUP_MAX_U)


@dataclass
class Capture:
    locked: bool = False
    hold_count: int = 0
    lost_count: int = 0

    def reset(self) -> None:
        self.locked = False
        self.hold_count = 0
        self.lost_count = 0

    def update(self, angle: float, angle_rate: float) -> None:
        err = theta_error(angle)
        err_abs = abs(err)
        rate_abs = abs(angle_rate)
        if not self.locked:
            if (err_abs < CAPTURE_ENTER_ANGLE and rate_abs < CAPTURE_ENTER_RATE
                    and abs(err + CAPTURE_LEAD_S * angle_rate) < CAPTURE_ENTER_PREDICT):
                self.hold_count = min(self.hold_count + 1, 255)
            else:
                self.hold_count = 0
            if self.hold_count >= CAPTURE_ENTER_SAMPLES:
                self.locked = True
                self.lost_count = 0
        else:
            if err_abs > CAPTURE_EXIT_ANGLE or rate_abs > CAPTURE_EXIT_RATE:
                self.lost_count = min(self.lost_count + 1, 255)
            else:
                self.lost_count = 0
            if self.lost_count >= CAPTURE_EXIT_SAMPLES:
                self.locked = False
                self.hold_count = 0


class Hybrid:
    """Swing-up until captured, LQR afterwards; mirrors HybridController."""

    def __init__(self) -> None:
        self.swingup = Swingup()
        self.capture = Capture()
        self.previous_command = 0.0

    def reset(self) -> None:
        self.swingup.reset()
        self.capture.reset()
        self.previous_command = 0.0

    @property
    def captured(self) -> bool:
        return self.capture.locked

    def step(self, x_ticks: float, x_vel_ticks_s: float,
             angle: float, angle_rate: float) -> float:
        """One 5 ms frame: update capture, return the unslewed command."""
        self.capture.update(angle, angle_rate)
        if self.capture.locked:
            return lqr_command(x_ticks, x_vel_ticks_s, angle, angle_rate)
        return self.swingup.compute(x_ticks, x_vel_ticks_s, angle, angle_rate)

    def apply_slew(self, command: float) -> float:
        """Firmware's per-frame limiter, selected by the capture state."""
        limit = BALANCE_SLEW if self.capture.locked else SWINGUP_SLEW
        self.previous_command += clamp(command - self.previous_command, -limit, limit)
        return self.previous_command
