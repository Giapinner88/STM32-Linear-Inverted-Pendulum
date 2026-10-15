#!/usr/bin/env python3
"""Binary serial bridge for STM32 inverted pendulum firmware.

Protocol:
- Telemetry packet (MCU -> PC): 0xAA 0xBB + 5 floats (x_pos, x_vel, theta,
  theta_vel, u) + uint16 frame + uint8 flags + uint8 cmd_seq + int16
  center_ticks + uint8 checksum (sum of bytes after the header)
- Command packet (PC -> MCU): 0xCC 0xDD + float control_effort + uint8 mode
  + uint8 seq + uint8 checksum (sum of the float, mode and seq bytes)
"""

from __future__ import annotations

import argparse
import glob
import math
import struct
import time
from collections import deque
from dataclasses import dataclass

import serial

TELEMETRY_HEADER = b"\xAA\xBB"
COMMAND_HEADER = b"\xCC\xDD"
TELEMETRY_FMT = "<2sfffffHBBhB"
COMMAND_FMT = "<2sfBBB"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FMT)

MODE_IDLE = 0
MODE_FULL = 4  # laptop u in every phase while the automatic sequence runs


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


SWINGUP_SLEW = 0.1125
BALANCE_SLEW = 0.10

def wrap_pi(angle: float) -> float:
    while angle > PI_F:
        angle -= 2.0 * PI_F
    while angle < -PI_F:
        angle += 2.0 * PI_F
    return angle


def theta_error(angle: float) -> float:
    return wrap_pi(angle - UPRIGHT_ANGLE_RAD)


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



@dataclass
class Telemetry:
    x_pos: float
    x_vel: float
    theta: float
    theta_vel: float
    u: float = 0.0
    frame: int = 0
    flags: int = 0
    cmd_seq: int = 0
    center_ticks: int = 0


class PacketParser:
    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[Telemetry]:
        out: list[Telemetry] = []
        self._buf.extend(data)

        while True:
            if len(self._buf) < TELEMETRY_SIZE:
                return out

            header_pos = self._buf.find(TELEMETRY_HEADER)
            if header_pos < 0:
                # Keep at most one byte in case it is first header byte.
                self._buf[:] = self._buf[-1:]
                return out

            if header_pos > 0:
                del self._buf[:header_pos]

            if len(self._buf) < TELEMETRY_SIZE:
                return out

            raw = bytes(self._buf[:TELEMETRY_SIZE])
            if (sum(raw[2:-1]) & 0xFF) != raw[-1]:
                # Header bytes inside a payload: skip one byte and resync.
                del self._buf[:1]
                continue
            del self._buf[:TELEMETRY_SIZE]

            (_, x_pos, x_vel, theta, theta_vel, u, frame, flags, cmd_seq,
             center_ticks, _) = struct.unpack(TELEMETRY_FMT, raw)
            out.append(Telemetry(x_pos=x_pos, x_vel=x_vel, theta=theta,
                                 theta_vel=theta_vel, u=u, frame=frame,
                                 flags=flags, cmd_seq=cmd_seq,
                                 center_ticks=center_ticks))


def build_command(control_effort: float, mode: int, seq: int = 0) -> bytes:
    u = max(-1.0, min(1.0, control_effort))
    m = mode if mode in (MODE_IDLE, MODE_FULL) else MODE_IDLE
    body = struct.pack("<fBB", u, m, seq & 0xFF)
    return COMMAND_HEADER + body + bytes([sum(body) & 0xFF])


def list_candidate_ports() -> list[str]:
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports


def probe_ports(baud: int, seconds: float) -> int:
    ports = list_candidate_ports()

    if not ports:
        print("No candidate serial ports found")
        return 1

    print(f"Probing {len(ports)} port(s) at {baud} baud for {seconds:.1f}s each")
    found_any = False

    for port in ports:
        total = 0
        try:
            ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as exc:  # pragma: no cover - hardware dependent
            print(f"{port}: open failed ({exc})")
            continue

        start = time.monotonic()
        while time.monotonic() - start < seconds:
            total += len(ser.read(4096))
        ser.close()

        if total > 0:
            found_any = True
        print(f"{port}: {total} byte(s)")

    return 0 if found_any else 2


def run(args: argparse.Namespace) -> None:
    if args.probe:
        raise SystemExit(probe_ports(args.baud, args.probe_seconds))

    ser = serial.Serial(args.port, args.baud, timeout=0.01)
    parser = PacketParser()

    last_tx = 0.0
    tx_period = 1.0 / args.tx_hz
    t0 = time.monotonic()

    samples: deque[Telemetry] = deque(maxlen=10)
    print(f"Connected: {args.port} @ {args.baud}")
    print("Press Ctrl+C to stop")

    try:
        while True:
            now = time.monotonic()

            if now - last_tx >= tx_period:
                t = now - t0
                u = args.u
                if args.sine:
                    u = args.amp * math.sin(2.0 * math.pi * args.freq * t)
                ser.write(build_command(u, args.mode))
                last_tx = now

            rx = ser.read(256)
            for telem in parser.feed(rx):
                samples.append(telem)

            if samples:
                latest = samples[-1]
                print(
                    f"x={latest.x_pos:8.1f}  dx={latest.x_vel:8.1f}  "
                    f"th={latest.theta:8.1f}  dth={latest.theta_vel:8.1f}",
                    end="\r",
                    flush=True,
                )

    except KeyboardInterrupt:
        pass
    finally:
        ser.write(build_command(0.0, MODE_IDLE))
        ser.close()
        print("\nSerial closed")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="STM32 binary serial link test")
    p.add_argument("--port", default="/dev/ttyACM0", help="Serial port (e.g. /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    p.add_argument("--mode", type=int, default=MODE_IDLE, choices=[0, 4], help="Control mode")
    p.add_argument("--u", type=float, default=0.0, help="Constant control effort in [-1,1]")
    p.add_argument("--tx-hz", type=float, default=100.0, help="Command transmit rate")
    p.add_argument("--sine", action="store_true", help="Send sinusoidal control effort")
    p.add_argument("--amp", type=float, default=0.2, help="Sine amplitude")
    p.add_argument("--freq", type=float, default=0.5, help="Sine frequency in Hz")
    p.add_argument("--probe", action="store_true", help="Probe all ttyACM/ttyUSB ports for incoming bytes")
    p.add_argument("--probe-seconds", type=float, default=2.0, help="Probe duration per port")
    return p.parse_args()


if __name__ == "__main__":
    run(parse_args())
