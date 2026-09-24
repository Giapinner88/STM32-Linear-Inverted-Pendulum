#!/usr/bin/env python3
"""Binary serial bridge for STM32 inverted pendulum firmware.

Protocol:
- Telemetry packet (MCU -> PC): 0xAA 0xBB + 5 floats (x_pos, x_vel, theta,
  theta_vel, u) + uint16 frame + uint8 flags + uint8 checksum
- Command packet (PC -> MCU): 0xCC 0xDD + float control_effort + uint8 mode
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
TELEMETRY_FMT = "<2sfffffHBB"
COMMAND_FMT = "<2sfB"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FMT)

MODE_IDLE = 0
MODE_SWINGUP = 1
MODE_LQR = 2


@dataclass
class Telemetry:
    x_pos: float
    x_vel: float
    theta: float
    theta_vel: float
    u: float = 0.0
    frame: int = 0
    flags: int = 0


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

            (_, x_pos, x_vel, theta, theta_vel, u, frame, flags,
             _) = struct.unpack(TELEMETRY_FMT, raw)
            out.append(Telemetry(x_pos=x_pos, x_vel=x_vel, theta=theta,
                                 theta_vel=theta_vel, u=u, frame=frame,
                                 flags=flags))


def build_command(control_effort: float, mode: int) -> bytes:
    u = max(-1.0, min(1.0, control_effort))
    m = mode if mode in (MODE_IDLE, MODE_SWINGUP, MODE_LQR) else MODE_IDLE
    return struct.pack(COMMAND_FMT, COMMAND_HEADER, u, m)


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
    p.add_argument("--mode", type=int, default=MODE_IDLE, choices=[0, 1, 2], help="Control mode")
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
