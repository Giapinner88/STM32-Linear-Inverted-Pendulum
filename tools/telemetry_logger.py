#!/usr/bin/env python3
"""Passive telemetry logger: records MCU telemetry to CSV without sending
commands, so the on-board automatic sequence keeps control of the cart.

Usage:
  ~/.platformio/penv/bin/python tools/telemetry_logger.py [--port /dev/ttyUSB0]
      [--seconds 60] [--out logs/run.csv]
"""

from __future__ import annotations

import argparse
import csv
import os
import time

import serial

from serial_link import PacketParser, list_candidate_ports

FLAG_CAPTURED = 0x01
FLAG_RUNNING = 0x02
FLAG_RAIL_BLOCKED = 0x04


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", help="Serial port (default: first ttyUSB/ttyACM)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--seconds", type=float, default=0.0,
                   help="Stop after this many seconds (0 = until Ctrl+C)")
    p.add_argument("--out", help="CSV path (default: logs/telemetry_<time>.csv)")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    port = args.port
    if port is None:
        ports = list_candidate_ports()
        if not ports:
            raise SystemExit("No /dev/ttyUSB* or /dev/ttyACM* port found")
        port = ports[0]

    out = args.out or time.strftime("logs/telemetry_%Y%m%d_%H%M%S.csv")
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    ser = serial.Serial(port, args.baud, timeout=0.02)
    parser = PacketParser()
    count = 0
    lost = 0
    last_frame = None
    t0 = time.monotonic()
    last_print = 0.0
    print(f"Logging {port} @ {args.baud} -> {out}  (Ctrl+C to stop)")

    with open(out, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["host_s", "frame", "x_ticks", "x_vel_ticks_s",
                         "theta_rad", "theta_vel_rad_s", "u",
                         "captured", "running", "rail_blocked"])
        try:
            while args.seconds <= 0 or time.monotonic() - t0 < args.seconds:
                for t in parser.feed(ser.read(512)):
                    now = time.monotonic() - t0
                    if last_frame is not None:
                        lost += (t.frame - last_frame - 1) & 0xFFFF
                    last_frame = t.frame
                    count += 1
                    writer.writerow([
                        f"{now:.4f}", t.frame, f"{t.x_pos:.0f}",
                        f"{t.x_vel:.1f}", f"{t.theta:.4f}",
                        f"{t.theta_vel:.3f}", f"{t.u:.4f}",
                        int(bool(t.flags & FLAG_CAPTURED)),
                        int(bool(t.flags & FLAG_RUNNING)),
                        int(bool(t.flags & FLAG_RAIL_BLOCKED)),
                    ])
                    if now - last_print >= 0.5:
                        last_print = now
                        print(f"\r{now:6.1f}s pkts={count} lost={lost} "
                              f"x={t.x_pos:6.0f} th={t.theta:6.2f} "
                              f"u={t.u:+.2f} flags={t.flags:03b}   ",
                              end="", flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()

    print(f"\nSaved {count} packets ({lost} lost) to {out}")


if __name__ == "__main__":
    main()
