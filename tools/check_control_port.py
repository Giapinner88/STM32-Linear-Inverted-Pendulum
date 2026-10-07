#!/usr/bin/env python3
"""Compare tools/cartpole_control.py against the firmware's C controller.

Builds lib/CartpoleControl into a shared library and drives both with the same
random state sequences (random walks, so the capture and return state machines
are exercised), then reports command and capture-flag differences.
"""

from __future__ import annotations

import ctypes
import math
import os
import random
import subprocess
import tempfile

import cartpole_control as cc
from cartpole_control import PI_F, UPRIGHT_ANGLE_RAD, Hybrid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = os.path.join(ROOT, "lib", "CartpoleControl", "src")

SHIM = r"""
#include "hybrid_controller.h"
static HybridController c;
void shim_reset(void) { HybridController_Reset(&c); }
void shim_step(float pos, float vel, float th, float thd,
               float *raw, float *slewed, int *captured)
{
  HybridController_UpdateCapture(&c, th, thd);
  *raw = HybridController_Compute(&c, pos, vel, th, thd);
  *slewed = HybridController_ApplySlew(&c, *raw);
  *captured = c.upright_locked;
}
"""


def build() -> ctypes.CDLL:
    tmp = tempfile.mkdtemp()
    shim = os.path.join(tmp, "shim.c")
    with open(shim, "w") as fh:
        fh.write(SHIM)
    sources = [os.path.join(LIB, n) for n in os.listdir(LIB) if n.endswith(".c")]
    so = os.path.join(tmp, "libcp.so")
    subprocess.check_call(["gcc", "-shared", "-fPIC", "-O2", "-I", os.path.join(ROOT, "include"),
                           "-I", LIB, shim, *sources, "-lm", "-o", so])
    lib = ctypes.CDLL(so)
    lib.shim_step.argtypes = [ctypes.c_float] * 4 + [ctypes.POINTER(ctypes.c_float)] * 2 + [ctypes.POINTER(ctypes.c_int)]
    return lib


def main() -> int:
    # Compare against the firmware's own cart-return window.
    cc.WINDOW_HALF_TICKS = cc.FIRMWARE_WINDOW_HALF_TICKS
    cc.RETURN_EXIT_HALF_TICKS = cc.FIRMWARE_RETURN_EXIT_HALF_TICKS
    lib = build()
    rng = random.Random(7)
    steps = max_raw = max_slew = flag_mismatch = decision_mismatch = 0
    max_raw = max_slew = 0.0
    worst = None
    for episode in range(400):
        lib.shim_reset()
        py = Hybrid()
        x = rng.uniform(-1500, 1500)
        v = rng.uniform(-3000, 3000)
        th = rng.uniform(0, 2 * PI_F)
        w = rng.uniform(-10, 10)
        for _ in range(1500):
            # Random walk, with occasional large jumps near upright so the
            # capture hysteresis is crossed in both directions.
            th += w * 0.005 + rng.gauss(0, 0.01)
            w += rng.gauss(0, 0.6)
            w = max(-14.0, min(14.0, w))
            x += v * 0.005
            v += rng.gauss(0, 150)
            if abs(x) > 2000:
                v = -math.copysign(abs(v), x)
            if rng.random() < 0.01:
                th = UPRIGHT_ANGLE_RAD + rng.gauss(0, 0.2)
                w = rng.gauss(0, 1.5)
            raw = ctypes.c_float()
            slewed = ctypes.c_float()
            cap = ctypes.c_int()
            lib.shim_step(x, v, th, w, ctypes.byref(raw), ctypes.byref(slewed), ctypes.byref(cap))
            u_raw = py.step(x, v, th, w)
            u_slew = py.apply_slew(u_raw)
            steps += 1
            if int(py.captured) != cap.value:
                flag_mismatch += 1
                # Realign so one boundary case does not cascade.
                py.capture.locked = bool(cap.value)
                continue
            d = abs(u_raw - raw.value)
            ds = abs(u_slew - slewed.value)
            if d > 1e-3:
                decision_mismatch += 1
            if d > max_raw:
                max_raw, worst = d, (x, v, th, w, u_raw, raw.value)
            max_slew = max(max_slew, ds)
            py.previous_command = slewed.value  # follow the C state
    print(f"{steps} steps over 400 random episodes")
    print(f"capture flag mismatches: {flag_mismatch}")
    print(f"steps with |python - C| > 1e-3 in the raw command: {decision_mismatch}")
    print(f"max raw difference {max_raw:.2e}, max slewed difference {max_slew:.2e}")
    if worst and max_raw > 1e-3:
        print("worst case (x, v, th, w, python, C):", worst)
    ok = flag_mismatch <= steps * 1e-4 and decision_mismatch <= steps * 1e-4
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
