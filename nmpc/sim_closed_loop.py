"""Shared physical constants and delay-prediction integration for NMPC."""

from __future__ import annotations

import math


import cartpole_nmpc as m

PI = math.pi
UP = 3.1415926 + 0.127
MPT = (0.036 * 3.1415926) / 1040.0
TS = 0.005
GUARD = 1945 * MPT
HARD = 2065 * MPT


def wrap(a):
    return (a + PI) % (2 * PI) - PI


def plant_step(s, u, rng_sub=10):
    x, xd, th, thd = s
    for _ in range(rng_sub):
        dt = TS / rng_sub
        a = (m.SUPPLY_V * u / m.KV - xd) / m.TAU if abs(u) >= 0.001 else \
            -2.8 * xd - 0.23 * math.copysign(1, xd) * (abs(xd) > 1e-3)
        thdd = m.W2 * math.sin(th) - m.G2 * a * math.cos(th) - m.DAMP * thd
        xd += a * dt
        x += xd * dt
        thd += thdd * dt
        th += thd * dt
        if abs(x) > HARD:
            dv = -xd
            xd = 0.0
            x = math.copysign(HARD, x)
            thd += -m.G2 * math.cos(th) * dv
            yield "hard"
    s[:] = [x, xd, th, thd]
