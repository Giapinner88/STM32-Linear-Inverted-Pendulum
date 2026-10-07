"""Closed-loop simulation of the RTI-NMPC against the identified plant.

Plant, sensor quantisation, the firmware's rate estimator, the rail guard, the
~10 ms link delay and the board's slew limit follow tools/ and the logs.
"""

from __future__ import annotations

import math
import random
import sys
import time

import numpy as np

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


def predict(state, in_flight, u_next):
    """Advance the measured state over the commands that are sent but not yet
    applied, so the NMPC plans from where the cart will be when its new command
    takes effect. state = [x, xd, th, thd]."""
    s = list(state)
    for u in in_flight:
        for _ in plant_step(s, u):
            pass
    return s


def run(solver, seed, T=15.0, delay_frames=2, warm_iters=40, verbose=False,
        use_lqr_handoff=False, delay_comp=True):
    rng = random.Random(seed)
    s = [rng.uniform(-0.02, 0.02), 0.0, PI + rng.uniform(-0.03, 0.03), 0.0]
    hist, filt = [], 0.0
    pipe = [0.0] * delay_frames
    u_applied = 0.0
    u_prev_cmd = 0.0
    stats = dict(hard=0, guard=0, times=[], bad=0, cap=None, sustained=None, maxx=0.0)
    run_cap = 0.0
    blocked = False
    solver.reset()
    first = True
    for k in range(int(T / TS)):
        t = k * TS
        th_app = round((UP - s[2] + rng.gauss(0, 0.0008)) / 0.00153) * 0.00153
        ticks = round(-s[0] / MPT)
        hist.append((th_app, ticks, t))
        old = hist[-5] if len(hist) > 4 else hist[0]
        span = (t - old[2]) or TS
        filt = 0.5 * filt + 0.5 * (th_app - old[0]) / span
        # measurement in the reference frame
        meas = np.array([-ticks * MPT, -(ticks - old[1]) / span * MPT,
                         wrap(UP - th_app - 0.0) if False else (UP - th_app),
                         -filt, u_prev_cmd])
        if delay_comp and delay_frames:
            pred = predict(meas[:4], list(pipe), u_prev_cmd)
            meas[:4] = pred
        solver.set(0, "lbx", meas)
        solver.set(0, "ubx", meas)
        if first:
            # acados starts from an all-zero trajectory (pendulum upright);
            # seed it with the measured state, then converge before using it.
            for i in range(solver.acados_ocp.solver_options.N_horizon + 1):
                solver.set(i, "x", meas)
            for i in range(solver.acados_ocp.solver_options.N_horizon):
                solver.set(i, "u", np.zeros(1))
            for i in range(warm_iters):
                solver.solve()
            first = False
        t0 = time.perf_counter()
        status = solver.solve()
        stats["times"].append(time.perf_counter() - t0)
        if status not in (0, 2):
            stats["bad"] += 1
        v0 = solver.get(0, "u")[0]
        u_cmd = float(np.clip(u_prev_cmd + v0 * TS, -m.U_MAX, m.U_MAX))
        u_prev_cmd = u_cmd
        pipe.append(u_cmd)
        u_in = pipe.pop(0)
        # board: slew (non-binding by construction) and rail guard
        u_applied += float(np.clip(u_in - u_applied, -0.1125, 0.1125))
        u_eff = u_applied
        if (s[0] >= GUARD and u_eff > 0) or (s[0] <= -GUARD and u_eff < 0):
            if not blocked:
                stats["guard"] += 1
            blocked = True
            u_eff = 0.0
            u_applied = 0.0
        else:
            blocked = False
        for ev in plant_step(s, u_eff):
            stats["hard"] += 1
        stats["maxx"] = max(stats["maxx"], abs(s[0]))
        err = abs(wrap(s[2]))
        if err < 0.1 and abs(s[3]) < 2.0:
            run_cap += TS
            if stats["cap"] is None:
                stats["cap"] = t
            if run_cap >= 3.0 and stats["sustained"] is None:
                stats["sustained"] = t
        else:
            run_cap = 0.0
        if verbose and k % 40 == 0:
            print(f"  t={t:5.2f} x={s[0]/MPT:+6.0f}tk th_err={math.degrees(wrap(s[2])):+7.1f}deg u={u_cmd:+.2f} st={status}")
    return stats


if __name__ == "__main__":
    solver = m.build(rebuild=("--no-build" not in sys.argv))
    seeds = range(6)
    for sd in seeds:
        r = run(solver, sd, verbose=(sd == 0 and "-v" in sys.argv))
        tm = np.array(r["times"]) * 1000
        print(f"seed {sd}: sustained capture at {r['sustained']}s, 1st upright {r['cap']}, "
              f"max|x| {r['maxx']*100:.1f} cm, guard trips {r['guard']}, hard hits {r['hard']}, "
              f"solver fails {r['bad']}, RTI time mean {tm.mean():.2f} ms p95 {np.percentile(tm,95):.2f} max {tm.max():.2f}")
