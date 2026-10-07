"""Closed-loop sim of the exact laptop pipeline: NmpcPolicy swing-up, the
Python capture detector, LQR once captured, 2-frame delay, board slew/guard."""
import math, random, sys
import numpy as np
sys.path.insert(0, "../tools")
import cartpole_control as cc
import cartpole_nmpc as m
from nmpc_policy import NmpcPolicy
from realtime_guard import RealtimeGuard
import time
from sim_closed_loop import MPT, UP, GUARD, TS, plant_step

def run(pol, seed, T=25.0, overload=None):
    rng = random.Random(seed)
    s = [rng.uniform(-0.02, 0.02), 0.0, math.pi + rng.uniform(-0.03, 0.03), 0.0]
    guard = RealtimeGuard(); res_ev = guard.events
    hyb = cc.Hybrid(); cap = hyb.capture; hist = []; filt = 0.0; pipe = [0.0, 0.0]; applied = 0.0
    res = dict(hard=0, guard=0, first=None, sustained=None, maxx=0.0, fallback=0)
    run_cap = 0.0; blocked = False; pol.reseed = True; pol.u_prev = 0.0; pol.in_flight.extend([0.0, 0.0])
    # warm on the hanging pendulum like the idle phase does
    th_app0 = UP - s[2]
    for _ in range(20): pol.idle(round(-s[0]/MPT), 0.0, th_app0, 0.0)
    for k in range(int(T / TS)):
        t = k * TS
        th_app = round((UP - s[2] + rng.gauss(0, 0.0008)) / 0.00153) * 0.00153
        ticks = round(-s[0] / MPT)
        hist.append((th_app, ticks, t)); old = hist[-5] if len(hist) > 4 else hist[0]
        span = (t - old[2]) or TS
        filt = 0.5 * filt + 0.5 * (th_app - old[0]) / span
        # raw encoder direction is left-positive: x_ticks_rel = -(reference ticks)... the sim's "ticks" is already raw
        xv = (ticks - old[1]) / span
        u_fb = hyb.step(ticks, xv, th_app, filt)
        extra = 10.0 if (overload and overload[0] <= t < overload[1]) else 0.0
        if guard.fallback:
            res["fallback"] += 1
            u = u_fb; pol.follow(u)
            if guard.tick_fallback():
                guard.record_trial(pol.trial(ticks, xv, th_app, filt) + extra, False)
        else:
            t0 = time.perf_counter()
            u, ok = pol.step(ticks, xv, th_app, filt)
            critical_ms = (time.perf_counter() - t0) * 1000.0
            if not ok:
                u = u_fb; pol.follow(u)
            else:
                pol.prepare()
            guard.record(critical_ms + extra, False, ok)
            if guard.fallback:
                pol.interrupt()
        cap = hyb.capture
        pipe.append(u); u_in = pipe.pop(0)
        lim = 0.10 if cap.locked else 0.1125
        applied += float(np.clip(u_in - applied, -lim, lim))
        ue = applied
        if (s[0] >= GUARD and ue > 0) or (s[0] <= -GUARD and ue < 0):
            if not blocked: res["guard"] += 1
            blocked = True; ue = 0.0; applied = 0.0
        else: blocked = False
        for _ in plant_step(s, ue): res["hard"] += 1
        res["maxx"] = max(res["maxx"], abs(s[0]))
        err = abs(((s[2] + math.pi) % (2*math.pi)) - math.pi)
        if err < 0.1 and abs(s[3]) < 2.0:
            run_cap += TS
            res["first"] = res["first"] if res["first"] is not None else t
            if run_cap >= 3.0 and res["sustained"] is None: res["sustained"] = t
        else: run_cap = 0.0
    res['events'] = [(k, round(f*TS,2), w) for k, f, w in guard.events]
    return res

if __name__ == "__main__":
    pol = NmpcPolicy()
    over = (8.0, 9.5) if "--overload" in sys.argv else None
    rs = []
    for sd in range(6):
        pol.solve_ms.clear(); r = run(pol, sd, overload=over); rs.append(r)
        print(f"\nRESULT seed {sd}: sustained {r['sustained']}s hard {r['hard']} guard {r['guard']} max|x| {r['maxx']*100:.1f}cm frames on LQR+ES {r['fallback']} events {r['events']}", flush=True)
    ok = sum(r["sustained"] is not None for r in rs)
    print(f"\nSUMMARY sustained capture {ok}/6, hard hits {sum(r['hard'] for r in rs)}, guard trips {sum(r['guard'] for r in rs)}")
