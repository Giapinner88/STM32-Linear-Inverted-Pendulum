#!/usr/bin/env python3
"""NMPC (or the LQR + energy-shaping law) on the MuJoCo cart-pole.

The plant is sim/cartpole_identified.xml, the cart motor follows the identified
first-order model, and the controller sees what the firmware would hand it:
quantised pot and encoder, the 20 ms differencing estimator, a 10 ms link
delay, the board's slew limit and rail guard. That is the same chain the laptop
runs on hardware, so a run here is a rehearsal of `nmpc/run_nmpc.sh`.

  nmpc/sim_mujoco.sh --viewer                 # watch one pure-NMPC run
  nmpc/sim_mujoco.sh --ctrl blend --viewer    # NMPC with the LQR + ES fallback
  nmpc/sim_mujoco.sh --ctrl hybrid --viewer   # the LQR + energy-shaping law
  nmpc/sim_mujoco.sh --ctrl all --seeds 6     # headless comparison of the three
  nmpc/sim_mujoco.sh --validate               # MuJoCo plant vs the ODE model
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))

import mujoco

import cartpole_control as cc
import cartpole_nmpc as m
from sim_closed_loop import GUARD, HARD, MPT, TS, UP

XML = os.path.join(os.path.dirname(HERE), "sim", "cartpole_identified.xml")
SUBSTEPS = 10


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class MujocoPlant:
    """State in the controller's reference frame: x right, theta from upright."""

    def __init__(self) -> None:
        self.model = mujoco.MjModel.from_xml_path(XML)
        self.data = mujoco.MjData(self.model)
        cart = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cart")
        self.cart_mass = float(self.model.body_subtreemass[cart])
        self.hard_hits = 0
        self._at_stop = False

    def reset(self, rng: random.Random) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0] = rng.uniform(-0.02, 0.02)
        self.data.qpos[1] = rng.uniform(-0.03, 0.03)   # hanging is theta_mj = 0
        mujoco.mj_forward(self.model, self.data)
        self.hard_hits = 0
        self._at_stop = False

    def state(self) -> list[float]:
        d = self.data
        return [d.qpos[0], d.qvel[0], d.qpos[1] - math.pi, d.qvel[1]]

    def step(self, u: float) -> None:
        d = self.data
        dt = self.model.opt.timestep
        for _ in range(SUBSTEPS):
            xd = d.qvel[0]
            if abs(u) >= 0.001:
                a = (m.SUPPLY_V * u / m.KV - xd) / m.TAU
            else:  # H-bridge off: the cart coasts against friction
                a = -2.8 * xd - 0.23 * math.copysign(1.0, xd) * (abs(xd) > 1e-3)
            d.ctrl[0] = self.cart_mass * a
            mujoco.mj_step(self.model, d)
        at_stop = abs(d.qpos[0]) >= 0.2245 - 5e-4
        if at_stop and not self._at_stop:
            self.hard_hits += 1
        self._at_stop = at_stop


class Episode:
    def __init__(self, ctrl: str, seed: int, plant: MujocoPlant, policy=None) -> None:
        self.ctrl, self.plant = ctrl, plant
        self.rng = random.Random(seed)
        plant.reset(self.rng)
        self.hyb = cc.Hybrid()
        self.policy = policy
        self.guard = None
        self.hist, self.filt = [], 0.0
        self.pipe = [0.0, 0.0]
        self.applied = 0.0
        self.blocked = False
        self.k = 0
        self.res = dict(guard=0, first=None, sustained=None, maxx=0.0, fallback=0, events=[], qp_fail=0)
        self.run_cap = 0.0
        self.u_cmd = 0.0
        self.mode = "hybrid"
        if ctrl in ("nmpc", "blend"):
            if ctrl == "blend":
                from realtime_guard import RealtimeGuard
                self.guard = RealtimeGuard()
            policy.reset()
            s = plant.state()
            for _ in range(20):  # the idle phase keeps the solver warm
                policy.idle(round(-s[0] / MPT), 0.0, UP - s[2], 0.0)
                policy.prepare()

    def frame(self) -> None:
        s, t, rng = self.plant.state(), self.k * TS, self.rng
        th_app = round((UP - s[2] + rng.gauss(0, 0.0008)) / 0.00153) * 0.00153
        ticks = round(-s[0] / MPT)
        self.hist.append((th_app, ticks, t))
        old = self.hist[-5] if len(self.hist) > 4 else self.hist[0]
        span = (t - old[2]) or TS
        self.filt = 0.5 * self.filt + 0.5 * (th_app - old[0]) / span
        xv = (ticks - old[1]) / span
        u_fb = self.hyb.step(ticks, xv, th_app, self.filt)
        if self.ctrl == "hybrid":
            u, self.mode = u_fb, ("LQR" if self.hyb.captured else "swing")
        else:
            u, self.mode = self._nmpc(ticks, xv, th_app, u_fb)
        self.u_cmd = u
        self.pipe.append(u)
        u_in = self.pipe.pop(0)
        lim = cc.BALANCE_SLEW if self.hyb.captured else cc.SWINGUP_SLEW
        self.applied += float(np.clip(u_in - self.applied, -lim, lim))
        ue = self.applied
        if (s[0] >= GUARD and ue > 0) or (s[0] <= -GUARD and ue < 0):
            if not self.blocked:
                self.res["guard"] += 1
            self.blocked, ue, self.applied = True, 0.0, 0.0
        else:
            self.blocked = False
        self.plant.step(ue)
        self.k += 1
        s = self.plant.state()
        self.res["maxx"] = max(self.res["maxx"], abs(s[0]))
        if abs(wrap(s[2])) < 0.1 and abs(s[3]) < 2.0:
            self.run_cap += TS
            if self.res["first"] is None:
                self.res["first"] = t
            if self.run_cap >= 3.0 and self.res["sustained"] is None:
                self.res["sustained"] = t
        else:
            self.run_cap = 0.0

    def _nmpc(self, ticks, xv, th_app, u_fb):
        p, g = self.policy, self.guard
        if g is not None and g.fallback:
            self.res["fallback"] += 1
            p.follow(u_fb)
            if g.tick_fallback():
                g.record_trial(p.trial(ticks, xv, th_app, self.filt), False)
            return u_fb, "LQR+ES"
        t0 = time.perf_counter()
        u, ok = p.step(ticks, xv, th_app, self.filt)
        critical_ms = (time.perf_counter() - t0) * 1000.0
        if ok:
            p.prepare()
        else:
            self.res["qp_fail"] += 1
            u = u_fb if g is not None else 0.0   # pure NMPC coasts for the frame
            p.follow(u)
        if g is not None:
            g.record(critical_ms, False, ok)
            if g.fallback:
                p.interrupt()
        return u, "NMPC"

    def finish(self) -> dict:
        if self.guard is not None:
            self.res["events"] = [(k, round(f * TS, 2), w) for k, f, w in self.guard.events]
        self.res["hard"] = self.plant.hard_hits
        return self.res


def run_headless(ctrl, seeds, seconds, policy):
    plant = MujocoPlant()
    out = []
    for sd in seeds:
        ep = Episode(ctrl, sd, plant, policy)
        if policy is not None:
            policy.solve_ms.clear()
        for _ in range(int(seconds / TS)):
            ep.frame()
        r = ep.finish()
        ms = np.array(policy.solve_ms) if (policy is not None and policy.solve_ms) else None
        out.append(r)
        sus = "none" if r["sustained"] is None else f"{r['sustained']:5.1f}s"
        extra = (f" | solve median {np.median(ms):.2f} p95 {np.percentile(ms, 95):.2f} ms, "
                 f"{r['fallback']} frames on LQR+ES {r['events']}" if ms is not None else "")
        print(f"\nRESULT {ctrl:6s} seed {sd}: sustained capture {sus}, rail hits {r['hard']}, "
              f"guard trips {r['guard']}, max|x| {r['maxx'] * 100:4.1f} cm{extra}", flush=True)
    return out


def run_viewer(ctrl, seed, seconds, policy):
    import mujoco.viewer
    plant = MujocoPlant()
    ep = Episode(ctrl, seed, plant, policy)
    with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_side")
        print(f"{ctrl} on the identified plant, seed {seed}. Close the window to stop.")
        t_wall = time.perf_counter()
        last_print = -1.0
        while viewer.is_running() and ep.k * TS < seconds:
            ep.frame()
            viewer.sync()
            t = ep.k * TS
            if t - last_print >= 1.0:
                last_print = t
                s = plant.state()
                print(f"{t:5.0f}s {ep.mode:7s} x={s[0] * 100:+6.1f}cm angle={math.degrees(wrap(s[2])):+7.1f}deg "
                      f"u={ep.u_cmd:+.2f} rail hits={plant.hard_hits}", flush=True)
            lag = t - (time.perf_counter() - t_wall)
            if lag > 0:
                time.sleep(lag)
    r = ep.finish()
    print(f"\nsustained capture {r['sustained']}, rail hits {r['hard']}, guard trips {r['guard']}, "
          f"max|x| {r['maxx'] * 100:.1f} cm, fallback frames {r['fallback']} {r['events']}")


def validate() -> int:
    """Open-loop: the same command sequence through MuJoCo and the ODE the NMPC uses."""
    from sim_closed_loop import plant_step
    rng = random.Random(1)
    plant = MujocoPlant()
    worst = []
    for trial in range(8):
        plant.reset(random.Random(trial))
        s = plant.state()
        ode = [s[0], s[1], s[2], s[3]]
        us, u = [], 0.0
        for k in range(600):          # 3 s, new random command every 100 ms
            if k % 20 == 0:
                u = rng.uniform(-0.5, 0.5)
            us.append(u)
        ex = eth = 0.0
        for k, u in enumerate(us):
            plant.step(u)
            for _ in plant_step(ode, u):
                pass
            s = plant.state()
            ex = max(ex, abs(s[0] - ode[0]))
            eth = max(eth, abs(s[2] - ode[2]))
        worst.append((ex, eth))
        print(f"trial {trial}: max |x| difference {ex * 1000:6.2f} mm, max theta difference {math.degrees(eth):5.2f} deg")
    ok = max(w[0] for w in worst) < 0.01 and max(w[1] for w in worst) < 0.1
    print("PASS" if ok else "CHECK: plant and ODE differ more than expected")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ctrl", choices=["nmpc", "blend", "hybrid", "all"], default="nmpc",
                    help="nmpc = pure NMPC; blend = NMPC + LQR/ES fallback; hybrid = LQR + ES")
    ap.add_argument("--seeds", type=int, default=6, help="headless: number of random starts")
    ap.add_argument("--seed-start", type=int, default=0, help="headless: first random start")
    ap.add_argument("--seed", type=int, default=0, help="viewer: random start")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--viewer", action="store_true", help="watch one run in the MuJoCo viewer")
    ap.add_argument("--validate", action="store_true", help="compare the plant with the ODE model")
    a = ap.parse_args()
    if a.validate:
        return validate()
    policy = None
    if a.ctrl in ("nmpc", "blend", "all"):
        from nmpc_policy import NmpcPolicy
        policy = NmpcPolicy()
    if a.viewer:
        run_viewer("nmpc" if a.ctrl == "all" else a.ctrl, a.seed, a.seconds, policy)
        # acados and the MuJoCo viewer crash against each other during normal
        # interpreter teardown; everything is printed, so leave directly.
        sys.stdout.flush()
        os._exit(0)
    results = {}
    for c in (["nmpc", "blend", "hybrid"] if a.ctrl == "all" else [a.ctrl]):
        results[c] = run_headless(c, range(a.seed_start, a.seed_start + a.seeds), a.seconds,
                                  policy if c in ("nmpc", "blend") else None)
    print("\n=== summary")
    for c, rs in results.items():
        ok = [r["sustained"] for r in rs if r["sustained"] is not None]
        print(f"{c:6s}: sustained capture {len(ok)}/{len(rs)}"
              f"{', median ' + format(float(np.median(ok)), '.1f') + ' s' if ok else ''} | "
              f"rail hits/run {np.mean([r['hard'] for r in rs]):.1f} | guard trips/run "
              f"{np.mean([r['guard'] for r in rs]):.1f} | max|x| {max(r['maxx'] for r in rs) * 100:.1f} cm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
