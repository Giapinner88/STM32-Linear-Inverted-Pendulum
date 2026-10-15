#!/usr/bin/env python3
"""Pure NMPC on MuJoCo, with pot sensing, firmware filtering and actuator guards."""

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

import serial_link as cc
import cartpole_nmpc as m
from sim_closed_loop import GUARD, HARD, MPT, TS, UP

XML = os.path.join(os.path.dirname(HERE), "sim", "cartpole_identified.xml")
SUBSTEPS = 10


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class MujocoPlant:
    """State in the controller's reference frame: x right, theta from upright."""

    def __init__(self, mismatch: dict | None = None) -> None:
        """mismatch scales the plant away from the model the controllers know:
        tau, kv (cart motor), inertia (pole inertia about its centre), damping."""
        mm = mismatch or {}
        self.model = mujoco.MjModel.from_xml_path(XML)
        self.data = mujoco.MjData(self.model)
        cart = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cart")
        pole = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pole")
        self.model.body_inertia[pole] *= mm.get("inertia", 1.0)
        self.model.dof_damping[1] *= mm.get("damping", 1.0)
        self.tau, self.kv = m.TAU * mm.get("tau", 1.0), m.KV * mm.get("kv", 1.0)
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
                a = (m.SUPPLY_V * u / self.kv - xd) / self.tau
            else:  # H-bridge off: the cart coasts against friction
                a = -2.8 * xd - 0.23 * math.copysign(1.0, xd) * (abs(xd) > 1e-3)
            d.ctrl[0] = self.cart_mass * a
            mujoco.mj_step(self.model, d)
        at_stop = abs(d.qpos[0]) >= 0.2245 - 5e-4
        if at_stop and not self._at_stop:
            self.hard_hits += 1
        self._at_stop = at_stop


class PotSensor:
    """The angle pot as seen through the firmware: a reading scale of (pi+0.127)/pi,
    a blind spot where the reading freezes and sometimes glitches to mid-scale
    (as in logs/laptop_nmpc_*.csv), then the firmware's 2 pi unwrapping."""

    READING_SCALE = (math.pi + 0.127) / math.pi
    R_LO = -math.pi / 2  # ADC 0, zero at hanging = ADC 1024
    GAP = 2.0 * math.pi - 2.0 * math.pi / READING_SCALE

    def __init__(self, rng: random.Random, glitch_p: float = 0.4) -> None:
        self.rng, self.glitch_p = rng, glitch_p
        self.last_valid = None
        self.prev_r = None
        self.theta_fw = 0.0

    def read(self, th_ref: float) -> float:
        phi = math.pi - th_ref                                # true angle from hanging
        width = 2.0 * math.pi / self.READING_SCALE              # true angle covered by the pot
        phi_m = (phi - self.R_LO / self.READING_SCALE) % (2.0 * math.pi)
        if phi_m <= width:
            r = self.R_LO + self.READING_SCALE * phi_m + self.rng.gauss(0, 0.0008)
            self.last_valid = r
        else:                                                  # blind spot
            r = self.last_valid if self.last_valid is not None else self.R_LO
            if self.rng.random() < self.glitch_p:              # averaged ADC lands mid-scale
                r = self.R_LO + (r - self.R_LO + 2.6) % (2.0 * math.pi)
        r = round(r / 0.00153) * 0.00153
        if self.prev_r is None:
            self.theta_fw = r
        else:
            d = r - self.prev_r
            if d > math.pi:
                d -= 2.0 * math.pi
            elif d < -math.pi:
                d += 2.0 * math.pi
            self.theta_fw += d
        self.prev_r = r
        return self.theta_fw


class Episode:
    def __init__(self, ctrl: str, seed: int, plant: MujocoPlant, policy=None,
                 pot: str = "real", kick: float = 0.0,
                 extra_delay: int = 0, fail_inject: tuple | None = None,
                 knock_rate: float = 0.0, init_state: tuple | None = None) -> None:
        self.ctrl, self.plant = ctrl, plant
        self.pot_mode = pot
        self.rng = random.Random(seed)
        plant.reset(self.rng)
        if init_state is not None:
            # Start from a recorded state: (x [m], xd [m/s], theta from upright, rate, u).
            x0, xd0, th0, om0, u0 = init_state
            d = plant.data
            d.qpos[0], d.qpos[1], d.qvel[0], d.qvel[1] = x0, wrap(th0) + math.pi, xd0, om0
            mujoco.mj_forward(plant.model, d)
        self.capture = cc.Capture()
        self.pot = PotSensor(self.rng) if pot == "real" else None
        self.policy = policy
        self.hist, self.filt = [], 0.0
        self.pipe = [0.0] * (2 + extra_delay)   # the controller only compensates the first 2 frames
        self.applied = 0.0
        self.blocked = False
        self.k = 0
        self.res = dict(guard=0, first=None, sustained=None, maxx=0.0, fallback=0, events=[], qp_fail=0, qp_maxiter=0, solves=0)
        self.run_cap = 0.0
        self.kick, self.kick_t, self.kick_ok_t = kick, None, None
        self.fail_inject, self.burst = fail_inject, 0
        self.knock_rate, self.knock_t, self.recover_t = knock_rate, None, None
        self.u_cmd = 0.0
        self.mode = "NMPC"
        self.init_u = init_state[4] if init_state is not None else 0.0
        if init_state is not None:              # the recorded command is already on its way
            self.pipe = [self.init_u] * len(self.pipe)
            self.applied = self.init_u
        if ctrl == "nmpc":
            policy.reset()
            s = plant.state()
            for _ in range(20):  # the idle phase keeps the solver warm
                policy.idle(round(-s[0] / MPT), -s[1] / MPT, UP - s[2], -s[3])
                policy.prepare()
            policy.u_prev = self.init_u
            policy.in_flight.extend([self.init_u, self.init_u])
        if init_state is not None:
            self._warm_start(init_state)

    def _warm_start(self, st) -> None:
        """A recorded state is mid-motion: give the estimators the history they would have
        (the firmware's rate differencing) and let the solver converge on it,
        as it would have been running on the hardware."""
        x0, xd0, th0, om0, u0 = st
        s = self.plant.state()
        th_fw = self.pot.read(s[2]) if self.pot is not None else UP - s[2]
        for k in range(4, 0, -1):
            self.hist.append((th_fw + om0 * k * TS, round(-(x0 - xd0 * k * TS) / MPT), -k * TS))
        self.filt = -om0
        n_th, n_filt = th_fw, -om0
        if self.ctrl in ("nmpc", "blend"):
            for _ in range(10):
                self.policy.trial(round(-x0 / MPT), -xd0 / MPT, n_th, n_filt)

    def frame(self) -> None:
        s, t, rng = self.plant.state(), self.k * TS, self.rng
        if self.pot is not None:
            th_app = self.pot.read(s[2])
        else:
            th_app = round((UP - s[2] + rng.gauss(0, 0.0008)) / 0.00153) * 0.00153
        ticks = round(-s[0] / MPT)
        self.hist.append((th_app, ticks, t))
        old = self.hist[-5] if len(self.hist) > 4 else self.hist[0]
        span = (t - old[2]) or TS
        self.filt = 0.5 * self.filt + 0.5 * (th_app - old[0]) / span
        xv = (ticks - old[1]) / span
        self.capture.update(th_app, self.filt)
        self.n_th, self.n_filt = th_app, self.filt      # what NMPC is given
        u, self.mode = self._nmpc(ticks, xv)
        self.u_cmd = u
        self.pipe.append(u)
        u_in = self.pipe.pop(0)
        lim = cc.BALANCE_SLEW if self.capture.locked else cc.SWINGUP_SLEW
        self.applied += float(np.clip(u_in - self.applied, -lim, lim))
        ue = self.applied
        if (s[0] >= GUARD and ue > 0) or (s[0] <= -GUARD and ue < 0):
            if not self.blocked:
                self.res["guard"] += 1
            # Like the firmware: the H-bridge is stopped, the slewed command is kept.
            self.blocked, ue = True, 0.0
        else:
            self.blocked = False
        self.plant.step(ue)
        self.k += 1
        if self.knock_rate and self.res["sustained"] is not None and self.knock_t is None \
                and t >= self.res["sustained"] + 3.0:
            # Knock the pendulum down (someone pushes it), then time the swing-up and re-capture.
            self.knock_t = t
            self.plant.data.qvel[1] += self.knock_rate
            self.run_cap = 0.0
        if self.kick and self.res["sustained"] is not None:
            # A kick to the pendulum tip, as in the Freiburg exercise: 5 s after a sustained capture.
            if self.kick_t is None and t >= self.res["sustained"] + 5.0:
                self.kick_t = t
                self.res["hard_before_kick"] = self.plant.hard_hits
                self.plant.data.qpos[1] += self.kick
                self.plant.data.qvel[1] += 0.0
        s = self.plant.state()
        self.res["maxx"] = max(self.res["maxx"], abs(s[0]))
        if self.kick_t is not None and self.kick_ok_t is None and t > self.kick_t + 0.5:
            if abs(wrap(s[2])) < 0.1 and abs(s[3]) < 1.0 and abs(s[0]) < 0.2:
                self.kick_ok_t = t - self.kick_t          # time to settle again
        if abs(wrap(s[2])) < 0.1 and abs(s[3]) < 2.0:
            self.run_cap += TS
            if self.res["first"] is None:
                self.res["first"] = t
            if self.run_cap >= 3.0 and self.res["sustained"] is None:
                self.res["sustained"] = t
            if (self.knock_t is not None and self.recover_t is None and self.run_cap >= 3.0
                    and t > self.knock_t + 3.0):
                self.recover_t = t - 3.0 - self.knock_t
        else:
            self.run_cap = 0.0

    def _nmpc(self, ticks, xv):
        p = self.policy
        if self.fail_inject is not None:                 # test harness: bursts of QP failures
            if self.burst == 0 and self.rng.random() < self.fail_inject[0]:
                self.burst = self.rng.randint(1, self.fail_inject[1])
        if self.burst > 0:
            self.burst -= 1
            p.failures += 1; p.consec_fail += 1; p.prepared = False
            u, ok = 0.0, False
        else:
            u, ok = p.step(ticks, xv, self.n_th, self.n_filt)
        self.res["solves"] += 1
        self.res["qp_maxiter"] += p.last_qp_status == 2
        if ok:
            p.prepare()
        else:
            self.res["qp_fail"] += 1
            u = p.plan_command()   # follow the last good NMPC plan
            p.follow(u)
        if p.plan_log is not None:
            p.plan_log.mark_sent(u, True, self.k)
        return u, "NMPC"

    def finish(self) -> dict:
        self.res["hard"] = self.plant.hard_hits
        self.res["knock_t"], self.res["recover"] = self.knock_t, self.recover_t
        self.res["kick_t"], self.res["kick_settle"] = self.kick_t, self.kick_ok_t
        return self.res


def save_plan_log(policy, path, **extra):
    log = policy.plan_log if policy is not None else None
    if log is None or not path:
        return
    path = log.save(path, extra=dict(source="sim_mujoco", **extra))
    print(f"plan log: {log.n} frames ({log.dropped} dropped) -> {path}", flush=True)
    log.clear()


def run_headless(ctrl, seeds, seconds, policy, pot="real", plan_log=None):
    plant = MujocoPlant()
    out = []
    for sd in seeds:
        ep = Episode(ctrl, sd, plant, policy, pot)
        if policy is not None:
            policy.solve_ms.clear()
        for _ in range(int(seconds / TS)):
            ep.frame()
        if plan_log:
            save_plan_log(policy, f"{plan_log.removesuffix('.npz')}_seed{sd}", seed=sd,
                          seconds=seconds, pot=pot, observer=False)
        r = ep.finish()
        ms = np.array(policy.solve_ms) if (policy is not None and policy.solve_ms) else None
        out.append(r)
        sus = "none" if r["sustained"] is None else f"{r['sustained']:5.1f}s"
        extra = (f" | solve median {np.median(ms):.2f} p95 {np.percentile(ms, 95):.2f} ms, "
                 f"{r['qp_fail']} failed steps" if ms is not None else "")
        print(f"\nRESULT {ctrl:6s} seed {sd}: sustained capture {sus}, rail hits {r['hard']}, "
              f"guard trips {r['guard']}, max|x| {r['maxx'] * 100:4.1f} cm{extra}", flush=True)
    return out


def run_viewer(ctrl, seed, seconds, policy, pot="real", plan_log=None):
    import mujoco.viewer
    plant = MujocoPlant()
    ep = Episode(ctrl, seed, plant, policy, pot)
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
    save_plan_log(policy, plan_log, seed=seed, seconds=ep.k * TS, pot=pot, observer=False)


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
    ap.add_argument("--ctrl", choices=["nmpc"], default="nmpc",
                    help="pure NMPC")
    ap.add_argument("--seeds", type=int, default=6, help="headless: number of random starts")
    ap.add_argument("--seed-start", type=int, default=0, help="headless: first random start")
    ap.add_argument("--seed", type=int, default=0, help="viewer: random start")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--pot", choices=["real", "ideal"], default="real",
                    help="real: blind spot, glitches and scale error of the hardware pot")
    ap.add_argument("--viewer", action="store_true", help="watch one run in the MuJoCo viewer")
    ap.add_argument("--upright-reseed", action=argparse.BooleanOptionalAction, default=True,
                    help="NMPC: try an upright initial guess when the warm start still swings away")
    ap.add_argument("--validate", action="store_true", help="compare the plant with the ODE model")
    ap.add_argument("--plan-log", metavar="PATH",
                    help="nmpc: record every plan (nmpc/plan_log.py) to PATH_seed<N>.npz "
                         "(viewer: PATH.npz); only reads the solver. "
                         "Runs on the pot and the firmware's rate filter")
    a = ap.parse_args()
    if a.validate:
        return validate()
    if a.plan_log and a.ctrl != "nmpc":
        ap.error("--plan-log needs --ctrl nmpc")
    policy = None
    if a.ctrl == "nmpc":
        from nmpc_policy import NmpcPolicy
        log = dict(plan_instrumentation=True, plan_log_frames=int(a.seconds / TS) + 200) if a.plan_log else {}
        policy = NmpcPolicy(upright_reseed=a.upright_reseed, **log)
    if a.viewer:
        run_viewer(a.ctrl, a.seed, a.seconds, policy,
                   a.pot, a.plan_log)
        # acados and the MuJoCo viewer crash against each other during normal
        # interpreter teardown; everything is printed, so leave directly.
        sys.stdout.flush()
        os._exit(0)
    results = {}
    for c in [a.ctrl]:
        results[c] = run_headless(c, range(a.seed_start, a.seed_start + a.seeds), a.seconds,
                                  policy, a.pot,
                                  a.plan_log)
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
