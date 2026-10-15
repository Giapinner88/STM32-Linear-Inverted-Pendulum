"""NMPC swing-up policy for tools/laptop_control.py --nmpc.

The solver keeps running on the hanging pendulum while the board is idle, so
it is already converged when the automatic sequence starts. Frame by frame it
turns the board's telemetry into the NMPC state, compensates the link delay
with the commands still in flight, and returns the next command. Only sane QP
steps are kept (NmpcPolicy._robust); a step that still fails is solved again
from a model rollout, and if that fails too the caller follows the last good
plan (plan_command).
"""

from __future__ import annotations

import math
import time
from collections import deque

import numpy as np

import cartpole_nmpc as m
from sim_closed_loop import GUARD, MPT, UP, plant_step

# Horizon: 15 x 20 ms then 25 x 80 ms (2.3 s). The long part is needed for the swing-up;
# the unstable pendulum makes long horizons ill-conditioned (4.8 s: QP failures on 21% of
# replayed hardware frames 5-45 deg from upright). The fine start is for the catch: started
# in MuJoCo from the 83 states where the real cart began a catch, 30 x 80 ms caught 51,
# the fine grid 58 (nmpc/catch_from_hw.py).
# Rail: soft limit at 19 cm with a moderate penalty and cart weight 10. With a stiff 16 cm
# limit the real cart parked at 16 cm and let the pendulum fall. With these settings and the
# solver safeguards below, the catch bench gives 73/83 (LQR + ES 80, with 7x more rail-guard
# trips) and a knock-down in MuJoCo is recovered in 1.1 s (LQR + ES 2.9 s, worst 8.6 s).
# qp_solver_mu0 1e6 (HPIPM's initial barrier) gave the fewest QP failures on this grid.
DEFAULT_WEIGHTS = {"x": 10.0, "xd": 0.4}
CONFIG = dict(time_steps=[0.02] * 15 + [0.08] * 25, robust=True, lm=1e-2, qp_iter_max=100,
              solver_opts={"qp_solver_mu0": 1e6},
              x_lim=0.19, rail_quad=1e4, rail_lin=1e2)
# A QP step that fails or is rejected (see NmpcPolicy._robust) is undone and solved again
# with other initial barrier values. The retry is a full RTI step (phase 0): retrying the
# feedback phase alone is not allowed, acados keeps statistics for one feedback per
# preparation and would write past them.
RETRY_MU0 = (1e5, 1e7, 1e4)
DELAY_FRAMES = 2          # measured link round trip is ~8.5 ms
TS = 0.005

# When the cart already stands beyond the soft rail limit, the first nodes cannot satisfy it
# and HPIPM failed there (MINSTEP, e.g. 51 frames in a row at x = 20 cm in a hardware log).
# The limit then becomes a funnel: |x0| + 1 cm at the start, closing at FUNNEL_RATE.
FUNNEL_RATE = 0.3          # [m/s]
ITERATE = ("x", "u", "sl", "su", "pi", "lam")   # what _snapshot saves
# A state this far from the one the iterate was built for (a knock or sensor jump) is
# re-seeded at once: in MuJoCo every knock (+8 rad/s) cost three failed QPs before.
# Between two frames the rate changes by at most ~0.7 rad/s and the angle by ~0.08 rad.
JUMP_RATE, JUMP_ANGLE = 2.0, 0.15
# Time a frame may spend on retries and re-seeding; past it the frame gives up and the caller
# follows the last good plan. A QP that runs into its iteration limit takes ~1.5 ms, so a
# chain of them took up to 15 ms on replayed hardware logs, three frames late.
STEP_BUDGET_S = 0.003
# The far end of the 2.3 s plan can drift off over successive RTI steps (rate of 1e5 rad/s
# at the last node in a replayed log) while the first nodes still look fine. A plan whose
# tail leaves this range, which no real motion reaches, is re-seeded at the next frame.
TAIL_RATE, TAIL_X = 25.0, 0.35
# Nodes of the last good plan that plan_command() follows after a failed solve (0.18 s).
PLAN_NODES = 10

# Near upright, RTI can keep the old swing-away-and-return branch indefinitely.
# Try a second initial guess of the SAME OCP, never a separate control law.
UPRIGHT_REPAIR = dict(angle=0.15, rate=1.5, position=0.16, velocity=0.5,
                      excursion=np.pi / 2, seed_tau=0.3, iterations=2,
                      cooldown_frames=40, reserve_s=0.0015,
                      max_angle=0.45, dynamics_tol=1e-3, cost_tol=1e-6)


def rail_limits(x_lim: float, start: float, node_t) -> np.ndarray:
    """Soft rail limit at each node: x_lim, or the funnel from start (see FUNNEL_RATE)."""
    return np.maximum(x_lim, start - FUNNEL_RATE * np.asarray(node_t))


def rollout(x0, node_t):
    """States at the node times and the input (command rate) per interval, from x0, with the
    command easing to 0 at the slew limit: a trajectory of the planning model to start the
    solver from. RK4, steps of at most 20 ms."""
    a_u, w2, g2, tau, damp = m.SUPPLY_V / m.KV, m.W2, m.G2, m.TAU, m.DAMP

    def f(x, xd, th, om, u):
        xdd = (a_u * u - xd) / tau
        return xd, xdd, om, w2 * math.sin(th) - g2 * xdd * math.cos(th) - damp * om

    x, xd, th, om, u = (float(c) for c in x0)
    X, U = [[x, xd, th, om, u]], []
    for i in range(len(node_t) - 1):
        t, t_end, u_start = node_t[i], node_t[i + 1], u
        while t < t_end - 1e-9:
            h = min(0.02, t_end - t)
            v = min(max(-u / h, -m.V_MAX), m.V_MAX)
            k1 = f(x, xd, th, om, u)
            k2 = f(x + h / 2 * k1[0], xd + h / 2 * k1[1], th + h / 2 * k1[2], om + h / 2 * k1[3], u + h / 2 * v)
            k3 = f(x + h / 2 * k2[0], xd + h / 2 * k2[1], th + h / 2 * k2[2], om + h / 2 * k2[3], u + h / 2 * v)
            k4 = f(x + h * k3[0], xd + h * k3[1], th + h * k3[2], om + h * k3[3], u + h * v)
            x += h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
            xd += h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
            th += h / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
            om += h / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])
            u += h * v
            t += h
        X.append([x, xd, th, om, u])
        U.append([(u - u_start) / (t_end - node_t[i])])
    return np.array(X), np.array(U)


class NmpcPolicy:
    def __init__(self, rebuild: bool = True, config: dict | None = None,
                 weights: dict | None = None, name: str = "cp6",
                 plan_instrumentation: bool = False, plan_acceptance: str = "off",
                 plan_log_frames: int = 24000, upright_reseed: bool = True) -> None:
        """plan_instrumentation records every solve into self.plan_log (nmpc/plan_log.py)
        for offline analysis; it only reads the solver and never changes a command.
        plan_acceptance is the persistent-plan line search, not implemented yet: "off".
        The acados multiple-shooting iterate is deliberately left unshifted: remeshing it
        by one 5 ms frame introduced defects at the fixed 20/80 ms stage boundaries and
        made one-step RTI markedly worse (nmpc/PERSISTENT_PLAN.md)."""
        if plan_acceptance != "off":
            raise ValueError("plan_acceptance: only 'off' exists (Phase 1 is instrumentation only)")
        cfg = dict(CONFIG)
        cfg.update(config or {})
        if name == "cp6":
            name = f"cp6_q{cfg['qp_iter_max']}"       # one generated solver per QP setting
        weights = {**DEFAULT_WEIGHTS, **(weights or {})}
        self.solver = m.build(name=name, rebuild=rebuild, weights=weights, **cfg)
        self.x_lim = cfg.get("x_lim", m.X_LIM)
        self.rail_start = self.x_lim      # rail limit the solver currently has at the first node
        steps = cfg.get("time_steps")
        self.n = len(steps) if steps is not None else cfg["N"]
        self.node_t = (np.concatenate([[0.0], np.cumsum(steps)]) if steps is not None
                       else np.arange(self.n + 1) * cfg["dt"])
        self.in_flight = deque([0.0] * DELAY_FRAMES, maxlen=DELAY_FRAMES)
        self.u_prev = 0.0
        self.solve_ms: list[float] = []
        self.failures = 0
        self.consec_fail = 0
        self.reseed_after = 3
        self.plan = None            # command part of the last good plan, one value per node
        self.plan_age = 0
        self.fallback_mode = "plan"  # on a failed solve: follow the last good plan ("plan") or send 0 ("zero")
        self.last_status = 0        # acados return code of the latest solve
        self.last_qp_status = 0     # status the QP solver reported for it
        self.reseed = True
        self.prepared = False      # linearisation for the next feedback is done
        self.feedback_ms: list[float] = []
        self.mu0 = float(self.solver.acados_ocp.solver_options.qp_solver_mu0)
        self.retries = 0            # QP re-solves in total
        self.last_retries = 0       # QP re-solves in the latest step
        self.reseeds = 0            # cold starts from a rollout
        self.jumps = 0              # of them because the state jumped
        self.rescues = 0            # failed steps solved again from a rollout in the same frame
        self.tail_resets = 0        # re-seeds because the plan's tail drifted off
        self.last_rescued = False
        self.deadline = None        # perf_counter() time after which a step stops retrying
        self.overruns = 0           # steps that gave up because of the deadline
        self.upright_reseed = bool(upright_reseed)
        self.upright_attempts = self.upright_accepts = 0
        self._upright_cooldown = 0
        self.plan_log = None
        if plan_instrumentation:
            from plan_log import PlanLog
            self.plan_log = PlanLog(self, plan_log_frames)

    def _state(self, x_ticks, x_vel_ticks_s, theta_app, theta_vel):
        s = [-x_ticks * MPT, -x_vel_ticks_s * MPT, UP - theta_app, -theta_vel]
        for u in self.in_flight:
            # The board's rail guard stops the motor while the cart is past it and the
            # command pushes outward: the cart coasts instead.
            blocked = (s[0] >= GUARD and u > 0) or (s[0] <= -GUARD and u < 0)
            for _ in plant_step(s, 0.0 if blocked else u):
                pass
        return np.array([*s, self.u_prev])

    def _note(self, status: int, qp_status: int) -> None:
        self.last_status = int(status)
        self.last_qp_status = int(qp_status)

    @staticmethod
    def _qp_status(solver) -> int:
        try:
            return int(np.atleast_1d(solver.get_stats("qp_stat"))[-1])
        except Exception:
            return -1

    @staticmethod
    def _snapshot(solver) -> list:
        return [solver.get_flat(f) for f in ITERATE]

    @staticmethod
    def _restore(solver, snap) -> None:
        for f, v in zip(ITERATE, snap):
            solver.set_flat(f, v)

    def _good(self, solver, status: int, qp: int) -> bool:
        """A QP step that converged, or stopped at its iteration limit, and left a plan
        within physical reach: commands within the limit (the last node has no bound),
        cart within half a metre, pendulum rate below 40 rad/s. A QP linearised far
        from the state can 'converge' to a plan with angles of 1e3 rad."""
        if status != 0 or qp not in (0, 2):
            return False
        return self._sane(solver)

    @staticmethod
    def _sane(solver, rate: float = 40.0, x: float = 0.5) -> bool:
        X = solver.get_flat("x").reshape(-1, m.NX)
        return bool(np.all(np.abs(X[:-1, 4]) <= m.U_MAX + 0.02)
                    and np.all(np.abs(X[:, 0]) <= x) and np.all(np.abs(X[:, 3]) <= rate))

    def _align(self, x0: np.ndarray) -> None:
        """Shift the iterate's angle by whole turns to the turn x0 is on: the plan is the
        same, but a linearisation turns away from x0 is not."""
        k = round((float(x0[2]) - float(self.solver.get(0, "x")[2])) / (2.0 * np.pi))
        if k:
            X = self.solver.get_flat("x").reshape(-1, m.NX)
            X[:, 2] += 2.0 * np.pi * k
            self.solver.set_flat("x", X.ravel())
            self.prepared = False

    def _robust(self, solver, phase: int, snap) -> tuple[int, int]:
        """One RTI step that is kept only if it leaves a sane plan (see _good). acados
        applies a QP step that stopped at its iteration limit as if it were solved; such a
        step had wrecked the iterate (commands of -2.75 in the plan) and every QP after it
        failed. A failed or rejected step is undone (iterate restored from snap) and, if the
        QP itself was the problem, retried as a full step with the other barrier starts; if
        none succeeds the iterate stays as it was. Returns (status, qp_status), status 0
        for an accepted step."""
        solver.options_set("rti_phase", phase)
        status, qp = solver.solve(), self._qp_status(solver)
        if self._good(solver, status, qp):
            return 0, qp
        if status == 0 and qp == 0:
            # The QP was solved, the plan is what is wrong: the same QP with another
            # barrier start gives the same plan. Only a fresh start helps (step()).
            self._restore(solver, snap)
            return 4, qp
        for mu in RETRY_MU0:
            self._restore(solver, snap)
            solver.options_set("qp_mu0", mu)
            solver.options_set("rti_phase", 0)
            status, qp = solver.solve(), self._qp_status(solver)
            self.retries += 1
            self.last_retries += 1
            if self._good(solver, status, qp):
                break
            if self._late():
                break
        solver.options_set("qp_mu0", self.mu0)
        if self._good(solver, status, qp):
            return 0, qp
        self._restore(solver, snap)
        return (status if status != 0 else 4), qp

    def _rail(self, x0: np.ndarray) -> None:
        """Funnel-shaped rail limit when the cart starts outside the soft limit (see FUNNEL_RATE)."""
        start = max(self.x_lim, abs(float(x0[0])) + 0.01)
        if abs(start - self.rail_start) < 0.002:
            return
        self.rail_start = start
        lims = rail_limits(self.x_lim, start, self.node_t)
        for i in range(1, self.n + 1):
            lim = lims[i]
            if i < self.n:
                self.solver.set(i, "lbx", np.array([-lim, -m.U_MAX]))
                self.solver.set(i, "ubx", np.array([lim, m.U_MAX]))
            else:
                self.solver.set(i, "lbx", np.array([-lim]))
                self.solver.set(i, "ubx", np.array([lim]))

    def _solve(self, x0: np.ndarray, iterations: int = 1, reseed_iters: int = 8) -> int:
        if self.reseed:
            # Start from a model rollout (command easing to 0), not from x0 copied to every
            # node: near the top that constant guess is far from any trajectory of the
            # unstable pendulum and the first QPs diverged (angles of 1e4 rad, MAXITER).
            X, U = rollout(x0, self.node_t)
            self.solver.set_flat("x", X.ravel())
            self.solver.set_flat("u", U.ravel())
            self.reseed = False
            self.reseeds += 1
            iterations = max(iterations, reseed_iters)
        else:
            self._align(x0)
        self._rail(x0)
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        self.prepared = False
        status = 0
        for _ in range(iterations):
            t0 = time.perf_counter()
            snap = self._snapshot(self.solver)
            if self.plan_log is not None:
                self.plan_log.prep(snap)
            status, qp = self._robust(self.solver, 0, snap)
            self._note(status, qp)
            self.solve_ms.append((time.perf_counter() - t0) * 1000.0)
            if status != 0 or self._late():
                break
        return status

    def _late(self) -> bool:
        return self.deadline is not None and time.perf_counter() > self.deadline

    def _repair_upright_plan(self, x0: np.ndarray) -> None:
        """Replace a swing-away branch only with a cheaper, consistent upright plan.

        Called after a successful solve, before committing its first command. The
        exponentially decaying guess is not a dynamically feasible trajectory: two
        RTI steps repair it, then the nonlinear equality residual is checked. Failed
        attempts restore every primal/dual iterate field of the original candidate.
        The normal prepare() rebuilds the linearisation after this method.
        """
        if self._upright_cooldown:
            self._upright_cooldown -= 1
            return
        if not self.upright_reseed:
            return
        c = UPRIGHT_REPAIR
        turn = round(float(x0[2]) / (2 * np.pi)) * (2 * np.pi)
        if (abs(x0[2] - turn) > c["angle"] or abs(x0[3]) > c["rate"]
                or abs(x0[0]) > c["position"] or abs(x0[1]) > c["velocity"]):
            return
        s = self.solver
        X = s.get_flat("x").reshape(-1, m.NX)
        if np.max(np.abs(X[:, 2] - turn)) < c["excursion"]:
            return
        if self.deadline is not None and self.deadline - time.perf_counter() < c["reserve_s"]:
            return
        snap = self._snapshot(s)
        base_cost = float(s.get_cost())
        self.upright_attempts += 1
        self._upright_cooldown = c["cooldown_frames"]
        accepted = False
        try:
            eq = np.zeros(m.NX)
            eq[2] = turn
            guess = eq + (x0 - eq) * np.exp(-self.node_t[:, None] / c["seed_tau"])
            s.set_flat("x", guess.ravel())
            s.set_flat("u", np.diff(guess[:, 4]) / np.diff(self.node_t))
            for field, old in zip(ITERATE[2:], snap[2:]):
                s.set_flat(field, np.zeros_like(old))
            s.options_set("rti_phase", 0)
            for _ in range(c["iterations"]):
                begin = time.perf_counter()
                status = s.solve()
                self.solve_ms.append((time.perf_counter() - begin) * 1000)
                if status != 0:
                    return
                if self._late():
                    break
            X = s.get_flat("x").reshape(-1, m.NX)
            V = s.get_flat("u")
            cost = float(s.get_cost())
            # Check the nonlinear dynamics, not just the QP status. This is after
            # feedback, never between RTI preparation and feedback.
            residual = s.get_residuals(recompute=True)
            accepted = bool(
                np.all(np.isfinite(X)) and np.all(np.isfinite(V))
                and np.all(np.isfinite(residual)) and np.isfinite(cost)
                and residual[1] <= c["dynamics_tol"]
                and np.max(np.abs(X[:, 0])) <= self.x_lim + 1e-6
                and np.max(np.abs(X[:, 2] - turn)) <= c["max_angle"]
                and np.max(np.abs(X[:-1, 4])) <= m.U_MAX + 1e-6
                and np.max(np.abs(V)) <= m.V_MAX + 1e-6
                and cost < base_cost - c["cost_tol"])
            if accepted:
                self.upright_accepts += 1
        finally:
            if not accepted:
                self._restore(s, snap)
            self.prepared = False

    def _jumped(self, x0: np.ndarray) -> bool:
        ref = self.solver.get(0, "x")
        return abs(float(x0[3] - ref[3])) > JUMP_RATE or abs(float(x0[2] - ref[2])) > JUMP_ANGLE

    def _run(self, x0: np.ndarray, reseed_iters: int) -> int:
        if not self.reseed:
            self._align(x0)
            if self._jumped(x0):
                self.reseed = True
                self.jumps += 1
        if self.prepared and not self.reseed:
            # RTI feedback phase only: the linearisation was prepared after the
            # previous command went out, so just the QP solve is on the path.
            self._rail(x0)
            self.solver.set(0, "lbx", x0)
            self.solver.set(0, "ubx", x0)
            if self.plan_log is not None:
                self.plan_log.prep(self.snap)
            t0 = time.perf_counter()
            status, qp = self._robust(self.solver, 2, self.snap)
            self._note(status, qp)
            self.prepared = False
            ms = (time.perf_counter() - t0) * 1000.0
            self.solve_ms.append(ms)
            self.feedback_ms.append(ms)
            return status
        return self._solve(x0, 1, reseed_iters=reseed_iters)

    def idle(self, x_ticks, x_vel_ticks_s, theta_app, theta_vel) -> None:
        """Keep the solution warm while nothing is being commanded."""
        self.u_prev = 0.0
        self.in_flight.extend([0.0] * DELAY_FRAMES)
        x0 = self._state(x_ticks, x_vel_ticks_s, theta_app, theta_vel)
        log = self.plan_log
        if log is not None:
            log.begin(log.IDLE, x0)
        status = self._run(x0, reseed_iters=8)
        if status != 0:
            self.reseed = True  # never leave a failed solution as the warm start
        if log is not None:
            log.end(status == 0, 0.0)

    def prepare(self) -> None:
        """RTI preparation phase for the next frame. Call right after the
        command has been written, while waiting for the next telemetry."""
        if self.plan_log is not None:
            self.plan_log.capture()   # the plan just solved, read after the command went out
        if self.reseed:
            return
        self.solver.options_set("rti_phase", 1)
        self.solver.solve()
        self.snap = self._snapshot(self.solver)   # what a failed feedback falls back to
        self.prepared = True

    def reset(self) -> None:
        """Forget everything from an earlier run, including the QP solver's
        warm start, so a run does not depend on what was solved before it."""
        if self.plan_log is not None:
            self.plan_log.capture()
            self.plan_log.note_reset()
        self.solver.reset()
        self.reseed = True
        self.prepared = False
        self.u_prev = 0.0
        self.in_flight.extend([0.0] * DELAY_FRAMES)
        self.failures = self.consec_fail = self.overruns = 0
        self.retries = self.reseeds = self.jumps = self.rescues = self.tail_resets = 0
        self.upright_attempts = self.upright_accepts = 0
        self._upright_cooldown = 0

    def interrupt(self) -> None:
        """NMPC lost control: re-seed from the live state at the next trial."""
        self.reseed = True

    def follow(self, u: float) -> None:
        """Another controller sent u; keep the command history consistent. The
        solution is kept warm by trial(), so NMPC can resume without a cold start."""
        self.u_prev = u
        self.in_flight.append(u)
        self.prepared = False
        if self.plan_log is not None:
            self.plan_log.followed(u)

    def trial(self, x_ticks, x_vel_ticks_s, theta_app, theta_vel) -> float:
        """Solve once without commanding; returns the solve time in ms."""
        x0 = self._state(x_ticks, x_vel_ticks_s, theta_app, theta_vel)
        log = self.plan_log
        if log is not None:
            log.begin(log.TRIAL, x0)
        n = len(self.solve_ms)
        self.last_retries = 0
        status = self._solve(x0, 1, reseed_iters=3)
        if log is not None:
            log.end(status == 0, None)
        return self.solve_ms[-1] if (status == 0 and len(self.solve_ms) > n) else 999.0

    def step(self, x_ticks, x_vel_ticks_s, theta_app, theta_vel):
        """Returns (command, ok). ok is False if the solver failed this frame."""
        self.deadline = time.perf_counter() + STEP_BUDGET_S
        x0 = self._state(x_ticks, x_vel_ticks_s, theta_app, theta_vel)
        log = self.plan_log
        if log is not None:
            log.begin(log.STEP, x0)
        self.last_retries = 0
        self.last_rescued = False
        reseeds = self.reseeds
        status = self._run(x0, reseed_iters=3)
        if status != 0 and self.reseeds == reseeds and not self._late():
            # Last resort within the frame: start again from a model rollout.
            self.reseed = True
            self.rescues += 1
            self.last_rescued = True
            status = self._solve(x0, 1, reseed_iters=3)
        late = self._late()
        frame_deadline = self.deadline
        self.deadline = None
        if status != 0:
            self.overruns += late
            self.failures += 1
            self.consec_fail += 1
            # A failed step leaves the last good iterate in place (see _robust); re-seed from
            # scratch only if that keeps failing.
            if self.consec_fail >= self.reseed_after:
                self.reseed = True
            if log is not None:
                log.end(False, None)    # the command sent comes from follow()
            return 0.0, False
        self.consec_fail = 0
        # Keep the same per-frame deadline for this optional second initial guess.
        self.deadline = frame_deadline
        self._repair_upright_plan(x0)
        self.deadline = None
        if not self._sane(self.solver, TAIL_RATE, TAIL_X):
            self.reseed = True          # the plan's tail is drifting off: start afresh next frame
            self.tail_resets += 1
        v0 = float(self.solver.get(0, "u")[0])
        u = float(np.clip(self.u_prev + v0 * TS, -m.U_MAX, m.U_MAX))
        self.u_prev = u
        self.in_flight.append(u)
        self.plan = np.array([self.solver.get(i, "x")[4] for i in range(min(PLAN_NODES, self.n + 1))])
        self.plan_age = 0
        if log is not None:
            log.end(True, u)
        return u, True

    def plan_command(self) -> float:
        """Command to send after a failed solve: where the last good plan says the command
        goes next, 5 ms further along each failed frame (0 once the plan runs out)."""
        if self.fallback_mode != "plan" or self.plan is None:
            return 0.0
        self.plan_age += 1
        tt = self.plan_age * TS
        times = self.node_t[:len(self.plan)]
        if tt >= times[-1]:
            return 0.0
        return float(np.clip(np.interp(tt, times, self.plan), -m.U_MAX, m.U_MAX))
