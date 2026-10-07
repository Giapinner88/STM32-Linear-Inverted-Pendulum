"""NMPC swing-up policy for tools/laptop_control.py --nmpc.

The solver keeps running on the hanging pendulum while the board is idle, so
it is already converged when the automatic sequence starts. Frame by frame it
turns the board's telemetry into the NMPC state, compensates the link delay
with the commands still in flight, and returns the next command. If the QP
solver fails, a frame falls back to the vendor swing-up law.
"""

from __future__ import annotations

import time
from collections import deque

import numpy as np

import cartpole_nmpc as m
from sim_closed_loop import MPT, UP, plant_step

CONFIG = dict(N=60, dt=0.08, sim_steps=2, robust=True, lm=1e-2,
              x_lim=0.15, rail_quad=1e8, rail_lin=1e6)
DELAY_FRAMES = 2          # measured link round trip is ~8.5 ms
TS = 0.005


class NmpcPolicy:
    def __init__(self, rebuild: bool = True, config: dict | None = None,
                 weights: dict | None = None, name: str = "cp6") -> None:
        cfg = dict(CONFIG)
        cfg.update(config or {})
        self.solver = m.build(name=name, rebuild=rebuild, weights=weights, **cfg)
        self.n = cfg["N"]
        self.in_flight = deque([0.0] * DELAY_FRAMES, maxlen=DELAY_FRAMES)
        self.u_prev = 0.0
        self.solve_ms: list[float] = []
        self.failures = 0
        self.last_status = 0        # acados return code of the latest solve
        self.last_qp_status = 0     # status the QP solver reported for it
        self.reseed = True
        self.prepared = False      # linearisation for the next feedback is done
        self.feedback_ms: list[float] = []

    def _state(self, x_ticks, x_vel_ticks_s, theta_app, theta_vel):
        s = [-x_ticks * MPT, -x_vel_ticks_s * MPT, UP - theta_app, -theta_vel]
        for u in self.in_flight:
            for _ in plant_step(s, u):
                pass
        return np.array([*s, self.u_prev])

    def _note(self, status: int) -> None:
        self.last_status = int(status)
        try:
            self.last_qp_status = int(np.atleast_1d(self.solver.get_stats("qp_stat"))[-1])
        except Exception:
            self.last_qp_status = -1

    def _solve(self, x0: np.ndarray, iterations: int = 1, reseed_iters: int = 8) -> int:
        if self.reseed:
            for i in range(self.n + 1):
                self.solver.set(i, "x", x0)
            for i in range(self.n):
                self.solver.set(i, "u", np.zeros(1))
            self.reseed = False
            iterations = max(iterations, reseed_iters)
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        self.solver.options_set("rti_phase", 0)
        self.prepared = False
        status = 0
        for _ in range(iterations):
            t0 = time.perf_counter()
            status = self.solver.solve()
            self.solve_ms.append((time.perf_counter() - t0) * 1000.0)
            self._note(status)
        return status

    def _run(self, x0: np.ndarray, reseed_iters: int) -> int:
        if self.prepared and not self.reseed:
            # RTI feedback phase only: the linearisation was prepared after the
            # previous command went out, so just the QP solve is on the path.
            self.solver.set(0, "lbx", x0)
            self.solver.set(0, "ubx", x0)
            self.solver.options_set("rti_phase", 2)
            t0 = time.perf_counter()
            status = self.solver.solve()
            ms = (time.perf_counter() - t0) * 1000.0
            self.solve_ms.append(ms)
            self.feedback_ms.append(ms)
            self.prepared = False
            self._note(status)
            return status
        return self._solve(x0, 1, reseed_iters=reseed_iters)

    def idle(self, x_ticks, x_vel_ticks_s, theta_app, theta_vel) -> None:
        """Keep the solution warm while nothing is being commanded."""
        self.u_prev = 0.0
        self.in_flight.extend([0.0] * DELAY_FRAMES)
        x0 = self._state(x_ticks, x_vel_ticks_s, theta_app, theta_vel)
        if self._run(x0, reseed_iters=8) not in (0, 2):
            self.reseed = True  # never leave a failed solution as the warm start

    def prepare(self) -> None:
        """RTI preparation phase for the next frame. Call right after the
        command has been written, while waiting for the next telemetry."""
        if self.reseed:
            return
        self.solver.options_set("rti_phase", 1)
        self.solver.solve()
        self.prepared = True

    def reset(self) -> None:
        """Forget everything from an earlier run, including the QP solver's
        warm start, so a run does not depend on what was solved before it."""
        self.solver.reset()
        self.reseed = True
        self.prepared = False
        self.u_prev = 0.0
        self.in_flight.extend([0.0] * DELAY_FRAMES)
        self.failures = 0

    def interrupt(self) -> None:
        """NMPC lost control: re-seed from the live state at the next trial."""
        self.reseed = True

    def follow(self, u: float) -> None:
        """Another controller sent u; keep the command history consistent. The
        solution is kept warm by trial(), so NMPC can resume without a cold start."""
        self.u_prev = u
        self.in_flight.append(u)
        self.prepared = False

    def trial(self, x_ticks, x_vel_ticks_s, theta_app, theta_vel) -> float:
        """Solve once without commanding; returns the solve time in ms."""
        x0 = self._state(x_ticks, x_vel_ticks_s, theta_app, theta_vel)
        n = len(self.solve_ms)
        status = self._solve(x0, 1, reseed_iters=3)
        return self.solve_ms[-1] if (status in (0, 2) and len(self.solve_ms) > n) else 999.0

    def step(self, x_ticks, x_vel_ticks_s, theta_app, theta_vel):
        """Returns (command, ok). ok is False if the solver failed this frame."""
        x0 = self._state(x_ticks, x_vel_ticks_s, theta_app, theta_vel)
        status = self._run(x0, reseed_iters=3)
        if status not in (0, 2):
            self.failures += 1
            self.reseed = True
            return 0.0, False
        v0 = float(self.solver.get(0, "u")[0])
        u = float(np.clip(self.u_prev + v0 * TS, -m.U_MAX, m.U_MAX))
        self.u_prev = u
        self.in_flight.append(u)
        return u, True
