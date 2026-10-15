"""Per-frame record of the NMPC's plans, for the persistent-plan analysis (Phase 1).

Enabled with NmpcPolicy(plan_instrumentation=True); the archived analyze_plans.py reads what
save() writes. The log only reads from the solver, never writes to it, and does not
change a command. All buffers are allocated and touched up front. On the critical path
(between telemetry and command) it only keeps a few Python values; they are written to
the buffers, and the plan the solver just produced is read, in prepare(), after the
command went out, or, when no prepare() follows that frame (a stale frame, a failed
step), at the start of the next solve before the solver is touched.

One row per idle(), step() or trial() call:
  kind          0 idle, 1 step, 2 trial
  t_begin/t_end perf_counter() at the start and end of the call [s]
  z             state handed to the solver (delay-compensated, u = command in place)
  prepared_in   the RTI preparation for this frame had been done
  reseed_in     a rollout re-seed was already pending at the start of the frame
  after_reset   first row after NmpcPolicy.reset()
  ok            the frame produced a plan (status 0 after retries / rescue)
  status, qp_status   NmpcPolicy.last_status / last_qp_status
  retries, reseeds, jumps, tail_resets, overruns   counter increments in this frame
  rescued       re-seeded from a rollout after a failed step in this frame
  rail_start    rail limit the solver had at node 0 (rail_limits() gives every node)
  n_solves, solve_ms   RTI steps in this frame and their total time
  plan_age      NmpcPolicy.plan_age after the frame (frames the fallback has followed)
  u_cmd, u_src  command the policy committed: 1 solver, 2 plan_command(), 3 idle zero
  sent, u_sent, telem_frame   from the caller (mark_sent): whether it went out, -1 unknown
  has_plan      X and V hold the plan the frame produced (the RTI candidate)
  X, V          (N+1, NX) states and (N,) command rates of that plan, as the solver has them
  pi, lam       its multipliers (dynamics; bounds and slacks), solver.get_flat order
  has_prep      X_prep and V_prep hold the prepared incumbent: the unshifted iterate the
                frame's first QP was built around (a rollout when the frame re-seeded)
"""

from __future__ import annotations

import json
import subprocess
import time

import numpy as np

import cartpole_nmpc as m

SCHEMA = 3


class PlanLog:
    IDLE, STEP, TRIAL = 0, 1, 2
    SRC_NONE, SRC_SOLVER, SRC_PLAN, SRC_IDLE = 0, 1, 2, 3

    def __init__(self, policy, frames: int) -> None:
        import nmpc_policy as pol
        from persistent_plan import resolved_config
        self.p = policy
        self.cap = cap = int(frames)
        N = policy.n

        def buf(shape, dtype, fill):
            a = np.empty(shape, dtype)
            a.fill(fill)                # touch every page now, not in the control loop
            return a

        self.cols = dict(
            kind=buf(cap, np.int8, -1), t_begin=buf(cap, np.float64, np.nan),
            t_end=buf(cap, np.float64, np.nan), z=buf((cap, m.NX), np.float64, np.nan),
            prepared_in=buf(cap, np.bool_, False), reseed_in=buf(cap, np.bool_, False),
            after_reset=buf(cap, np.bool_, False), ok=buf(cap, np.bool_, False),
            status=buf(cap, np.int16, -1), qp_status=buf(cap, np.int16, -1),
            retries=buf(cap, np.int16, 0), reseeds=buf(cap, np.int16, 0),
            jumps=buf(cap, np.int16, 0), tail_resets=buf(cap, np.int16, 0),
            overruns=buf(cap, np.int16, 0), rescued=buf(cap, np.bool_, False),
            upright_attempts=buf(cap, np.int16, 0), upright_accepts=buf(cap, np.int16, 0),
            rail_start=buf(cap, np.float64, np.nan), n_solves=buf(cap, np.int16, 0),
            solve_ms=buf(cap, np.float64, np.nan), plan_age=buf(cap, np.int32, -1),
            u_cmd=buf(cap, np.float64, np.nan), u_src=buf(cap, np.int8, 0),
            sent=buf(cap, np.int8, -1), u_sent=buf(cap, np.float64, np.nan),
            telem_frame=buf(cap, np.int64, -1), has_plan=buf(cap, np.bool_, False),
            X=buf((cap, N + 1, m.NX), np.float64, np.nan), V=buf((cap, N), np.float64, np.nan),
            pi=buf((cap, policy.solver.get_flat("pi").size), np.float64, np.nan),
            lam=buf((cap, policy.solver.get_flat("lam").size), np.float64, np.nan),
            has_prep=buf(cap, np.bool_, False),
            X_prep=buf((cap, N + 1, m.NX), np.float64, np.nan),
            V_prep=buf((cap, N), np.float64, np.nan),
        )
        for k, v in self.cols.items():
            setattr(self, k, v)
        self.meta = dict(
            schema=SCHEMA, ts=pol.TS, delay_frames=pol.DELAY_FRAMES, plan_nodes=pol.PLAN_NODES,
            x_lim=policy.x_lim, funnel_rate=pol.FUNNEL_RATE, node_t=policy.node_t.tolist(),
            model=dict(TAU=m.TAU, KV=m.KV, W2=m.W2, G2=m.G2, DAMP=m.DAMP, SUPPLY_V=m.SUPPLY_V,
                       U_MAX=m.U_MAX, V_MAX=m.V_MAX, MODEL_BIAS=m.MODEL_BIAS),
            resolved=resolved_config(policy.solver), capacity=cap, warm_shift="off",
            upright_reseed=policy.upright_reseed, upright_repair=dict(pol.UPRIGHT_REPAIR))
        self.clear()

    def clear(self) -> None:
        """Start over (the buffers are reused)."""
        self.n = 0
        self.row = -1               # row of the frame in progress, -1 none (or dropped)
        self.dropped = 0            # frames that did not fit
        self.unmatched_follow = 0   # follow() calls with no failed step to attach to
        self._pending = False       # the row still has to be written (capture)
        self._marked = True
        self._reset = False
        self._b = self._e = self._f = self._p = None

    # --- hooks called by NmpcPolicy; begin, end and followed are on the critical path and
    # only keep Python values, capture writes them out -------------------------------
    def begin(self, kind: int, z) -> None:
        if self._pending:
            self.capture()          # nothing has touched the solver since that frame
        self._marked = True
        self._b = self._e = self._f = self._p = None
        if self.n >= self.cap:
            self.row = -1
            self.dropped += 1
            return
        p = self.p
        self.row = self.n
        self.n += 1
        self._b = (kind, time.perf_counter(), z, p.prepared, p.reseed, self._reset, p.retries,
                   p.reseeds, p.jumps, p.tail_resets, p.overruns, len(p.solve_ms),
                   p.upright_attempts, p.upright_accepts)
        self._reset = False
        self._marked = False
        self._pending = True

    def prep(self, snap) -> None:
        """The iterate (NmpcPolicy._snapshot) the frame's first QP is built around."""
        if self.row >= 0 and self._p is None:
            self._p = snap

    def end(self, ok: bool, u) -> None:
        if self.row >= 0:
            self._e = (time.perf_counter(), ok, u)

    def capture(self) -> None:
        """Write out the frame in progress and, if it produced one, its plan. Called once the
        command is out (prepare), else before the next solve; the solver is unchanged."""
        if not self._pending:
            return
        self._pending = False
        p, r = self.p, self.row
        (kind, t0, z, self.prepared_in[r], self.reseed_in[r], self.after_reset[r], *c) = self._b
        self.kind[r], self.t_begin[r], self.z[r] = kind, t0, z
        if self._p is not None:     # snapshot arrays are never written to after they are taken
            self.X_prep[r] = self._p[0].reshape(-1, m.NX)
            self.V_prep[r] = self._p[1]
            self.has_prep[r] = True
        if self._e is None:         # the call did not return normally
            return
        t1, ok, u = self._e
        self.t_end[r], self.ok[r] = t1, ok
        if u is not None:
            self.u_cmd[r] = u
            self.u_src[r] = self.SRC_SOLVER if kind == self.STEP else self.SRC_IDLE
        elif self._f is not None:
            self.u_cmd[r] = self._f
            self.u_src[r] = self.SRC_PLAN
        self.status[r] = p.last_status
        self.qp_status[r] = p.last_qp_status
        self.retries[r] = p.retries - c[0]
        self.reseeds[r] = p.reseeds - c[1]
        self.jumps[r] = p.jumps - c[2]
        self.tail_resets[r] = p.tail_resets - c[3]
        self.overruns[r] = p.overruns - c[4]
        self.rescued[r] = p.last_rescued and kind == self.STEP
        self.rail_start[r] = p.rail_start
        self.n_solves[r] = len(p.solve_ms) - c[5]
        self.solve_ms[r] = sum(p.solve_ms[c[5]:])
        self.upright_attempts[r] = p.upright_attempts - c[6]
        self.upright_accepts[r] = p.upright_accepts - c[7]
        self.plan_age[r] = p.plan_age
        if ok:
            s = p.solver
            self.X[r] = s.get_flat("x").reshape(-1, m.NX)
            self.V[r] = s.get_flat("u")
            self.pi[r] = s.get_flat("pi")
            self.lam[r] = s.get_flat("lam")
            self.has_plan[r] = True

    def followed(self, u: float) -> None:
        """The caller sends u after a failed step (plan_command())."""
        e = self._e
        if (self.row >= 0 and self._b[0] == self.STEP and e is not None and not e[1]
                and self._f is None):
            self._f = u
            if not self._pending:   # already written out
                self.u_cmd[self.row], self.u_src[self.row] = u, self.SRC_PLAN
        else:
            self.unmatched_follow += 1

    def note_reset(self) -> None:
        self._reset = True

    # --- called by the caller -------------------------------------------------------
    def mark_sent(self, u: float, sent: bool, telem_frame: int = -1) -> None:
        """What happened to this frame's command: sent (u) or dropped (stale frame). Call it
        after the command went out."""
        r = self.row
        if r < 0 or self._marked:
            return
        self._marked = True
        self.sent[r] = 1 if sent else 0
        self.u_sent[r] = u if sent else np.nan
        self.telem_frame[r] = telem_frame

    def save(self, path: str, extra: dict | None = None) -> str:
        if not path.endswith(".npz"):
            path += ".npz"
        self.capture()
        n = self.n
        meta = dict(self.meta, frames=n, dropped=self.dropped,
                    unmatched_follow=self.unmatched_follow, saved=time.strftime("%Y-%m-%d %H:%M:%S"),
                    **(extra or {}))
        try:
            meta["git"] = subprocess.run(["git", "describe", "--always", "--dirty"], capture_output=True,
                                         text=True, timeout=5).stdout.strip()
        except Exception:
            pass
        np.savez_compressed(path, meta=json.dumps(meta), **{k: v[:n] for k, v in self.cols.items()})
        return path
