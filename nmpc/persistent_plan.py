"""Offline plan arithmetic for the persistent-plan RTI-NMPC: time-address a plan,
roll it out, score it, and inspect an alpha ladder against an RTI candidate.

Nothing in this module changes the online warm start. A fixed-grid multiple-shooting
shift was tested and rejected because it introduced dynamics defects at the 20/80 ms
stage boundaries; see nmpc/PERSISTENT_PLAN.md. The model and the cost residual are the owners in cartpole_nmpc (dynamics,
cost_residual); weights, slack penalties, bounds, time grid, cost scaling and integrator
settings come from the resolved acados OCP (resolved_config), so a plan is scored the way
acados scores it. Checked against acados in self_check(). The objective is

  J = sum_{i<N} c_i (1/2 r_i' W_i r_i + zl s_l,i + 1/2 Zl s_l,i^2 + zu s_u,i + 1/2 Zu s_u,i^2)
      + c_N (1/2 r_N' W_e r_N + the same slack terms with the _e penalties)

with r = y - yref and c = solver_options.cost_scaling: acados multiplies each stage's
whole cost, slack penalties included, by its time step (terminal by 1;
ocp_nlp_cost_nls.c). Node 0 has no slack: the rail is soft only at nodes 1..N, where its
limit is the funnel nmpc_policy.rail_limits() gives for the frame's rail_start. The
slack of a plan is the smallest one its states allow, which is the one acados' QP
picks for fixed states.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import casadi as ca
import numpy as np

import cartpole_nmpc as m

RAIL = 0          # state index of the cart position, the only soft-bounded state
CMD = 4           # state index of the command u


def _rk4(x, v, h, n: int):
    """n classic RK4 steps over h under the constant input v (acados ERK with 4 stages)."""
    hs = h / n
    for _ in range(n):
        k1 = m.dynamics(x, v)
        k2 = m.dynamics(x + hs / 2 * k1, v)
        k3 = m.dynamics(x + hs / 2 * k2, v)
        k4 = m.dynamics(x + hs * k3, v)
        x = x + hs / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def _l(a) -> list:
    return np.asarray(a, dtype=float).ravel().tolist()


def resolved_config(solver) -> dict:
    """What the plan arithmetic needs from the solver, as plain lists (JSON-serialisable).
    Raises NotImplementedError for OCP features the evaluator does not reproduce."""
    ocp = solver.acados_ocp
    so, c, k, d = ocp.solver_options, ocp.cost, ocp.constraints, ocp.dims
    extra = [f for f in ("nh", "nh_e", "nh_0", "ng", "ng_e", "nsbu", "nsg", "nsg_e", "nsh",
                         "nsh_e", "nphi", "nphi_e", "nz", "np") if int(getattr(d, f, 0) or 0)]
    kinds = {so.integrator_type, c.cost_type_0, c.cost_type, c.cost_type_e}
    if extra or kinds != {"ERK", "NONLINEAR_LS"}:
        raise NotImplementedError(f"plan evaluator: unsupported OCP features {extra} {kinds}")
    cfg = dict(
        name=ocp.model.name,
        time_steps=_l(so.time_steps), cost_scaling=_l(so.cost_scaling),
        sim_method_num_steps=[int(s) for s in np.atleast_1d(so.sim_method_num_steps)],
        sim_method_num_stages=[int(s) for s in np.atleast_1d(so.sim_method_num_stages)],
        W_0=np.asarray(c.W_0, float).tolist(), W=np.asarray(c.W, float).tolist(),
        W_e=np.asarray(c.W_e, float).tolist(),
        yref_0=_l(c.yref_0), yref=_l(c.yref), yref_e=_l(c.yref_e),
        zl=_l(c.zl), zu=_l(c.zu), Zl=_l(c.Zl), Zu=_l(c.Zu),
        zl_e=_l(c.zl_e), zu_e=_l(c.zu_e), Zl_e=_l(c.Zl_e), Zu_e=_l(c.Zu_e),
        idxbu=[int(i) for i in k.idxbu], lbu=_l(k.lbu), ubu=_l(k.ubu),
        idxbx=[int(i) for i in k.idxbx], lbx=_l(k.lbx), ubx=_l(k.ubx),
        idxsbx=[int(i) for i in k.idxsbx], lsbx=_l(k.lsbx), usbx=_l(k.usbx),
        idxbx_e=[int(i) for i in k.idxbx_e], lbx_e=_l(k.lbx_e), ubx_e=_l(k.ubx_e),
        idxsbx_e=[int(i) for i in k.idxsbx_e], lsbx_e=_l(k.lsbx_e), usbx_e=_l(k.usbx_e),
        hessian_approx=so.hessian_approx, nlp_solver_type=so.nlp_solver_type,
    )
    # The soft bounds are the rail (state 0) and nothing else; acados indexes soft bounds
    # within idxbx. Slacks have lower bound 0 (lsbx) and no upper bound on use (usbx).
    if ([cfg["idxbx"][i] for i in cfg["idxsbx"]] != [RAIL]
            or [cfg["idxbx_e"][i] for i in cfg["idxsbx_e"]] != [RAIL]
            or any(cfg["lsbx"]) or any(cfg["lsbx_e"])):
        raise NotImplementedError("plan evaluator: soft bounds other than the rail")
    return cfg


@dataclass
class PlanEval:
    X: np.ndarray                 # (N+1, NX) states from the rollout
    V: np.ndarray                 # (N,) command rates
    J: float                      # acados objective (inf if the rollout is not finite)
    J_slack: float                # its rail-slack part
    finite: bool                  # every state and input finite
    viol_u: float                 # largest |u| beyond its bound, nodes 1..N-1 (hard)
    viol_v: float                 # largest |v| beyond its bound, intervals 0..N-1 (hard)
    first_violation: int          # first node with a hard violation or a non-finite value, -1 none
    rail_excess: float            # largest rail slack, nodes 1..N (soft, priced in J_slack)
    rail_excess_e: float          # rail slack at the terminal node
    hard_valid: bool              # finite and both hard violations within the tolerance
    stage: np.ndarray             # (N+1,) each node's share of J (least squares + slack)

    @property
    def max_violation(self) -> float:
        return max(self.viol_u, self.viol_v)


class PlanModel:
    """Rollout and objective of a plan on the solver's grid, built once from resolved_config."""

    def __init__(self, cfg: dict) -> None:
        self.cfg = cfg
        steps = np.array(cfg["time_steps"])
        self.N = N = len(steps)
        self.node_t = np.concatenate([[0.0], np.cumsum(steps)])
        self.h = steps
        if set(cfg["sim_method_num_stages"]) != {4}:
            raise NotImplementedError("plan evaluator: ERK with 4 stages (RK4) only")
        nsub = cfg["sim_method_num_steps"]
        nsub = nsub * N if len(nsub) == 1 else nsub
        c = np.array(cfg["cost_scaling"])
        W0, W, We = (np.array(cfg[k]) for k in ("W_0", "W", "W_e"))
        r0, r, re = (np.array(cfg[k]) for k in ("yref_0", "yref", "yref_e"))

        def stage(x, v, i):
            y, _ = m.cost_residual(x, v)
            e = y - (r0 if i == 0 else r)
            return c[i] * 0.5 * ca.dot(e, ca.mtimes(W0 if i == 0 else W, e))

        def terminal(x):
            _, ye = m.cost_residual(x, ca.SX.zeros(1))
            return c[N] * 0.5 * ca.dot(ye - re, ca.mtimes(We, ye - re))

        z = ca.SX.sym("z", m.NX)
        V = ca.SX.sym("V", N)
        x, xs, ls = z, [z], []
        for i in range(N):
            ls.append(stage(x, V[i], i))
            x = _rk4(x, V[i], steps[i], nsub[i])
            xs.append(x)
        ls.append(terminal(x))
        self._f = ca.Function("plan_rollout", [z, V], [ca.horzcat(*xs).T, ca.vertcat(*ls)])
        # The plan as the solver holds it (multiple shooting): defects and cost at its own states.
        Xs = ca.SX.sym("X", N + 1, m.NX)
        nxt = [_rk4(Xs[i, :].T, V[i], steps[i], nsub[i]) for i in range(N)]
        lsm = [stage(Xs[i, :].T, V[i], i) for i in range(N)] + [terminal(Xs[N, :].T)]
        self._g = ca.Function("plan_shooting", [Xs, V],
                              [Xs[1:, :] - ca.horzcat(*nxt).T, ca.vertcat(*lsm)])

        self.c = c
        sb = [cfg["idxbx"][i] for i in cfg["idxsbx"]]
        hard = [j for j, ix in enumerate(cfg["idxbx"]) if ix not in sb]
        self.hard_x = [(cfg["idxbx"][j], cfg["lbx"][j], cfg["ubx"][j]) for j in hard]
        self.hard_x_e = [(cfg["idxbx_e"][j], cfg["lbx_e"][j], cfg["ubx_e"][j])
                         for j, ix in enumerate(cfg["idxbx_e"])
                         if ix not in [cfg["idxbx_e"][s] for s in cfg["idxsbx_e"]]]
        if cfg["idxbu"] != [0]:
            raise NotImplementedError("plan evaluator: one bounded input expected")
        self.v_lo, self.v_hi = cfg["lbu"][0], cfg["ubu"][0]
        self.soft = (cfg["zl"][0], cfg["zu"][0], cfg["Zl"][0], cfg["Zu"][0])
        self.soft_e = (cfg["zl_e"][0], cfg["zu_e"][0], cfg["Zl_e"][0], cfg["Zu_e"][0])
        self.x_lim = cfg["ubx"][cfg["idxbx"].index(RAIL)]

    def rollout(self, z0, V) -> tuple[np.ndarray, np.ndarray]:
        """States at the nodes from z0 under the command rates V, and each stage's least-squares cost."""
        X, ls = self._f(np.asarray(z0, float), np.asarray(V, float).ravel())
        return np.array(X), np.array(ls).ravel()

    def slack(self, X, lims) -> tuple[np.ndarray, np.ndarray]:
        """Smallest rail slacks (lower, upper) the states allow, nodes 0..N (node 0: none)."""
        p = X[:, RAIL]
        sl = np.maximum(0.0, -lims - p)
        su = np.maximum(0.0, p - lims)
        sl[0] = su[0] = 0.0
        return sl, su

    def slack_cost(self, X, lims) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Each node's rail-slack penalty (scaled as acados does), and the slacks."""
        N = self.N
        sl, su = self.slack(X, lims)
        zl, zu, Zl, Zu = self.soft
        pen = self.c[:N] * (zl * sl[:N] + 0.5 * Zl * sl[:N] ** 2 + zu * su[:N] + 0.5 * Zu * su[:N] ** 2)
        zl, zu, Zl, Zu = self.soft_e
        pen_e = self.c[N] * (zl * sl[N] + 0.5 * Zl * sl[N] ** 2 + zu * su[N] + 0.5 * Zu * su[N] ** 2)
        return np.append(pen, pen_e), sl, su

    def shooting(self, X, V, lims, z0=None) -> tuple[np.ndarray, float]:
        """The plan as the solver holds it, without a rollout: defects X_{i+1} - F_i(X_i, V_i)
        (N, NX) and the acados objective at its own states (smallest slacks). If z0 is
        given, node 0 is replaced first so two iterates can be compared at the same
        measured initial condition."""
        X = np.array(X, dtype=float, copy=True)
        if z0 is not None:
            X[0] = np.asarray(z0, float)
        d, ls = self._g(X, np.asarray(V, float).ravel())
        return np.array(d), float(np.sum(np.array(ls)) + np.sum(self.slack_cost(X, lims)[0]))

    def shooting_hard_violation(self, X, V) -> tuple[float, float]:
        """(largest, L1 sum) hard-bound violation at the multiple-shooting values.
        The rail is soft and remains priced in shooting(); node 0 is equality-constrained
        by the live state and is therefore not counted here."""
        X = np.asarray(X, float)
        V = np.asarray(V, float).ravel()
        if not (np.all(np.isfinite(X)) and np.all(np.isfinite(V))):
            return math.inf, math.inf
        pieces = [np.maximum(0.0, np.maximum(V - self.v_hi, self.v_lo - V))]
        for ix, lo, hi in self.hard_x:
            x = X[1:self.N, ix]
            pieces.append(np.maximum(0.0, np.maximum(x - hi, lo - x)))
        for ix, lo, hi in self.hard_x_e:
            x = X[self.N:self.N + 1, ix]
            pieces.append(np.maximum(0.0, np.maximum(x - hi, lo - x)))
        flat = np.concatenate([p.ravel() for p in pieces])
        return float(np.max(flat, initial=0.0)), float(np.sum(flat))

    def evaluate(self, z0, V, lims, tol: float = 1e-3) -> PlanEval:
        """Roll V out from z0 and score it. lims: rail limit per node (rail_limits())."""
        V = np.asarray(V, float).ravel()
        X, ls = self.rollout(z0, V)
        N = self.N
        finite = bool(np.all(np.isfinite(X)) and np.all(np.isfinite(V)))
        pens, sl, su = self.slack_cost(X, lims)
        J_slack = float(np.sum(pens))
        J = float(np.sum(ls) + J_slack) if finite else math.inf
        if not math.isfinite(J):
            finite, J = False, math.inf

        bad = np.zeros(N + 1, bool)
        bad[:N] |= ~np.isfinite(V)
        bad |= ~np.all(np.isfinite(X), axis=1)
        dv = np.maximum(0.0, np.maximum(V - self.v_hi, self.v_lo - V))
        bad[:N] |= dv > tol
        du = np.zeros(N + 1)
        for ix, lo, hi in self.hard_x:              # nodes 1..N-1
            x = X[1:N, ix]
            du[1:N] = np.maximum(du[1:N], np.maximum(x - hi, lo - x))
        for ix, lo, hi in self.hard_x_e:            # node N
            du[N] = max(du[N], X[N, ix] - hi, lo - X[N, ix])
        bad |= du > tol
        viol_u = float(np.nanmax(du)) if finite else math.inf
        viol_v = float(np.nanmax(dv)) if finite else math.inf
        first = int(np.argmax(bad)) if bad.any() else -1
        rail = np.maximum(sl, su)
        return PlanEval(X=X, V=V, J=J, J_slack=J_slack, finite=finite, viol_u=viol_u,
                        viol_v=viol_v, first_violation=first,
                        rail_excess=float(np.max(rail[1:])) if finite else math.inf,
                        rail_excess_e=float(rail[N]) if finite else math.inf,
                        hard_valid=finite and first < 0, stage=ls + pens)


def shift_inputs(u_nodes, V_acc, node_t, delta: float, u0: float, v_lo: float, v_hi: float,
                 mode: str = "track") -> tuple[np.ndarray, int]:
    """Input sequence of an accepted plan delta seconds after it was made, on the same
    (non-uniform) grid, starting from the command u0 that is actually in place.

    track: the command follows the accepted command trajectory, which is linear between
           the plan's node commands u_nodes (what plan_command() interpolates): v_i takes
           u from its value at node i to u_acc(t_{i+1} + delta), within the slew limit.
           The first interval absorbs any offset between u0 and u_acc(delta).
    hold:  v_i is the accepted rate at t_i + delta (piecewise constant), u0 offset kept.

    Past the accepted horizon the command eases to 0 at the slew limit, as in
    nmpc_policy.rollout(). Returns (V, first_tail): intervals from first_tail on reach past
    the accepted horizon and are appended, not optimised (untrusted)."""
    node_t = np.asarray(node_t, float)
    u_nodes = np.asarray(u_nodes, float)
    V_acc = np.asarray(V_acc, float).ravel()
    N, T = len(node_t) - 1, float(node_t[-1])
    h = np.diff(node_t)
    t1 = node_t[1:] + delta
    late = t1 > T + 1e-12
    first_tail = int(np.argmax(late)) if late.any() else N
    rate = min(-v_lo, v_hi)
    if mode == "track":
        target = np.interp(t1, node_t, u_nodes)
        if late.any():
            uT = float(u_nodes[-1])
            target[late] = np.copysign(np.maximum(0.0, abs(uT) - rate * (t1[late] - T)), uT)
        V = np.diff(np.concatenate([[float(u0)], target])) / h
        if V.min() >= v_lo and V.max() <= v_hi:
            return V, first_tail
        u = float(u0)                   # the slew limit binds: follow the targets within it
        for i in range(N):
            V[i] = min(max((target[i] - u) / h[i], v_lo), v_hi)
            u += h[i] * V[i]
        return V, first_tail
    if mode != "hold":
        raise ValueError(mode)
    V, u = np.empty(N), float(u0)
    for i in range(N):
        s = node_t[i] + delta
        v = float(V_acc[np.searchsorted(node_t, s, "right") - 1]) if s < T - 1e-12 else -u / h[i]
        V[i] = min(max(v, v_lo), v_hi)
        u += h[i] * V[i]
    return V, first_tail


def line_search(model: PlanModel, z0, V_base, V_cand, lims, alphas=(1.0, 0.5, 0.25),
                eps_abs: float = 1e-8, eps_rel: float = 1e-6, tol: float = 1e-3):
    """Score the base plan and V_base + alpha (V_cand - V_base), each rolled out from z0.
    Returns (alpha, evals, margin): the largest alpha whose plan is hard-valid and has
    J <= J_base - margin, margin = eps_abs + eps_rel |J_base|; alpha 0 keeps the base.
    evals maps every alpha, 0 included, to its PlanEval."""
    V_base = np.asarray(V_base, float).ravel()
    dV = np.asarray(V_cand, float).ravel() - V_base
    base = model.evaluate(z0, V_base, lims, tol)
    margin = eps_abs + eps_rel * abs(base.J) if math.isfinite(base.J) else 0.0
    evals, chosen = {0.0: base}, 0.0
    for a in sorted(alphas, reverse=True):
        e = model.evaluate(z0, V_base + a * dV, lims, tol)
        evals[a] = e
        if chosen == 0.0 and e.hard_valid and e.J <= base.J - margin:
            chosen = a
    return chosen, evals, margin


def plan_command_at(u_nodes, node_t, plan_nodes: int, delta: float, u_max: float) -> float:
    """nmpc_policy.NmpcPolicy.plan_command() for a plan delta seconds old: its command
    trajectory over the first plan_nodes nodes, 0 once that runs out."""
    times = np.asarray(node_t)[:plan_nodes]
    if delta >= times[-1]:
        return 0.0
    return float(np.clip(np.interp(delta, times, np.asarray(u_nodes)[:plan_nodes]), -u_max, u_max))


def self_check(policy, z0, V, rail_start: float) -> dict:
    """Compare this module with acados on one plan: roll V out from z0, load it into the
    policy's solver and read back acados' objective (get_cost) and the dynamics residual of
    its integrator (res_eq, recomputed). Overwrites the solver's iterate and linearisation;
    use a policy that is not controlling anything."""
    from nmpc_policy import rail_limits
    model = PlanModel(resolved_config(policy.solver))
    lims = rail_limits(policy.x_lim, rail_start, model.node_t)
    ev = model.evaluate(z0, V, lims)
    s = policy.solver
    policy.rail_start = -1.0
    policy._rail(np.array([rail_start - 0.01, 0, 0, 0, 0]) if rail_start > policy.x_lim
                 else np.zeros(5))
    s.set(0, "lbx", np.asarray(z0, float))
    s.set(0, "ubx", np.asarray(z0, float))
    s.set_flat("x", ev.X.ravel())
    s.set_flat("u", ev.V.ravel())
    sl, su = model.slack(ev.X, lims)
    for i in range(1, model.N + 1):
        s.set(i, "sl", np.array([sl[i]]))
        s.set(i, "su", np.array([su[i]]))
    J_acados = float(s.get_cost())
    b = float(s.get_residuals(recompute=True)[1])
    policy.reseed = True
    policy.prepared = False
    return dict(J=ev.J, J_acados=J_acados, rel_err=abs(ev.J - J_acados) / max(1e-12, abs(J_acados)),
                J_slack=ev.J_slack, max_defect=b, rail_excess=ev.rail_excess)
