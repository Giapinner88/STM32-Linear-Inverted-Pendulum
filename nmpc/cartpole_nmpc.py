"""RTI-NMPC for the cart-pole, built with acados.

Model (reference frame: x positive right, theta from upright, u>0 pushes right):
  x'' = (12*u/KV - x') / TAU                       motor + cart, identified
  th'' = W2*sin(th) - G2*x''*cos(th) - DAMP*th'    pendulum, identified
The command u is a state and its rate v the input, so the board's slew limit
(0.1125 per 5 ms) is part of the model instead of an after-the-fact surprise.
Parameters come from logs/run1.csv and logs/run2.csv, see parameters.md.
"""

from __future__ import annotations

import os

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

TAU, KV = 0.057, 13.8          # cart time constant [s], volts per m/s
# The planning model's pendulum is deliberately 6% stiffer than the identified one
# (identified: omega^2 = 37.8 1/s^2, coupling 3.85 1/m, both from logs, +-4% scatter
# between logs). In MuJoCo the NMPC captures the pendulum reliably only when its model
# omega^2 is not below the plant's: with the identified value, a plant 4% lighter
# (0/8 captures) already defeats it; at +6% it captures 12/12 from -8% to +4%.
# See model_bias.py in the exploratory-script archive (see root README.md).
MODEL_BIAS = 1.06
W2 = 37.8 * MODEL_BIAS         # pendulum omega^2 [1/s^2]
G2 = 3.85 * MODEL_BIAS         # cart-acceleration coupling [1/m]
DAMP = 0.15                    # pendulum viscous damping [1/s]
SUPPLY_V = 12.0
U_MAX = 0.95                   # firmware clamps at 6900/7199 = 0.958
V_MAX = 0.1125 / 0.005         # slew limit of the board [1/s]
X_LIM = 0.18                   # [m]; the rail guard trips at 0.2115 m

NX, NU = 5, 1


def dynamics(x, v):
    """x = [pos, vel, theta, theta_rate, u]; v = du/dt. Works with casadi or numpy."""
    xdd = (SUPPLY_V * x[4] / KV - x[1]) / TAU
    thdd = W2 * ca.sin(x[2]) - G2 * xdd * ca.cos(x[2]) - DAMP * x[3]
    return ca.vertcat(x[1], xdd, x[3], thdd, v)


def energy(x):
    """Nondimensional pendulum energy: 0 upright and still, -2 hanging still."""
    return x[3] ** 2 / (2.0 * W2) + ca.cos(x[2]) - 1.0


def cost_residual(x, v):
    """Least-squares residuals of the cost: stage y(x, v) and terminal y_e(x)."""
    y = ca.vertcat(x[0], x[1], energy(x), ca.sin(x[2]), 1.0 - ca.cos(x[2]), x[3], x[4], v[0])
    return y, y[:6]


def build(N: int = 60, dt: float = 0.025, name: str = "cartpole",
          weights: dict | None = None, rebuild: bool = True,
          x_lim: float = X_LIM, rail_quad: float = 2e3, rail_lin: float = 0.0,
          sim_steps: int = 1, robust: bool = False, lm: float = 0.0,
          term_mult: float = 10.0, qp_iter_max: int = 30, qp_warm_start: int = 0,
          qp_tol: float | None = None, solver_opts: dict | None = None,
          x_comfort: float | None = None, comfort_quad: float = 1e3,
          comfort_lin: float = 1e2, time_steps: list | None = None) -> AcadosOcpSolver:
    w = dict(x=1.0, xd=0.1, e=30.0, s=5.0, c=5.0, thd=0.05, u=0.05, v=5e-4)
    w.update(weights or {})
    sx = ca.SX.sym("x", NX)
    sv = ca.SX.sym("v", NU)

    model = AcadosModel()
    model.name = name
    model.x, model.u = sx, sv
    model.f_expl_expr = dynamics(sx, sv)
    model.xdot = ca.SX.sym("xdot", NX)
    model.f_impl_expr = model.xdot - model.f_expl_expr

    ocp = AcadosOcp()
    ocp.model = model
    if time_steps is not None:
        # Non-uniform grid: fine steps first (the catch), coarse later (the swing-up).
        N = len(time_steps)
        ocp.solver_options.N_horizon = N
        ocp.solver_options.time_steps = np.array(time_steps, dtype=float)
        ocp.solver_options.tf = float(np.sum(time_steps))
    else:
        ocp.solver_options.N_horizon = N
        ocp.solver_options.tf = N * dt

    y, y_e = cost_residual(sx, sv)
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr = y
    ocp.model.cost_y_expr_e = y_e
    ocp.cost.yref = np.zeros(8)
    ocp.cost.yref_e = np.zeros(6)
    stage = [w["x"], w["xd"], w["e"], w["s"], w["c"], w["thd"], w["u"], w["v"]]
    ocp.cost.W = np.diag(stage)
    ocp.cost.W_e = np.diag([term_mult * s for s in stage[:6]])

    ocp.constraints.x0 = np.array([0.0, 0.0, np.pi, 0.0, 0.0])
    ocp.constraints.idxbu = np.array([0])
    ocp.constraints.lbu = np.array([-V_MAX])
    ocp.constraints.ubu = np.array([V_MAX])
    ocp.constraints.idxbx = np.array([0, 4])
    ocp.constraints.lbx = np.array([-x_lim, -U_MAX])
    ocp.constraints.ubx = np.array([x_lim, U_MAX])
    ocp.constraints.idxbx_e = np.array([0])
    ocp.constraints.lbx_e = np.array([-x_lim])
    ocp.constraints.ubx_e = np.array([x_lim])
    # Soft rail limit so a start or disturbance outside it stays feasible.
    ocp.constraints.idxsbx = np.array([0])
    ocp.constraints.idxsbx_e = np.array([0])
    soft = [(rail_lin, rail_quad)]
    if x_comfort is not None:
        # Two-level rail: x_lim is the guard (heavy penalty, just inside the board's motor
        # cut-off) and x_comfort a moderately penalised zone. Outside the comfort zone the
        # plan pays a little, so it can still borrow travel to catch the pendulum; with a
        # single heavy limit the cheapest plan was often to let the pendulum fall.
        model.con_h_expr = sx[0]
        model.con_h_expr_e = sx[0]
        ocp.constraints.lh = np.array([-x_comfort])
        ocp.constraints.uh = np.array([x_comfort])
        ocp.constraints.lh_e = np.array([-x_comfort])
        ocp.constraints.uh_e = np.array([x_comfort])
        ocp.constraints.idxsh = np.array([0])
        ocp.constraints.idxsh_e = np.array([0])
        soft.append((comfort_lin, comfort_quad))
    for attr, k in (("zl", 0), ("zu", 0), ("Zl", 1), ("Zu", 1)):
        vals = np.array([s[k] for s in soft])
        setattr(ocp.cost, attr, vals)
        setattr(ocp.cost, attr + "_e", vals)

    so = ocp.solver_options
    so.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    so.qp_solver_cond_N = max(2, N // 4)
    so.hessian_approx = "GAUSS_NEWTON"
    so.integrator_type = "ERK"
    so.sim_method_num_stages = 4
    if time_steps is not None:
        so.sim_method_num_steps = np.array([max(1, int(np.ceil(h / 0.04))) for h in time_steps])
    else:
        so.sim_method_num_steps = sim_steps
    so.nlp_solver_type = "SQP_RTI"
    so.qp_solver_iter_max = qp_iter_max
    so.qp_solver_warm_start = qp_warm_start
    if qp_tol is not None:
        so.qp_solver_tol_stat = so.qp_solver_tol_eq = so.qp_solver_tol_ineq = so.qp_solver_tol_comp = qp_tol
    so.globalization = "FIXED_STEP"
    for key, val in (solver_opts or {}).items():   # e.g. qp_solver_cond_N, as_rti_level
        setattr(so, key, val)
    if robust and "hpipm_mode" not in (solver_opts or {}):
        so.hpipm_mode = "ROBUST"
    if lm > 0.0:
        so.levenberg_marquardt = lm

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "generated")
    os.makedirs(out, exist_ok=True)
    ocp.code_export_directory = os.path.join(out, name + "_code")
    return AcadosOcpSolver(ocp, json_file=os.path.join(out, name + ".json"),
                           generate=rebuild, build=rebuild, verbose=False)
