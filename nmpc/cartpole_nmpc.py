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
W2 = 37.8                      # pendulum omega^2 [1/s^2]
G2 = 3.85                      # cart-acceleration coupling [1/m]
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


def build(N: int = 60, dt: float = 0.025, name: str = "cartpole",
          weights: dict | None = None, rebuild: bool = True,
          x_lim: float = X_LIM, rail_quad: float = 2e3, rail_lin: float = 0.0,
          sim_steps: int = 1, robust: bool = False, lm: float = 0.0,
          term_mult: float = 10.0) -> AcadosOcpSolver:
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
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * dt

    y = ca.vertcat(sx[0], sx[1], energy(sx), ca.sin(sx[2]), 1.0 - ca.cos(sx[2]),
                   sx[3], sx[4], sv[0])
    y_e = y[:6]
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
    for attr, val in (("zl", rail_lin), ("zu", rail_lin), ("Zl", rail_quad), ("Zu", rail_quad)):
        setattr(ocp.cost, attr, np.array([val]))
        setattr(ocp.cost, attr + "_e", np.array([val]))

    so = ocp.solver_options
    so.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    so.qp_solver_cond_N = max(2, N // 4)
    so.hessian_approx = "GAUSS_NEWTON"
    so.integrator_type = "ERK"
    so.sim_method_num_stages = 4
    so.sim_method_num_steps = sim_steps
    so.nlp_solver_type = "SQP_RTI"
    so.qp_solver_iter_max = 30
    so.globalization = "FIXED_STEP"
    if robust:
        so.hpipm_mode = "ROBUST"
    if lm > 0.0:
        so.levenberg_marquardt = lm

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "generated")
    os.makedirs(out, exist_ok=True)
    ocp.code_export_directory = os.path.join(out, name + "_code")
    return AcadosOcpSolver(ocp, json_file=os.path.join(out, name + ".json"),
                           generate=rebuild, build=rebuild, verbose=False)
