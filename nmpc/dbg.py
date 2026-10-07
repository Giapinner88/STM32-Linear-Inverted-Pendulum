import sys, numpy as np, cartpole_nmpc as m
from acados_template import AcadosOcpSolver
s = m.build(rebuild=False)
x0 = np.array([0.0, 0.0, np.pi + 0.02, 0.0, 0.0])
def init(s):
    s.reset()
    for i in range(61): s.set(i, "x", x0)
    for i in range(60): s.set(i, "u", np.zeros(1))
    s.set(0,"lbx",x0); s.set(0,"ubx",x0)
init(s)
codes = []
for i in range(30):
    codes.append(s.solve())
print("status sequence (first 30 RTI iterations):", codes)
print("cost", s.get_cost(), "u0", s.get(0,"u"), "x1", s.get(1,"x"))
