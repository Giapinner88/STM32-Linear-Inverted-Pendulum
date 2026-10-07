import sys, math, numpy as np
sys.argv=[sys.argv[0]]
import sim_mujoco as sm
from nmpc_policy import NmpcPolicy
pol = NmpcPolicy(); plant = sm.MujocoPlant()
seed = int(__import__("os").environ.get("SEED", "3"))
ep = sm.Episode("nmpc", seed, plant, pol)
print("t     x[cm]  angle[deg]  e(energy)  u    note")
for k in range(int(26/sm.TS)):
    ep.frame()
    if k % 100 == 0:
        s = plant.state(); e = s[3]**2/(2*37.8) + math.cos(s[2]) - 1
        print(f"{k*sm.TS:5.1f} {s[0]*100:+6.1f} {math.degrees(sm.wrap(s[2])):+8.1f}  {e:+6.2f}  {ep.u_cmd:+.2f}  hits={plant.hard_hits}")
print(ep.finish())
