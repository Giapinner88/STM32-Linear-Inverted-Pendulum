import sys, numpy as np, cartpole_nmpc as m, sim_closed_loop as sim
sv = m.build(name="cp_trace", x_lim=0.15, rail_quad=1e5, rail_lin=1e3)
# Re-run seeds until one has a wall hit, logging the plan when it happens.
orig = sim.plant_step
for sd in range(6):
    log = []
    def spy(s, u, rng_sub=10, _log=log):
        for ev in orig(s, u, rng_sub):
            yield ev
        _log.append((s[0]/sim.MPT, s[1], u))
    sim.plant_step = spy
    r = sim.run(sv, sd, T=8.0)
    sim.plant_step = orig
    if r["hard"] or r["guard"]:
        hit = next(i for i,(x,v,u) in enumerate(log) if abs(x) > 1945)
        print(f"seed {sd}: first rail guard region at t={hit*0.005:.3f}s; x(t), v, u around it:")
        for i in range(max(0,hit-40), hit+8, 4):
            x,v,u = log[i]; print(f"  t={i*0.005:5.3f} x={x:+7.0f}tk v={v:+5.2f} m/s u={u:+.2f}")
        break
