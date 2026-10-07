import sys, numpy as np, cartpole_nmpc as m, sim_closed_loop as sim
cfgs = {
 "tf 6 s N60 dt.10 robust+LM": dict(N=60, dt=0.10, sim_steps=3, robust=True, lm=1e-2),
 "tf 6 s N80 dt.075 robust+LM": dict(N=80, dt=0.075, sim_steps=2, robust=True, lm=1e-2),
}
which = sys.argv[1]
kw = cfgs[which]
sv = m.build(name="cp6", x_lim=0.15, rail_quad=1e8, rail_lin=1e6, **kw)
rs = [sim.run(sv, sd, T=22.0) for sd in range(6)]
ok = [r["sustained"] for r in rs if r["sustained"] is not None]
print(f"RESULT {which:30s} sustained {len(ok)}/6 at {np.median(ok) if ok else float('nan'):4.1f}s | hard hits/run {np.mean([r['hard'] for r in rs]):.1f} guard trips/run {np.mean([r['guard'] for r in rs]):.1f} max|x| {max(r['maxx'] for r in rs)*100:.1f}cm | RTI {1000*np.mean([np.mean(r['times']) for r in rs]):.2f} ms | QP fails {sum(r['bad'] for r in rs)}", flush=True)
