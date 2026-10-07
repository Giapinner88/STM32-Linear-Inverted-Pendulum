import sys, numpy as np
DC = '--no-comp' not in sys.argv
print('delay compensation:', DC)
import sys, numpy as np, cartpole_nmpc as m, sim_closed_loop as sim
configs = {
 "tf 3 s  (dt .05)":  dict(dt=0.050, sim_steps=1, x_lim=0.15, rail_quad=1e8, rail_lin=1e6),
 "tf 4 s  (dt .067)": dict(dt=4.0/60, sim_steps=2, x_lim=0.15, rail_quad=1e8, rail_lin=1e6),
 "tf 5 s  (dt .083)": dict(dt=5.0/60, sim_steps=2, x_lim=0.15, rail_quad=1e8, rail_lin=1e6),
 "tf 6 s  (dt .10)":  dict(dt=6.0/60, sim_steps=3, x_lim=0.15, rail_quad=1e8, rail_lin=1e6),
}
for name, kw in configs.items():
    sv = m.build(name="cp_sweep", **kw)
    TF = kw.get('dt', 0.025)
    rs = [sim.run(sv, sd, T=25.0, delay_comp=DC) for sd in range(12)]
    ok = [r["sustained"] for r in rs if r["sustained"] is not None]
    print(f"{name:28s} sustained {len(ok)}/12 at {np.median(ok) if ok else float('nan'):4.1f}s | hard hits/run {np.mean([r['hard'] for r in rs]):.1f} guard trips/run {np.mean([r['guard'] for r in rs]):.1f} max|x| {max(r['maxx'] for r in rs)*100:.1f}cm | RTI {1000*np.mean([np.mean(r['times']) for r in rs]):.2f} ms",flush=True)
