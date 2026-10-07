#!/usr/bin/env python3
"""Parallel NMPC tuning on the MuJoCo plant (real-time supervisor off, so the
numbers reflect the controller and not the machine's timing)."""
import multiprocessing as mp
import os
import re
import sys

import numpy as np

SEED_START = int(os.environ.get("SEED_START", "0"))
SEED_COUNT = int(os.environ.get("SEED_COUNT", "8"))
CHUNK = 4
SECONDS = 30.0


def evaluate(job):
    name, cfg, weights, seeds = job
    import sim_mujoco as sm
    from nmpc_policy import NmpcPolicy
    safe = "tune_" + re.sub(r"_+", "_", "".join(c if c.isalnum() else "_" for c in name)).strip("_")
    pol = NmpcPolicy(config=cfg, weights=weights, name=safe)
    plant = sm.MujocoPlant()
    rows = []
    for sd in seeds:
        pol.solve_ms.clear()
        ep = sm.Episode("nmpc", sd, plant, pol)
        for _ in range(int(SECONDS / sm.TS)):
            ep.frame()
        rows.append(ep.finish())
    return name, rows


def summarise(name, rows):
    ok = [r["sustained"] for r in rows if r["sustained"] is not None]
    return dict(name=name, ok=len(ok), n=len(rows),
                med=float(np.median(ok)) if ok else float("nan"),
                worst=max(ok) if ok else float("nan"),
                hits=float(np.mean([r["hard"] for r in rows])),
                qp=float(np.mean([r["qp_fail"] for r in rows])),
                maxx=max(r["maxx"] for r in rows) * 100)


def run(jobs):
    ctx = mp.get_context("spawn")
    seeds = list(range(SEED_START, SEED_START + SEED_COUNT))
    tasks = [(n, c, w, seeds[i:i + CHUNK]) for n, c, w in jobs for i in range(0, len(seeds), CHUNK)]
    merged: dict = {}
    with ctx.Pool(min(len(tasks), os.cpu_count() or 4)) as pool:
        for name, rows in pool.imap_unordered(evaluate, tasks):
            merged.setdefault(name, []).extend(rows)
    results = [summarise(n, r) for n, r in merged.items()]
    results.sort(key=lambda r: (-r["ok"], r["med"] if r["med"] == r["med"] else 1e9))
    print(f"{'config':26s} {'capture':>8s} {'median':>7s} {'worst':>6s} {'rail hits':>9s} {'QP fails':>8s} {'max|x|':>7s}")
    for r in results:
        print(f"{r['name']:26s} {r['ok']:>4d}/{r['n']:<3d} {r['med']:6.1f}s {r['worst']:5.1f}s {r['hits']:9.2f} {r['qp']:8.1f} {r['maxx']:6.1f}cm")
    return results


if __name__ == "__main__":
    from nmpc_policy import CONFIG
    base_w = {}
    jobs = [
        ("N60 dt.080 (4.8s)", {"dt": 0.08, "sim_steps": 2}, base_w),
        ("N60 dt.100 (6.0s) old", {}, base_w),
    ]
    run([(n, c, w) for n, c, w in jobs])
