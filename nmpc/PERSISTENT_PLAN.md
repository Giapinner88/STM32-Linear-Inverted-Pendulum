# Historical method and validation record

Scripts named below may now be archived; see root README.md. The record is kept
as provenance, not as a list of current commands. Firmware failure behavior changed
with the NMPC-only cleanup; see docs/firmware-architecture.md.

# Persistent-plan RTI-NMPC: current design record

## Active decision

The acados multiple-shooting iterate is **not shifted between 5 ms telemetry
frames**. The existing unshifted iterate remains the warm start for the next
SQP-RTI step. This is independent of the last accepted plan used as a fallback:
that plan is time-addressed by `NmpcPolicy.plan_command()` and is not remeshed
onto the solver grid.

The general persistent-plan alpha line search is inactive. The targeted upright
warm-start repair below compares two solved candidates online. Other merit
calculations remain instrumentation only.

## Upright warm-start repair (2026-10-15)

The hardware log saved at 16:25 (`nmpc_persistent_hw_20261015_trial2`) overwrote
the earlier run under that name. All 2744 captured frames in this newer run
still predicted a departure beyond 90 degrees before returning near upright
at the terminal node. There were no solver failures or reseeds during capture.
At the recorded 10.0007 s state, the same OCP solved from an upright initial
guess reduced cost from about 8.887 to 0.031 and kept its predicted angle near
upright. This is fixed-state solver evidence, not hardware validation of a fix.

`NmpcPolicy._repair_upright_plan` now tries this alternate initialization after
a successful normal solve. It is enabled by default; `--no-upright-reseed` in
the hardware and simulation runners restores the previous runtime behavior.
One acados OCP continues to control swing-up, capture, and balance. No observer,
LQR, new actuator model, objective weights, or horizon changes are introduced.

The AI-selected development thresholds live in `UPRIGHT_REPAIR`: current
angle within 0.15 rad of the nearest upright, angular rate <=1.5 rad/s,
position <=0.16 m, speed <=0.5 m/s, and the existing plan leaves that upright
by at least pi/2. There must be at least 1.5 ms left in the existing 3 ms frame
budget. Attempts have a 40-active-successful-frame cooldown (nominally 0.2 s).

For the same current state z0 and nearest upright equilibrium ze, initialize
`X_i = ze + (z0 - ze) exp(-t_i/0.3)` and
`V_i = (X_{i+1,u} - X_{i,u}) / dt_i`, clearing slack and dual guesses.
This is only a multiple-shooting initial guess, not a separate controller or
a claimed feasible rollout. At most two full RTI steps repair it, stopping
between solves at the frame deadline. A single solve cannot be preempted, so
this is a bounded-attempt policy, not a hard real-time guarantee.

Accept only finite candidates with nonlinear equality residual <=1e-3,
all node positions within the existing soft rail limit (0.19 m), all node
angles within 0.45 rad of the same upright, original hard duty/rate bounds,
and acados objective at least 1e-6 lower than the saved candidate at the same
z0. Otherwise restore the saved x/u/sl/su/pi/lam. Ordinary preparation is
rebuilt afterwards. The normal command commit also updates the last-good
fallback plan from whichever candidate survived this comparison.

Schema 3 logs add per-frame `upright_attempts`/`upright_accepts` and save the
enable flag and thresholds in metadata. `X_prep` is still the original normal
RTI warm start, not the alternate guess. `solve_ms` includes the extra solves;
feedback-only timing does not. The console prints accepted/attempted repairs.
Existing schema 2 logs remain readable but do not describe this new behavior.
The general persistent-plan alpha search remains off.

Physical validation is pending. In particular, a low-cost, dynamically
consistent predicted plan does not establish actuator-model accuracy or
robustness to sensing errors. Use a new timestamped hardware log for every run.

### Bounded validation performed

`check_upright_reseed.py` reuses the built solver and seven recorded states:
all seven accepted upright plans, reducing objective from 8.88-8.91 to
0.020-0.089. Forced rejection restored every saved primal/dual array exactly.
Expired deadlines and the disabled option left the iterate untouched. The
actual step/prepare path updated the fallback plan and schema 3 counters.

MuJoCo, real pot, no observer, 30 s episodes, deadlines disabled in both arms
(`nmpc_policy.STEP_BUDGET_S = float('inf')`), same existing model and sensor:

| Seed | Repair off: longest capture | Repair on: longest capture | On: upright at 30 s |
| --- | ---: | ---: | --- |
| 0 | 7.565 s | 18.305 s | yes |
| 1 | 7.725 s | 26.800 s | yes |

All four episodes had zero rail hits/guard events. Each enabled episode accepted
one upright repair. Seed 0 with an additional +8 rad/s knock three seconds after
the first sustained capture recovered in 1.605 s (confirmed by a subsequent
3 s hold), accepted two repairs, and stayed upright at the end; zero rail hits.
These are small development checks, not a robustness campaign.

With the real 3 ms attempt budget restored, seed 1 with plan instrumentation
held for 26.8 s, accepted one repair, and had zero rail hits. The repair frame
took 2.242 ms; complete step p50/p95 were 0.484/1.367 ms. An unrelated swing-up
frame took 120.537 ms while its timed solver took 2.129 ms and repair was not
attempted. This host scheduling/runtime outlier remains unexplained; the
3 ms budget is not a hard real-time guarantee, and simulated physics does not
reproduce that wall-clock stall. Raw log:
`logs/nmpc_upright_validation_20261015_seed1_deadline3ms.npz`.

Hardware snapshot provenance: input log saved 2026-10-15 16:25:10,
git metadata `3109e35-dirty`, NPZ SHA256
`b8603c39586cdaad65fea34c6b8a3844cf9f33581b981697b4182f1fa8238150`.

## Why fixed-grid multiple-shooting shift was rejected

The OCP grid is 15 x 20 ms followed by 25 x 80 ms while telemetry arrives every
5 ms. Moving a piecewise-constant command-rate trajectory by 5 ms puts old
switching times inside the new intervals. A single constant command rate cannot
reproduce that shifted trajectory on the unchanged grid.

The experimental `track` shift reduced the mismatch between the measured state
and prepared node 0, but inserted larger dynamics defects into the incumbent.
One RTI step did not remove them reliably.

Deterministic 30 s MuJoCo runs with deadlines disabled gave:

| Arm | Pot model, no observer, 5 scenarios x 3 seeds | Ideal sensing, capture / recovery after knock |
| --- | ---: | --- |
| Unshifted warm start | 15/15 captured | 5.3-5.4 s / 0.8-1.0 s |
| Fixed-grid `track` shift | 0/15 captured | 5.7-6.0 s / 2.6 s |

With the real-pot model, shift also increased failures, reseeds and tail resets.
Their per-episode ranges changed from 31-219 / 86-396 / 18-45 to
176-296 / 485-566 / 61-85, respectively.
The prepared-incumbent median angular-rate defect rose from about 0.002 rad/s to
0.17 rad/s; candidate p99 angular-rate defect rose from 1.8 to 5.0 rad/s.
Preparation time rose from 126 us to 364 us, but closed-loop degradation—not
runtime—is the reason for rejection.

These are simulation results only. The shift was not run on hardware.
With the unshifted default, 30/30 instrumentation on/off comparisons were
bit-for-bit identical. The rejected transformation is preserved in the external
cleanup archive as `nmpc/rejected_plan_shift.py`; see this directory's `README.md`
for the archive location and recovery information.

## Merit instrumentation

`PlanLog` records the candidate, its multipliers, and the prepared incumbent.
`analyze_plans.py` reports objective, dynamics-defect and hard-bound-violation
components after replacing node 0 of both plans with the same current state:

$$
X_{b,0}=X_{c,0}=\hat{x}_k.
$$

This supports offline inspection of an exact-penalty-style merit without fixing
penalty weights prematurely:

$$
\Phi(P)=J(P)+\rho_d\sum_i\lVert X_{i+1}-F_i(X_i,V_i)\rVert_1
        +\rho_g\,\mathrm{viol}_{hard}(P).
$$

The components are logged separately. Choosing $\rho_d$, $\rho_g$, an acceptance
tolerance, or enabling the general merit-based alpha search requires a separate
evidence-based decision; it is not the targeted upright repair above.
