# NMPC-only firmware contract

## Ownership and changes (2026-10-15)

Specified: remove LQR/blend; one NMPC for all phases, no observer.
Inherited: UART layout and FULL=4, 5 ms frame, pot/rate filtering, capture
thresholds, slew limits, rail protection, calibration and manual controls.
AI-selected replacement: expired laptop commands stop the H-bridge instead of
switching to a local controller. This failure-policy change needs hardware
revalidation after explicit flashing; it does not change the solver.

- `src/pendulum_app.c`: sampling/filtering, command freshness, USER sequence,
  command selection, rail guard and motor output.
- `lib/CartpoleControl/src/drive_state.*`: capture hysteresis and slew only.
- `lib/Protocol/src/comms.*`: UART; IDLE=0 and FULL=4; IDs 1..3 retired.
- `include/cartpole_config.h`: timing, capture, slew and hardware limits.
- `nmpc/nmpc_policy.py`: unchanged NMPC/warm-start/last-plan fallback.
- `tools/laptop_control.py`: NMPC only; legacy `--nmpc` is a no-op alias.

## Runtime and boundaries

Pot/encoder → firmware filtering → UART → NMPC → FULL duty → slew → rail guard
→ motor. Capture selects only the inherited slew limit/display, not a controller.

USER calibration must finish before FULL drives the automatic sequence.
The inherited age test allows <= 30/5 frames; expiry is on the next frame.
On expiry `StopDrive()` zeros output immediately and resets slew/capture.
There is no ramp or recovery controller. A fresh finite FULL command resumes
if the USER sequence is still running. IDLE, non-finite and retired mode
commands revoke the previous command immediately. USER stop, calibration
inhibition, homing and jog keep their priority. Homing/jog are manual
maintenance actions, never NMPC fallback.

A solver failure with a healthy link still uses `plan_command()`, the last
good NMPC plan. This is distinct from losing UART commands.

## Compatibility and verification boundary

Sensing, OCP model/weights, solver timing, warm start, last-plan fallback and
simulation capture scoring are unchanged. Healthy-link pure NMPC is intended
to remain equivalent. LQR/blend baselines cannot be run here.
Historical CSV/NPZ logs are retained; their older firmware behavior must not
be attributed to the new firmware.

Compilation and CLI checks are code/workflow checks, not closed-loop evidence.
Hardware behavior and watchdog stopping are not yet physically revalidated.
The board is not flashed automatically. Old LQR method/runbook/test documents
are archived rather than kept as current instructions.

Cleanup checks on 2026-10-15: PlatformIO build passed (Flash 22008 bytes,
RAM 1924 bytes); hardware and simulation launcher `--help`, shell syntax and
`git diff --check` passed. No simulation episode or hardware run was performed
for this cleanup. Source deletions are intentional; no commit was created.
