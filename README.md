# STM32 cart-pole — pure NMPC branch

One acados RTI NMPC controls swing-up and balance on the Wheeltec IP570.
The laptop uses pot angle and firmware rate filtering, last-good-plan fallback
and upright warm-start repair. No observer, LQR or energy-shaping controller
is included here; LQR remains on main.

## Run

From this repository, with the existing `.venv-nmpc` environment and acados:

```bash
./nmpc/run_nmpc.sh 30
```

CSV and plan logs go to `logs/`. Centre the cart and hang the pendulum at its
calibrated reference (ADC 990–1060). Press USER to calibrate/start; USER again
or Ctrl+C stops. Keep the rail clear and power disconnect reachable.

**Firmware changed:** expired laptop commands now stop the motor instead of
switching to a board controller. The board is not flashed automatically.
Before relying on this behavior, build and deliberately flash the firmware.
This cleanup has not been hardware-validated; prior logs use older firmware.

Documented commands (upload resets hardware; perform separately):

```bash
pio run
pio run -t upload
```

## Core

- `nmpc/`: OCP, online policy, MuJoCo runner and plan recording.
- `tools/laptop_control.py`: NMPC UART loop; `serial_link.py`: protocol,
  capture classifier and firmware-compatible sensor constants.
- `src/`, `include/`, `lib/`: sensing, USER sequence, motor safety and I/O.
  `drive_state.c` owns capture hysteresis and slew, not a control law.
- `sim/cartpole_identified.xml`: identified MuJoCo plant.
- `logs/`: retained evidence; `parameters.md`: calibration history.

See [NMPC details](nmpc/PERSISTENT_PLAN.md),
[firmware semantics](docs/firmware-architecture.md) and [pinout](pinout.md).

Optional analysis/check scripts and old build outputs are archived too.
Only runtime, simulation, logging and hardware documentation remain active.

Retired code and pre-edit snapshots are recoverable at
`/home/giap/Projects/STM32-NMPC-archive-20261015-5a7xa94_/nmpc_only/`.
Active code does not depend on this archive. Vendor/Cube build dependencies remain.

Hardware reference: [playfultechnology/pid-invertedpendulum](https://github.com/playfultechnology/pid-invertedpendulum).
Original hardware/firmware by Wheeltec / MiniBalance.
