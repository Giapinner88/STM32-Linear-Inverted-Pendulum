#!/usr/bin/env bash
# LQR + energy shaping (the hybrid law) on the laptop; the STM32 only measures
# and actuates. Baseline for run_nmpc.sh and run_blend.sh.
# Usage: lqr/run_lqr.sh [seconds] [extra laptop_control.py args]
# Then press USER on the board; press it again (or Ctrl+C here) to stop.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SECONDS_TO_RUN="${1:-90}"
shift || true
PY="$ROOT/.venv-nmpc/bin/python"
[ -x "$PY" ] || PY="$HOME/.platformio/penv/bin/python"   # only needs pyserial
exec "$PY" "$ROOT/tools/laptop_control.py" --full --seconds "$SECONDS_TO_RUN" "$@"
