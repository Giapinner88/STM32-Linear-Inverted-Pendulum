#!/usr/bin/env bash
# NMPC with a real-time fallback: when the solver stops keeping its 5 ms frame
# (late solves, telemetry backlog, QP failures) control drops to LQR + energy
# shaping, and returns to NMPC after it has proven fast again.
# Usage: nmpc/run_blend.sh [seconds] [extra laptop_control.py args]
# Then press USER on the board; press it again (or Ctrl+C here) to stop.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
SECONDS_TO_RUN="${1:-90}"
shift || true
exec "$ROOT/.venv-nmpc/bin/python" "$ROOT/tools/laptop_control.py" --blend --seconds "$SECONDS_TO_RUN" "$@"
