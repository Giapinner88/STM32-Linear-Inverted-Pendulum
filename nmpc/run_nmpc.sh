#!/usr/bin/env bash
# Pure RTI-NMPC on the laptop: no LQR, no supervisor. A frame where the QP
# solver fails sends u = 0 and re-seeds. The STM32 only measures and actuates;
# its own 30 ms command timeout is the only safety net behind this.
# Usage: nmpc/run_nmpc.sh [seconds] [extra laptop_control.py args]
# Then press USER on the board; press it again (or Ctrl+C here) to stop.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
SECONDS_TO_RUN="${1:-90}"
shift || true
exec "$ROOT/.venv-nmpc/bin/python" "$ROOT/tools/laptop_control.py" --nmpc --seconds "$SECONDS_TO_RUN" "$@"
