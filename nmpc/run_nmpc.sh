#!/usr/bin/env bash
# Pure NMPC with upright warm-start repair, raw pot + firmware rate filter.
# Usage: ./nmpc/run_nmpc.sh [seconds=30] [extra laptop_control.py args]
# Each run saves a timestamped CSV and plan log under the repository's logs/.
# Press USER to start; USER again or Ctrl+C stops the run.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
SECONDS_TO_RUN=30
if [[ ${1:-} =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    SECONDS_TO_RUN="$1"
    shift
fi
NMPC_RUN_ID="$(date +%Y%m%d_%H%M%S)_$$"
NMPC_LOG_PREFIX="$ROOT/logs/nmpc_upright_$NMPC_RUN_ID"
exec "$ROOT/.venv-nmpc/bin/python" "$ROOT/tools/laptop_control.py" \
    --nmpc --upright-reseed --seconds "$SECONDS_TO_RUN" \
    --out "$NMPC_LOG_PREFIX.csv" --plan-log "${NMPC_LOG_PREFIX}_plans" "$@"
