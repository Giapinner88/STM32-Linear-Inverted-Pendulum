#!/usr/bin/env bash
# MuJoCo rehearsal of the laptop controllers; see sim_mujoco.py for options.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
exec "$ROOT/.venv-nmpc/bin/python" "$ROOT/nmpc/sim_mujoco.py" "$@"
