#!/usr/bin/env bash
set -euo pipefail
LUNAR_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${ISAAC_SIM_PYTHON:-}" ]]; then
    exec "$ISAAC_SIM_PYTHON" "$LUNAR_DIR/launch.py" "$@"
fi
if [[ -n "${ISAAC_SIM_PATH:-}" && -x "$ISAAC_SIM_PATH/python.sh" ]]; then
    exec "$ISAAC_SIM_PATH/python.sh" "$LUNAR_DIR/launch.py" "$@"
fi
if command -v python >/dev/null 2>&1 && python -c 'import importlib.util, sys; sys.exit(importlib.util.find_spec("isaacsim") is None)' >/dev/null 2>&1; then
    exec python "$LUNAR_DIR/launch.py" "$@"
fi
# Local pip-install environment discovered on the development workstation.
LUNAR_PYTHON="$HOME/anaconda3/envs/isaaclab/bin/python"
if [[ -x "$LUNAR_PYTHON" ]]; then
    exec "$LUNAR_PYTHON" "$LUNAR_DIR/launch.py" "$@"
fi
printf '%s\n' 'Isaac Sim Python was not found. Set ISAAC_SIM_PYTHON=/path/to/python or ISAAC_SIM_PATH=/path/to/isaac-sim.' >&2
exit 1
