#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_PYTHON="${ISAAC_SIM_PYTHON:-$HOME/anaconda3/envs/isaaclab/bin/python}"
exec "$SIM_PYTHON" "$PROJECT_ROOT/third_party/Deep-Whole-Body-Control/isaaclab/launch.py" "$@"
