#!/usr/bin/env bash
# start_sim_local.sh — start the FastAPI backend locally with SIM_MODE=1
#
# Called by the UI's /api/sim-launcher route when you hit "Start Sim Backend"
# without the robot present.  Searches for a venv in the usual locations.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(dirname "$(dirname "$BACKEND_DIR")")"

cd "$BACKEND_DIR"

# Find a Python interpreter — prefer a project venv
if [ -f "$BACKEND_DIR/venv/bin/activate" ]; then
    source "$BACKEND_DIR/venv/bin/activate"
elif [ -f "$REPO_ROOT/venv/bin/activate" ]; then
    source "$REPO_ROOT/venv/bin/activate"
fi

export SIM_MODE=1
export PYTHONPATH="$BACKEND_DIR"
export ROS_BRIDGE_ENABLED=0

# exec replaces bash with python — PID stays the same (easier to kill)
exec python main_api.py
