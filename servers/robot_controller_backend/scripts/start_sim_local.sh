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

# Find a Python interpreter that has FastAPI installed.
# Preference order: project venv → repo venv → python3.10 → python3 → python
PYTHON=""
if [ -f "$BACKEND_DIR/venv/bin/python" ] && "$BACKEND_DIR/venv/bin/python" -c "import fastapi" 2>/dev/null; then
    PYTHON="$BACKEND_DIR/venv/bin/python"
elif [ -f "$REPO_ROOT/venv/bin/python" ] && "$REPO_ROOT/venv/bin/python" -c "import fastapi" 2>/dev/null; then
    PYTHON="$REPO_ROOT/venv/bin/python"
elif command -v python3.10 >/dev/null 2>&1 && python3.10 -c "import fastapi" 2>/dev/null; then
    PYTHON="python3.10"
elif command -v python3 >/dev/null 2>&1 && python3 -c "import fastapi" 2>/dev/null; then
    PYTHON="python3"
elif command -v python >/dev/null 2>&1 && python -c "import fastapi" 2>/dev/null; then
    PYTHON="python"
fi

if [ -z "$PYTHON" ]; then
    echo "ERROR: No Python interpreter with FastAPI found." >&2
    echo "Install with: pip install fastapi uvicorn" >&2
    exit 1
fi

echo "Using interpreter: $PYTHON ($(${PYTHON} --version 2>&1))"

export SIM_MODE=1
export PYTHONPATH="$BACKEND_DIR"
export ROS_BRIDGE_ENABLED=0

# exec replaces bash with python — PID stays the same (easier to kill)
exec "$PYTHON" main_api.py
