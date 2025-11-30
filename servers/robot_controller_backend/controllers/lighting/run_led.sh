#!/usr/bin/env bash
set -euo pipefail
# Resolve this script's directory reliably (works when called from anywhere)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Set PYTHONPATH to include the parent directory so Python can find the 'controllers' module
# The parent directory is: servers/robot_controller_backend
PARENT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
export PYTHONPATH="${PARENT_DIR}:${PYTHONPATH:-}"

# Find Python executable - try venv first, then system python3
PYTHON_CMD=""
if [ -f "$PARENT_DIR/venv/bin/python" ]; then
    PYTHON_CMD="$PARENT_DIR/venv/bin/python"
elif [ -f "$PARENT_DIR/venv/bin/python3" ]; then
    PYTHON_CMD="$PARENT_DIR/venv/bin/python3"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_CMD="python3"
else
    echo "❌ [ERROR] Python not found. Please install Python 3 or create venv." >&2
    exit 1
fi

# hexColor hexColor2 mode pattern interval brightness
sudo -E "$PYTHON_CMD" led_control.py "$1" "$2" "$3" "$4" "$5" "$6"
