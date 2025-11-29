#!/usr/bin/env bash
set -euo pipefail
# Resolve this script's directory reliably (works when called from anywhere)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Set PYTHONPATH to include the parent directory so Python can find the 'controllers' module
# The parent directory is: servers/robot_controller_backend
PARENT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
export PYTHONPATH="${PARENT_DIR}:${PYTHONPATH:-}"

# hex mode pattern interval brightness
sudo -E /home/omega1/Omega-Code/servers/robot_controller_backend/venv/bin/python \
  led_control.py "$1" "$2" "$3" "$4" "$5"
