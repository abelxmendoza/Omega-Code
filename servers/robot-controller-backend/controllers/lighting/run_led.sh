#!/usr/bin/env bash
set -euo pipefail
# Resolve this script's directory reliably (works when called from anywhere)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# hex mode pattern interval brightness
sudo -E /home/omega1/Omega-Code/servers/robot-controller-backend/venv/bin/python \
  led_control.py "$1" "$2" "$3" "$4" "$5"
