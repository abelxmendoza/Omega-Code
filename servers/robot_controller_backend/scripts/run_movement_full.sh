#!/usr/bin/env bash
set -euo pipefail

# Defaults (can be overridden via env)
: "${PORT_MOVEMENT:=8081}"
: "${ROBOT_SIM:=0}"
: "${ORIGIN_ALLOW:=http://localhost:3000}"

cd "$(dirname "$0")/.."

if [ -d venv ]; then
  source venv/bin/activate
fi

echo "Starting full movement WS on :${PORT_MOVEMENT} (SIM=${ROBOT_SIM})"
exec sudo -E env PORT_MOVEMENT="${PORT_MOVEMENT}" ROBOT_SIM="${ROBOT_SIM}" ORIGIN_ALLOW="${ORIGIN_ALLOW}" \
  "$(pwd)/venv/bin/python" movement/movement_ws_server.py


