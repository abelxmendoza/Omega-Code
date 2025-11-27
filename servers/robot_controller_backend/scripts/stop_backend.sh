#!/usr/bin/env bash
set -euo pipefail

echo "Stopping movement and gateway processes (best-effort)"
pkill -f movement/simple_movement_ws_server.py 2>/dev/null || true
pkill -f movement/movement_ws_server.py 2>/dev/null || true
pkill -f "uvicorn servers.gateway_api:app" 2>/dev/null || true

echo "Done."


