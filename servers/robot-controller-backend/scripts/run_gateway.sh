#!/usr/bin/env bash
set -euo pipefail

# Defaults (can be overridden via env)
: "${PORT:=7070}"
: "${HOST:=0.0.0.0}"
# Example: DS_MOVE_WS=ws://127.0.0.1:8081

cd "$(dirname "$0")/.."

if [ -d venv ]; then
  source venv/bin/activate
fi

echo "Starting gateway on ${HOST}:${PORT} (DS_MOVE_WS=${DS_MOVE_WS:-unset})"
exec env ${DS_MOVE_WS:+DS_MOVE_WS="$DS_MOVE_WS"} \
  "$(pwd)/venv/bin/uvicorn" servers.gateway_api:app --host "$HOST" --port "$PORT"
