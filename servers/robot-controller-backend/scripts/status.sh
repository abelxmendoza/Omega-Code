#!/usr/bin/env bash
# File: scripts/status.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
PID_DIR="$ROOT/.pids"

services=( line_tracker movement ultrasonic lighting )  # add video here if you add it later

check_one () {
  local name="$1"
  local pidf="$PID_DIR/$name.pid"
  if [[ -f "$pidf" ]]; then
    local pid
    pid="$(tr -d ' \t\r\n' < "$pidf" 2>/dev/null || true)"
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      echo "[$name] RUNNING (pid $pid)"
    else
      echo "[$name] DEAD (stale pid ${pid:-?})"
    fi
  else
    echo "[$name] STOPPED"
  fi
}

for s in "${services[@]}"; do
  check_one "$s"
done
