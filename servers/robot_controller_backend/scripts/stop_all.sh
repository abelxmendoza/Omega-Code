#!/usr/bin/env bash
# File: scripts/stop_all.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
PID_DIR="$ROOT/.pids"

services=( line_tracker movement ultrasonic lighting )  # add video if needed

stop() {
  local svc="$1"
  local pidf="$PID_DIR/$svc.pid"
  if [[ -f "$pidf" ]]; then
    local pid
    pid="$(tr -d ' \t\r\n' < "$pidf" 2>/dev/null || true)"
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      echo "[$svc] stopping (pid $pid)…"
      # Try graceful stop
      kill "$pid" 2>/dev/null || true
      sleep 1
      # If still alive, try killing the whole process group
      if kill -0 "$pid" 2>/dev/null; then
        echo "[$svc] force-stopping process group (-$pid)…"
        kill -TERM "-$pid" 2>/dev/null || true
        sleep 1
      fi
      # Last resort
      if kill -0 "$pid" 2>/dev/null; then
        kill -KILL "-$pid" 2>/dev/null || true
      fi
    else
      echo "[$svc] not running"
    fi
    rm -f "$pidf"
  else
    echo "[$svc] not running"
  fi
}

for s in "${services[@]}"; do
  stop "$s"
done
