#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PID_DIR="$BASE_DIR/.pids"

stop_one () {
  local name="$1" pidf="$PID_DIR/$name.pid"
  if [ -f "$pidf" ]; then
    local pid; pid="$(cat "$pidf")"
    if ps -p "$pid" >/dev/null 2>&1; then
      echo "[$name] stopping pid $pid"
      kill "$pid" || true
      for _ in {1..20}; do ps -p "$pid" >/dev/null 2>&1 || break; sleep 0.2; done
      ps -p "$pid" >/dev/null 2>&1 && kill -9 "$pid" || true
    else
      echo "[$name] not running"
    fi
    rm -f "$pidf"
  else
    echo "[$name] no pid file"
  fi
}

stop_one "line_tracker"
stop_one "movement"
stop_one "ultrasonic"
stop_one "lighting"
# stop_one "video"
echo "[all] Stopped."

