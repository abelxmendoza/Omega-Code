#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PID_DIR="$BASE_DIR/.pids"

check_one () {
  local name="$1" pidf="$PID_DIR/$name.pid"
  if [ -f "$pidf" ]; then
    local pid; pid="$(cat "$pidf")"
    ps -p "$pid" >/dev/null 2>&1 && echo "[$name] RUNNING (pid $pid)" || echo "[$name] DEAD (stale pid $pid)"
  else
    echo "[$name] STOPPED"
  fi
}

check_one "line_tracker"
check_one "movement"
check_one "ultrasonic"
check_one "lighting"
# check_one "video"
