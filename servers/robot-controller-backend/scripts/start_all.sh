#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

PID_DIR="$BASE_DIR/.pids"
LOG_DIR="$BASE_DIR/logs"
VENV_DIR="$BASE_DIR/venv"

PY="$VENV_DIR/bin/python"
PIP="$VENV_DIR/bin/pip"

mkdir -p "$PID_DIR" "$LOG_DIR"

echo "[all] Backend: $BASE_DIR"

# Load .env if present (export all vars)
if [ -f "$BASE_DIR/.env" ]; then
  echo "[all] Loading .env"
  set -a; . "$BASE_DIR/.env"; set +a
fi

# Ensure venv
if [ ! -x "$PY" ]; then
  echo "[all] Creating venv"
  python3 -m venv "$VENV_DIR"
fi
echo "[all] Upgrading pip/wheel"
"$PIP" install -U pip wheel >/dev/null

# Install requirements (safe to re-run)
if [ -f "$BASE_DIR/requirements.txt" ]; then
  echo "[all] Installing Python deps"
  "$PIP" install -r "$BASE_DIR/requirements.txt"
fi

run_bg () {
  local name="$1"; shift
  local cmd="$*"
  local log="$LOG_DIR/$name.log"
  local pidf="$PID_DIR/$name.pid"

  if [ -f "$pidf" ] && ps -p "$(cat "$pidf")" >/dev/null 2>&1; then
    echo "[$name] already running (pid $(cat "$pidf"))."
    return 0
  fi

  echo "[$name] starting → $cmd"
  bash -lc "$cmd" >>"$log" 2>&1 & echo $! > "$pidf"
  sleep 0.3
  if ps -p "$(cat "$pidf")" >/dev/null 2>&1; then
    echo "[$name] OK (pid $(cat "$pidf")). log: $log"
  else
    echo "[$name] FAILED. See $log"
  fi
}

# --- Servers you have ---
# Line tracker (Python): :8090/line-tracker
run_bg "line_tracker" "cd '$BASE_DIR/sensors' && '$PY' line_tracking_ws_server.py"

# Movement (Python): :8081
run_bg "movement" "cd '$BASE_DIR/movement' && '$PY' movement_ws_server.py"

# Ultrasonic (Go): :8080/ultrasonic
if command -v go >/dev/null 2>&1; then
  run_bg "ultrasonic" "cd '$BASE_DIR/sensors' && go run main_ultrasonic.go"
else
  echo "[ultrasonic] Skipped (Go not installed)"
fi

# Lighting (Go): :8082/lighting
if command -v go >/dev/null 2>&1; then
  run_bg "lighting" "cd '$BASE_DIR/controllers/lighting' && go run main_lighting.go"
else
  echo "[lighting] Skipped (Go not installed)"
fi

# OPTIONAL: Flask video server (uncomment/adjust if you use it)
# run_bg "video" "cd '$BASE_DIR' && '$PY' video_server.py"

echo "[all] Done. Use scripts/status.sh to check; scripts/stop_all.sh to stop."
