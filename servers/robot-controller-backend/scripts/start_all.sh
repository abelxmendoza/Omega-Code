#!/usr/bin/env bash
# File: scripts/start_all.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
PID_DIR="$ROOT/.pids"
LOG_DIR="$ROOT/logs"
VENV_DIR="$ROOT/venv"
REQS="$ROOT/requirements.txt"
PY="$VENV_DIR/bin/python"

echo "[all] Backend: $ROOT"

mkdir -p "$PID_DIR" "$LOG_DIR"

# Load .env (export all keys while sourcing)
if [[ -f "$ROOT/.env" ]]; then
  echo "[all] Loading .env"
  set -a
  # shellcheck disable=SC1091
  source "$ROOT/.env"
  set +a
fi

# Ensure venv + tools
if [[ ! -x "$PY" ]]; then
  echo "[all] Creating venv"
  python3 -m venv "$VENV_DIR"
fi

echo "[all] Upgrading pip/wheel"
"$VENV_DIR/bin/pip" install -U pip wheel >/dev/null

# Install Python deps (idempotent)
if [[ -f "$REQS" ]]; then
  echo "[all] Installing Python deps"
  "$VENV_DIR/bin/pip" install -r "$REQS"
fi

rotate_log() {
  local name="$1"
  local log="$LOG_DIR/$name.log"
  if [[ -f "$log" ]]; then
    mv "$log" "$LOG_DIR/$name.log.$(date +%Y%m%d-%H%M%S)" || true
  fi
  : > "$log"
}

already_running() {
  local name="$1"
  local pidf="$PID_DIR/$name.pid"
  if [[ -f "$pidf" ]]; then
    local pid; pid="$(cat "$pidf" || true)"
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      echo "[$name] already running (pid $pid)."
      return 0
    fi
  fi
  return 1
}

start_cmd() {
  local name="$1"; shift
  local workdir="$1"; shift
  local log="$LOG_DIR/$name.log"
  local pidf="$PID_DIR/$name.pid"

  if already_running "$name"; then
    return
  fi

  rotate_log "$name"
  echo "[$name] starting → cd '$workdir' && $*"
  (
    cd "$workdir"
    # Run detached, capture PID of the background shell (sufficient for stop/status)
    nohup bash -lc "$*" >>"$log" 2>&1 &
    echo $! >"$pidf"
  )

  local pid; pid="$(cat "$pidf" || true)"
  if [[ -n "${pid:-}" ]] && sleep 0.5 && kill -0 "$pid" 2>/dev/null; then
    echo "[$name] OK (pid $pid). log: $log"
  else
    echo "[$name] FAILED. See $log"
  fi
}

# ---- Start services ----

# 1) Line tracker (Python)
start_cmd "line_tracker" "$ROOT/sensors" \
  "'$PY' line_tracking_ws_server.py"

# 2) Movement (Python)
start_cmd "movement" "$ROOT/movement" \
  "'$PY' movement_ws_server.py"

# 3) Ultrasonic (Go)
if command -v go >/dev/null 2>&1; then
  start_cmd "ultrasonic" "$ROOT/sensors" \
    "go run main_ultrasonic.go"
else
  echo "[ultrasonic] Skipped (go not installed)."
fi

# 4) Lighting (Go)
if command -v go >/dev/null 2>&1; then
  start_cmd "lighting" "$ROOT/controllers/lighting" \
    "go run main_lighting.go"
else
  echo "[lighting] Skipped (go not installed)."
fi

echo "[all] Done. Use scripts/status.sh to check; scripts/stop_all.sh to stop."
