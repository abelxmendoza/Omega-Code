# File: /Omega-Code/scripts/check_endpoints.sh
# Summary:
#   Profile-aware endpoint check for coffee-shop networks.
#   - Works without UI env; derives URLs from backend ports if needed.
#   - --profile tailscale|lan|local
#   - --host <ip|name> to override HOST
#   - --video <url> to override video URL
#   - --skip-ping to avoid ICMP (guest Wi-Fi often blocks it)
#   - Also does TCP port probes (5000/8080/8081/8082/8090/8091)

#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# ---- Load backend (for ports) + optional UI envs ----
set -o allexport
[ -f "$ROOT_DIR/servers/robot-controller-backend/.env" ] && . "$ROOT_DIR/servers/robot-controller-backend/.env"
[ -f "$ROOT_DIR/servers/robot-controller-backend/.env.local" ] && . "$ROOT_DIR/servers/robot-controller-backend/.env.local"
for f in "$ROOT_DIR/ui/.env" "$ROOT_DIR/ui/.env.local" \
         "$ROOT_DIR/ui/robot-controller-ui/.env" "$ROOT_DIR/ui/robot-controller-ui/.env.local"; do
  [ -f "$f" ] && . "$f"
done
set +o allexport

# ---- Args ----
PROFILE="${NEXT_PUBLIC_NETWORK_PROFILE:-local}"
HOST_OVERRIDE=""
VIDEO_OVERRIDE=""
SKIP_PING=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--profile) PROFILE="${2:-$PROFILE}"; shift 2 ;;
    --host)       HOST_OVERRIDE="${2:-}";   shift 2 ;;
    --video)      VIDEO_OVERRIDE="${2:-}";  shift 2 ;;
    --skip-ping)  SKIP_PING=1;              shift   ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done
PROFILE="$(echo "$PROFILE" | tr A-Z a-z)"
UPROFILE="$(echo "$PROFILE" | tr a-z A-Z)"

# ---- Host selection ----
case "$PROFILE" in
  tailscale) HOST_DEFAULT="${TAILSCALE_IP_PI:-100.93.225.61}" ;;
  lan)       HOST_DEFAULT="${PI_IP:-192.168.1.107}" ;;
  *)         HOST_DEFAULT="localhost" ;;
esac
HOST="${HOST_OVERRIDE:-$HOST_DEFAULT}"

# ---- Helper to read NEXT_PUBLIC_* for current profile ----
get_env_by_profile() {
  local base="$1"           # e.g., NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT
  local key="${base}_${UPROFILE}"
  printf '%s' "${!key-}"
}

# ---- Build URLs (env wins; else fallback to ports/paths) ----
video="${VIDEO_OVERRIDE:-$(get_env_by_profile NEXT_PUBLIC_VIDEO_STREAM_URL)}"
[ -z "$video" ] && video="http://${HOST}:${VIDEO_PORT:-5000}/video_feed"

ws_move="$(get_env_by_profile NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT)"
[ -z "$ws_move" ] && ws_move="ws://${HOST}:${PORT_MOVEMENT:-8081}"

ws_ultra="$(get_env_by_profile NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC)"
[ -z "$ws_ultra" ] && ws_ultra="ws://${HOST}:${PORT_ULTRASONIC:-8080}${ULTRA_PATH:-/ultrasonic}"

ws_line="$(get_env_by_profile NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER)"
[ -z "$ws_line" ] && ws_line="ws://${HOST}:${PORT_LINE_TRACKER:-8090}/line-tracker"

ws_light="$(get_env_by_profile NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING)"
[ -z "$ws_light" ] && ws_light="ws://${HOST}:${PORT_LIGHTING:-8082}/lighting"

ws_loc="$(get_env_by_profile NEXT_PUBLIC_BACKEND_WS_URL_LOCATION)"
[ -z "$ws_loc" ] && ws_loc="ws://${HOST}:${PORT_LOCATION:-8091}/location"

printf "Profile: %s\n" "$PROFILE"
printf "Host:    %s\n\n" "$HOST"
printf "%-22s %s\n" "Video:" "$video"
printf "%-22s %s\n" "WS Movement:" "$ws_move"
printf "%-22s %s\n" "WS Ultrasonic:" "$ws_ultra"
printf "%-22s %s\n" "WS Line:" "$ws_line"
printf "%-22s %s\n" "WS Lighting:" "$ws_light"
printf "%-22s %s\n\n" "WS Location:" "$ws_loc"

# ---- Ping (optional) ----
if [[ "$SKIP_PING" -eq 1 ]]; then
  echo "Ping ${HOST}… (skipped)"
else
  if ping -c2 -W2 "$HOST" >/dev/null 2>&1; then
    echo "✅ Ping OK"
  else
    echo "⚠️ Ping failed (often blocked on guest Wi-Fi); continuing with TCP checks…"
  fi
fi

# ---- TCP port checks (nc or /dev/tcp fallback) ----
check_port() {
  local host="$1" port="$2" label="$3"
  if command -v nc >/dev/null 2>&1; then
    if nc -z -w2 "$host" "$port" >/dev/null 2>&1; then
      echo "✅ ${label} port ${port} reachable (TCP)"
    else
      echo "❌ ${label} port ${port} not reachable"
    fi
  else
    (echo >/dev/tcp/"$host"/"$port") >/dev/null 2>&1 \
      && echo "✅ ${label} port ${port} reachable (TCP)" \
      || echo "❌ ${label} port ${port} not reachable"
  fi
}

check_port "$HOST" "${VIDEO_PORT:-5000}"      "Video"
check_port "$HOST" "${PORT_MOVEMENT:-8081}"   "Movement"
check_port "$HOST" "${PORT_ULTRASONIC:-8080}" "Ultrasonic"
check_port "$HOST" "${PORT_LINE_TRACKER:-8090}" "Line"
check_port "$HOST" "${PORT_LIGHTING:-8082}"   "Lighting"
check_port "$HOST" "${PORT_LOCATION:-8091}"   "Location"

echo "HEAD ${video}…"
if curl -sSfI "$video" >/dev/null; then
  echo "✅ Video reachable"
else
  echo "❌ Video not reachable"
  echo "   Hint: start your video server (e.g., python3 video_server.py) or run the quick mock."
fi
