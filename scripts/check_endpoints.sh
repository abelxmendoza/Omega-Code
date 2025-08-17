# File: /Omega-Code/scripts/check_endpoints.sh
# Summary:
#   Prints active profile, resolved URLs, pings host, and HEADs the video feed.
#   - No UI env required; falls back to backend .env + ports.
#   - Override profile via:  ./check_endpoints.sh --profile tailscale
#   - Override host or video URL via: --host <ip|name> , --video <url>

#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# ---- Load backend env (for ports, IPs) ----
set -o allexport
[ -f "$ROOT_DIR/servers/robot-controller-backend/.env" ] && . "$ROOT_DIR/servers/robot-controller-backend/.env"
[ -f "$ROOT_DIR/servers/robot-controller-backend/.env.local" ] && . "$ROOT_DIR/servers/robot-controller-backend/.env.local"
# Try common UI env locations if present (optional)
[ -f "$ROOT_DIR/ui/.env" ] && . "$ROOT_DIR/ui/.env"
[ -f "$ROOT_DIR/ui/.env.local" ] && . "$ROOT_DIR/ui/.env.local"
[ -f "$ROOT_DIR/ui/robot-controller-ui/.env" ] && . "$ROOT_DIR/ui/robot-controller-ui/.env"
[ -f "$ROOT_DIR/ui/robot-controller-ui/.env.local" ] && . "$ROOT_DIR/ui/robot-controller-ui/.env.local"
set +o allexport

# ---- Args ----
PROFILE="${NEXT_PUBLIC_NETWORK_PROFILE:-local}"
HOST_OVERRIDE=""
VIDEO_OVERRIDE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--profile) PROFILE="${2:-$PROFILE}"; shift 2;;
    --host)       HOST_OVERRIDE="${2:-}"; shift 2;;
    --video)      VIDEO_OVERRIDE="${2:-}"; shift 2;;
    *) echo "Unknown arg: $1"; exit 1;;
  esac
done
PROFILE="$(echo "$PROFILE" | tr 'A-Z' 'a-z')"

# ---- Host selection ----
case "$PROFILE" in
  tailscale) HOST_DEFAULT="${TAILSCALE_IP_PI:-100.93.225.61}" ;;
  lan)       HOST_DEFAULT="${PI_IP:-192.168.1.107}" ;;
  local|*)   HOST_DEFAULT="localhost" ;;
esac
HOST="${HOST_OVERRIDE:-$HOST_DEFAULT}"
UPROFILE="$(echo "$PROFILE" | tr 'a-z' 'A-Z')"

# ---- Helper to read NEXT_PUBLIC_* for current profile ----
get_env_by_profile() {
  local base="$1"           # e.g., NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT
  local key="${base}_${UPROFILE}"
  printf '%s' "${!key-}"
}

# ---- Build URLs (use NEXT_PUBLIC_* if present; else fallback to ports) ----
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

# ---- Output ----
printf "Profile: %s\n" "$PROFILE"
printf "Host:    %s\n\n" "$HOST"
printf "%-22s %s\n" "Video:" "$video"
printf "%-22s %s\n" "WS Movement:" "$ws_move"
printf "%-22s %s\n" "WS Ultrasonic:" "$ws_ultra"
printf "%-22s %s\n" "WS Line:" "$ws_line"
printf "%-22s %s\n" "WS Lighting:" "$ws_light"
printf "%-22s %s\n\n" "WS Location:" "$ws_loc"

echo "Ping ${HOST}…"
ping -c2 -W2 "$HOST" >/dev/null && echo "✅ Ping OK" || echo "⚠️ Ping failed (may still be fine)"

echo "HEAD ${video}…"
if curl -sSfI "$video" >/dev/null; then
  echo "✅ Video reachable"
else
  echo "❌ Video not reachable"
  echo "   Hint: start your video server (e.g., python3 video_server.py) or run a quick mock:"
  echo "   python3 - <<'PY'; from http.server import BaseHTTPRequestHandler, HTTPServer"
  echo "   class H(BaseHTTPRequestHandler):"
  echo "       def do_HEAD(self): self.send_response(200); self.end_headers()"
  echo "       def do_GET(self):  self.do_HEAD()"
  echo "   HTTPServer(('0.0.0.0', ${VIDEO_PORT:-5000}), H).serve_forever(); PY"
fi
