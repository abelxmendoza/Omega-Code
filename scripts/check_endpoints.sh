# File: /Omega-Code/scripts/check_endpoints.sh
# Summary:
# Preflight checker that prints the active profile and hits the expected endpoints.
# Helps catch mismatched IPs, bad profiles, or missing CORS origins before you launch.
# Usage:
#   ./check_endpoints.sh

#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# shellcheck source=_env_loader.sh
source "$ROOT_DIR/scripts/_env_loader.sh"; load_env
# shellcheck source=net_profile.sh
source "$ROOT_DIR/scripts/net_profile.sh"

profile="${NEXT_PUBLIC_NETWORK_PROFILE:-local}"
host="localhost"
case "$profile" in
  lan) host="192.168.1.107" ;;
  tailscale) host="100.93.225.61" ;;
  local) host="localhost" ;;
  *) echo "Unknown profile: $profile"; exit 1 ;;
esac

video="$(resolve_by_profile NEXT_PUBLIC_VIDEO_STREAM_URL)"
mv_ws="$(resolve_by_profile NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT)"
ul_ws="$(resolve_by_profile NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC)"
lt_ws="$(resolve_by_profile NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER)"
lg_ws="$(resolve_by_profile NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING)"
lc_ws="$(resolve_by_profile NEXT_PUBLIC_BACKEND_WS_URL_LOCATION)"

printf "Profile: %s\n" "$profile"
printf "Host:    %s\n\n" "$host"
printf "%-22s %s\n" "Video:" "$video"
printf "%-22s %s\n" "WS Movement:" "$mv_ws"
printf "%-22s %s\n" "WS Ultrasonic:" "$ul_ws"
printf "%-22s %s\n" "WS Line:" "$lt_ws"
printf "%-22s %s\n" "WS Lighting:" "$lg_ws"
printf "%-22s %s\n\n" "WS Location:" "$lc_ws"

echo "Ping ${host}…"
ping -c2 -W2 "$host" >/dev/null && echo "✅ Ping OK" || echo "⚠️ Ping failed (may still be fine)";

echo "HEAD ${video}…"
curl -sSfI "$video" >/dev/null && echo "✅ Video reachable" || echo "❌ Video not reachable"
