#!/bin/bash
# macOS: Connect Mac to iPhone PAN with blueutil, then make the Pi join via BlueZ.

set -euo pipefail

ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ENV_FILE="${ENV_FILE:-$ROOT_DIR/servers/robot-controller-backend/.env}"

# Load .env safely (handles spaces)
if [ -f "$ENV_FILE" ]; then
  set -o allexport
  # shellcheck disable=SC1090
  . "$ENV_FILE"
  set +o allexport
else
  echo "❌ .env not found at $ENV_FILE"; exit 1
fi

: "${PHONE_MAC:?❌ PHONE_MAC missing in .env}"
: "${PI_HOST:?❌ PI_HOST missing in .env}"

# macOS requirements
if ! command -v blueutil >/dev/null 2>&1; then
  echo "❌ 'blueutil' not installed. Install: brew install blueutil"; exit 1
fi

echo "🔗 macOS → connecting to iPhone ($PHONE_MAC) via Bluetooth..."
# Pair if needed
if ! blueutil --paired | grep -qi "$PHONE_MAC"; then
  echo "   Pairing… (accept on phone if prompted)"
  blueutil --pair "$PHONE_MAC" || true
fi

# Connect (this triggers PAN)
blueutil --connect "$PHONE_MAC" || {
  echo "❌ Failed to connect Mac to iPhone. Make sure Personal Hotspot is ON (Allow Others to Join)."
  exit 1
}

# Show PAN interface info (macOS does NOT have bnep0)
IFACE=$(networksetup -listnetworkserviceorder | \
  awk '/Bluetooth PAN/ {getline; if ($0 ~ /Device:/) { sub(/.*Device: /,""); sub(/\)/,""); print }}')
if [ -n "${IFACE:-}" ]; then
  echo "📡 macOS Bluetooth PAN interface: $IFACE"
  ifconfig "$IFACE" || true
else
  echo "ℹ️ Bluetooth PAN service present; interface becomes active once traffic flows."
fi

echo "🌍 Quick connectivity probe…"
if ping -c 2 -t 2 1.1.1.1 >/dev/null 2>&1; then
  echo "✅ macOS has network connectivity via Bluetooth PAN."
else
  echo "⚠️ Couldn’t confirm Internet; continuing anyway."
fi

# --- Pi side ---
PI_HELPER="$ROOT_DIR/scripts/pi_connect_phone_pan.sh"
[ -f "$PI_HELPER" ] || { echo "❌ Missing $PI_HELPER"; exit 1; }

echo "📤 Copying Pi helper…"
scp "$PI_HELPER" "$PI_HOST:/tmp/pi_connect_phone_pan.sh"

echo "🔗 Asking Pi to connect to iPhone PAN…"
ssh "$PI_HOST" "bash /tmp/pi_connect_phone_pan.sh '$PHONE_MAC'"

echo "✅ Mac and Pi should now both be on the iPhone Bluetooth PAN."
echo "🔁 If flaky, toggle iPhone Personal Hotspot OFF/ON, then re-run."
