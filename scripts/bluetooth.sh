#!/usr/bin/env bash
# bluetooth.sh — Bluetooth utilities for the Omega robot
#
# Usage:
#   ./scripts/bluetooth.sh connect   # Run on Mac: connect Mac + Pi to iPhone BT PAN
#   ./scripts/bluetooth.sh diagnose  # Run on Pi:  scan for devices, show BT state

set -euo pipefail

CMD="${1:-}"

usage() {
  echo "Usage: $0 [connect|diagnose]"
  echo "  connect   -- (Mac) Connect Mac and Pi to iPhone Bluetooth PAN"
  echo "  diagnose  -- (Pi)  Run Bluetooth diagnostics and scan for devices"
  exit 1
}

# ------------------------------------------------------------
# connect: Mac-side — pair/connect to iPhone, then trigger Pi
# ------------------------------------------------------------
cmd_connect() {
  ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
  ENV_FILE="${ENV_FILE:-$ROOT_DIR/servers/robot_controller_backend/.env}"

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

  if ! command -v blueutil >/dev/null 2>&1; then
    echo "❌ 'blueutil' not installed. Install: brew install blueutil"; exit 1
  fi

  echo "Connecting Mac to iPhone ($PHONE_MAC) via Bluetooth..."
  if ! blueutil --paired | grep -qi "$PHONE_MAC"; then
    echo "   Pairing... (accept on phone if prompted)"
    blueutil --pair "$PHONE_MAC" || true
  fi

  blueutil --connect "$PHONE_MAC" || {
    echo "❌ Failed to connect. Make sure iPhone Personal Hotspot is ON (Allow Others to Join)."
    exit 1
  }

  IFACE=$(networksetup -listnetworkserviceorder | \
    awk '/Bluetooth PAN/ {getline; if ($0 ~ /Device:/) { sub(/.*Device: /,""); sub(/\)/,""); print }}')
  if [ -n "${IFACE:-}" ]; then
    echo "Bluetooth PAN interface: $IFACE"
    ifconfig "$IFACE" || true
  else
    echo "Bluetooth PAN service present; interface activates once traffic flows."
  fi

  echo "Quick connectivity probe..."
  if ping -c 2 -t 2 1.1.1.1 >/dev/null 2>&1; then
    echo "Mac has network connectivity via Bluetooth PAN."
  else
    echo "Warning: could not confirm Internet; continuing anyway."
  fi

  # Trigger Pi to connect using pi_bt_pair_and_pan.sh
  PI_HELPER="$ROOT_DIR/scripts/pi_bt_pair_and_pan.sh"
  [ -f "$PI_HELPER" ] || { echo "❌ Missing $PI_HELPER"; exit 1; }

  echo "Copying Pi helper..."
  scp "$PI_HELPER" "$PI_HOST:/tmp/pi_bt_pair_and_pan.sh"

  echo "Asking Pi to connect to iPhone PAN..."
  ssh "$PI_HOST" "bash /tmp/pi_bt_pair_and_pan.sh '$PHONE_MAC'"

  echo "Mac and Pi should now both be on the iPhone Bluetooth PAN."
  echo "If flaky, toggle iPhone Personal Hotspot OFF/ON and re-run."
}

# ------------------------------------------------------------
# diagnose: Pi-side — show BT state and scan for devices
# ------------------------------------------------------------
cmd_diagnose() {
  echo "[Pi] Bluetooth diagnostics starting..."

  sudo rfkill unblock bluetooth || true
  sudo systemctl restart bluetooth
  sudo hciconfig hci0 up || true

  echo "---- hciconfig -a ----"
  hciconfig -a || true

  echo "---- bluetoothctl show ----"
  bluetoothctl show || true

  sudo pkill -f bt-agent >/dev/null 2>&1 || true
  sudo nohup bt-agent -c NoInputNoOutput >/dev/null 2>&1 &
  sleep 1

  bluetoothctl <<BT
power on
agent on
default-agent
pairable on
discoverable on
scan on
scan on bredr
BT

  echo "[Pi] Scanning for up to 25s... keep iPhone Bluetooth screen open."
  bluetoothctl --timeout 25 scan on >/dev/null 2>&1 || true

  echo "---- bluetoothctl devices (visible) ----"
  bluetoothctl devices || true

  echo "---- bluetoothctl devices Paired ----"
  bluetoothctl devices Paired || true

  echo "[Pi] Done. If your iPhone is listed above, copy its MAC (AA:BB:CC:DD:EE:FF)."
}

# ------------------------------------------------------------
case "$CMD" in
  connect)  cmd_connect ;;
  diagnose) cmd_diagnose ;;
  *)        usage ;;
esac
