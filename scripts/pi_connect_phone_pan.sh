#!/usr/bin/env bash
set -Eeuo pipefail

QUERY_RAW="${1:?Usage: pi_connect_phone_pan.sh <iPhone-MAC or name-fragment>}"

norm_mac() {
  local s="${1//-/:}"; s="${s^^}"
  [[ "$s" =~ ^([0-9A-F]{2}:){5}[0-9A-F]{2}$ ]] && { echo "$s"; return 0; }
  return 1
}

PHONE_MAC=""
if norm_mac "$QUERY_RAW" >/dev/null 2>&1; then
  PHONE_MAC="$(norm_mac "$QUERY_RAW")"
else
  # We got a name fragment (e.g., "iphone" or "abel"). Discover a matching device.
  QUERY="$(echo "$QUERY_RAW" | tr '[:upper:]' '[:lower:]')"

  echo "[Pi] Ensuring BlueZ tooling…"
  if ! command -v bt-network >/dev/null 2>&1; then
    sudo apt-get update -y
    sudo apt-get install -y bluez bluez-tools
  fi

  echo "[Pi] Starting bluetooth service…"
  sudo rfkill unblock bluetooth || true
  sudo systemctl enable bluetooth --now
  sudo hciconfig hci0 up || true

  # Clean agent, then register
  echo "[Pi] Prepare agent + make discoverable/pairable…"
  bluetoothctl <<BT
agent off
power on
agent on
default-agent
pairable on
discoverable on
scan on
BT

  echo "[Pi] Scanning for device name containing \"$QUERY\" (keep iPhone Hotspot ON, screen awake)…"
  # Give it up to ~25s to show
  for i in {1..25}; do
    LINE="$(bluetoothctl devices | awk '{$1=$1}1' | tr -d '\r' | grep -iE '^Device [0-9A-F:]{17} .*$' | grep -i "$QUERY" || true)"
    if [[ -n "$LINE" ]]; then
      PHONE_MAC="$(echo "$LINE" | awk '{print $2}' | head -n1)"
      break
    fi
    sleep 1
  done

  if [[ -z "$PHONE_MAC" ]]; then
    echo "[Pi] ❌ Could not find a device matching \"$QUERY\". Open iPhone Settings → Bluetooth and Hotspot, keep screen ON, then retry."
    exit 1
  fi
fi

echo "[Pi] Target device: $PHONE_MAC"

# Remove stale record (ignore errors)
bluetoothctl remove "$PHONE_MAC" >/dev/null 2>&1 || true

# Pair / trust / connect (ignore harmless agent warnings)
bluetoothctl <<BT
power on
agent on
default-agent
pairable on
discoverable on
pair $PHONE_MAC
trust $PHONE_MAC
connect $PHONE_MAC
BT

# Verify paired
if ! bluetoothctl info "$PHONE_MAC" | grep -q "Paired: yes"; then
  echo "[Pi] ❌ Pairing failed. If phone prompted, accept; otherwise open Bluetooth screen and retry."
  exit 1
fi

echo "[Pi] Connect to iPhone NAP…"
for attempt in 1 2 3; do
  if sudo bt-network -c "$PHONE_MAC" nap; then
    break
  fi
  echo "[Pi] bt-network failed (try $attempt). Brief rescan and retry…"
  bluetoothctl --timeout 5 scan on >/dev/null 2>&1 || true
  sleep 2
done

echo "[Pi] Waiting for bnep0…"
for i in {1..15}; do
  if ip link show bnep0 >/dev/null 2>&1; then
    ip addr show bnep0 || true
    echo "[Pi] ✅ bnep0 up. Internet should be available via iPhone PAN."
    exit 0
  fi
  sleep 1
done

echo "[Pi] ❌ bnep0 not present. Try:"
echo "    • iPhone: Hotspot OFF→ON (stay on the Hotspot screen)"
echo "    • Pi:     sudo systemctl restart bluetooth"
echo "    • Re-run:  bash /tmp/pi_connect_phone_pan.sh '$QUERY_RAW'"
exit 1
