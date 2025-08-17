#!/usr/bin/env bash
set -Eeuo pipefail

QUERY="${1:-iphone}"

# Ensure tools + controller up
sudo apt-get update -y >/dev/null 2>&1 || true
sudo apt-get install -y bluez bluez-tools >/dev/null 2>&1 || true
sudo rfkill unblock bluetooth || true
sudo systemctl restart bluetooth
sudo hciconfig hci0 up || true

# Start a persistent agent (auto-accept, no input)
sudo pkill -f bt-agent >/dev/null 2>&1 || true
sudo nohup bt-agent -c NoInputNoOutput >/dev/null 2>&1 &

# Prep controller
bluetoothctl <<BT
power on
pairable on
discoverable on
scan on
BT

# Resolve target by MAC or name
norm_mac() { [[ "$1" =~ ^([0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}$ ]]; }
if norm_mac "$QUERY"; then
  TARGET_MAC="$(echo "$QUERY" | tr '[:lower:]' '[:upper:]' | tr '-' ':')"
else
  TARGET_MAC=""
  for i in {1..30}; do
    LINE="$(bluetoothctl devices | grep -i "$QUERY" || true)"
    if [[ -n "$LINE" ]]; then TARGET_MAC="$(echo "$LINE" | awk '{print $2}' | head -n1)"; break; fi
    sleep 1
  done
  [[ -z "$TARGET_MAC" ]] && { echo "[Pi] ❌ No device matching \"$QUERY\". Open iPhone **Bluetooth** + **Hotspot** (ON), keep screen awake."; exit 1; }
fi
echo "[Pi] Using device: $TARGET_MAC"

# Clean stale record
bluetoothctl remove "$TARGET_MAC" >/dev/null 2>&1 || true

# Pair / trust / connect (agent is persistent via bt-agent)
bluetoothctl <<BT
pair $TARGET_MAC
trust $TARGET_MAC
connect $TARGET_MAC
BT

# Try NAP connect a few times
for attempt in 1 2 3; do
  if sudo bt-network -c "$TARGET_MAC" nap; then
    break
  fi
  bluetoothctl --timeout 5 scan on >/dev/null 2>&1 || true
  sleep 2
done

# Wait for bnep0
for i in {1..15}; do
  if ip link show bnep0 >/dev/null 2>&1; then
    ip addr show bnep0 || true
    echo "[Pi] ✅ bnep0 up. PAN active."
    exit 0
  fi
  sleep 1
done

echo "[Pi] ❌ bnep0 not present. Toggle iPhone Hotspot OFF→ON and rerun."
exit 1
