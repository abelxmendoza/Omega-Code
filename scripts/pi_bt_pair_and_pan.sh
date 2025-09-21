#!/usr/bin/env bash
set -Eeuo pipefail
QUERY="${1:-iphone}"

# Prep BT stack
sudo rfkill unblock bluetooth || true
sudo systemctl restart bluetooth
sudo hciconfig hci0 up || true

# Start a persistent agent (auto-accept pairing)
sudo pkill -f bt-agent >/dev/null 2>&1 || true
sudo nohup bt-agent -c NoInputNoOutput >/dev/null 2>&1 &

# Enable scan / discoverable
bluetoothctl <<BT
power on
pairable on
discoverable on
scan on
BT

# Resolve target by MAC or by name (e.g., "iphone" / "abel")
if [[ "$QUERY" =~ ^([0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}$ ]]; then
  MAC=$(echo "$QUERY" | tr '[:lower:]' '[:upper:]' | tr '-' ':')
else
  MAC=""
  for i in {1..30}; do
    MAC=$(bluetoothctl devices | grep -iE 'iphone|abel' | awk '{print $2}' | head -n1 || true)
    [[ -n "$MAC" ]] && break
    sleep 1
  done
fi
[[ -n "$MAC" ]] || { echo "[Pi] iPhone not discovered. Keep Bluetooth & Hotspot screens open, then rerun."; exit 1; }
echo "[Pi] Using device $MAC"

# Pair / trust / connect (idempotent; agent handles prompts)
bluetoothctl <<BT
pair $MAC
trust $MAC
connect $MAC
BT

# Bring up PAN (NAP), with a couple retries
for attempt in 1 2 3; do
  if sudo bt-network -c "$MAC" nap; then break; fi
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
