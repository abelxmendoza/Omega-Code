#!/usr/bin/env bash
# Raspberry Pi: connect to iPhone Bluetooth PAN (NAP) via BlueZ.

set -euo pipefail

PHONE_MAC="${1:-}"
if [[ -z "$PHONE_MAC" ]]; then
  echo "Usage: $0 <PHONE_MAC>"; exit 1
fi

echo "[Pi] Ensuring BlueZ tooling…"
if ! dpkg -s bluez >/dev/null 2>&1; then
  sudo apt update && sudo apt install -y bluez
fi
if ! command -v bt-network >/dev/null 2>&1 && ! command -v bt-pan >/dev/null 2>&1; then
  sudo apt update && sudo apt install -y bluez-tools
fi

echo "[Pi] Starting bluetooth service…"
sudo rfkill unblock bluetooth || true
sudo systemctl enable bluetooth >/dev/null 2>&1 || true
sudo systemctl start bluetooth

echo "[Pi] Pair/trust iPhone ($PHONE_MAC)…"
bluetoothctl <<BT
power on
agent NoInputNoOutput
default-agent
pair $PHONE_MAC
trust $PHONE_MAC
BT

echo "[Pi] Connect to iPhone NAP…"
if command -v bt-network >/dev/null 2>&1; then
  sudo bt-network -c "$PHONE_MAC" nap || true
elif command -v bt-pan >/dev/null 2>&1; then
  sudo bt-pan client "$PHONE_MAC" || true
fi

echo "[Pi] Waiting for bnep0…"
for i in {1..8}; do
  ip link show bnep0 >/dev/null 2>&1 && break
  sleep 1
done

ip link show bnep0 >/dev/null 2>&1 || {
  echo "[Pi] ❌ bnep0 not present. Toggle Personal Hotspot OFF/ON and re-run."; exit 1; }

echo "[Pi] Requesting IP on bnep0…"
if command -v dhclient >/dev/null 2>&1; then
  sudo dhclient -v bnep0 || true
fi
if command -v dhcpcd >/dev/null 2>&1; then
  sudo dhcpcd -n bnep0 || true
fi

echo "[Pi] Interface status:"
ip addr show bnep0

if ping -c 2 -W 2 1.1.1.1 >/dev/null 2>&1; then
  echo "[Pi] ✅ Connectivity OK via Bluetooth PAN."
else
  echo "[Pi] ⚠️ No ping; still fine for LAN to phone/Mac."
fi
