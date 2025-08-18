#!/usr/bin/env bash
set -Eeuo pipefail
echo "[Pi] Bluetooth diagnostics starting…"

sudo rfkill unblock bluetooth || true
sudo systemctl restart bluetooth
sudo hciconfig hci0 up || true

echo "---- hciconfig -a ----"
hciconfig -a || true

echo "---- bluetoothctl show ----"
bluetoothctl show || true

# Clean & start a persistent agent so pairing can auto-accept
sudo pkill -f bt-agent >/dev/null 2>&1 || true
sudo nohup bt-agent -c NoInputNoOutput >/dev/null 2>&1 &
sleep 1

# Make the controller discoverable/pairable and scan
bluetoothctl <<BT
power on
agent on
default-agent
pairable on
discoverable on
scan on
scan on bredr
BT

echo "[Pi] Scanning for up to 25s… keep iPhone Bluetooth screen open."
bluetoothctl --timeout 25 scan on >/dev/null 2>&1 || true

echo "---- bluetoothctl devices (what we see) ----"
bluetoothctl devices || true

echo "---- bluetoothctl devices Paired (already paired) ----"
bluetoothctl devices Paired || true

echo "[Pi] Done. If your iPhone is listed above, copy its MAC (AA:BB:CC:DD:EE:FF)."
