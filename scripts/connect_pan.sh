#!/bin/bash

# Script: connect_pan.sh
# 
# This script connects your MacBook to your iPhone's Personal Hotspot using Bluetooth PAN (Personal Area Network).
# Instead of using WiFi or a USB cable, it enables internet access via Bluetooth tethering.
#
# How it works:
# 1. Loads the iPhone's Bluetooth MAC address from the .env file.
# 2. Connects the MacBook to the iPhone via Bluetooth PAN.
# 3. Requests an IP address for the Bluetooth interface (bnep0).
# 4. Verifies the connection by checking network details.
# 5. Tests internet connectivity by pinging Google.
#
# Requirements:
# - Bluetooth PAN must be enabled on your iPhone (Settings → Personal Hotspot → Allow Others to Join).
# - The MacBook must be paired with the iPhone via Bluetooth.
# - The .env file must contain the iPhone’s Bluetooth MAC address.

# Determine project root. Allow override via OMEGA_CODE_ROOT

#!/usr/bin/env bash
# Pi-only: connect to iPhone Bluetooth PAN

set -euo pipefail

ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ENV_FILE="${ENV_FILE:-$ROOT_DIR/servers/robot-controller-backend/.env}"

if [ -f "$ENV_FILE" ]; then
  set -o allexport; . "$ENV_FILE"; set +o allexport
else
  echo "Error: .env not found at $ENV_FILE"; exit 1
fi

: "${PHONE_MAC:?PHONE_MAC missing in .env}"

# Require bluez-tools for bt-network
if ! command -v bt-network >/dev/null 2>&1; then
  sudo apt update && sudo apt install -y bluez-tools
fi

sudo rfkill unblock bluetooth || true
sudo systemctl start bluetooth

# Pair/trust (idempotent)
bluetoothctl <<BT
power on
agent NoInputNoOutput
default-agent
pair $PHONE_MAC
trust $PHONE_MAC
BT

# Connect & DHCP
sudo bt-network -c "$PHONE_MAC" nap
if command -v dhclient >/dev/null 2>&1; then
  sudo dhclient -v bnep0 || true
fi
if command -v dhcpcd >/dev/null 2>&1; then
  sudo dhcpcd -n bnep0 || true
fi

ip addr show bnep0
ping -c 2 -W 2 1.1.1.1 && echo "OK"
