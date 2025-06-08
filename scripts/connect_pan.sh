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
ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# Path to the .env file (can be overridden with ENV_FILE)
ENV_FILE="${ENV_FILE:-$ROOT_DIR/servers/robot-controller-backend/.env}"

# Load environment variables from .env file
if [ -f "$ENV_FILE" ]; then
  export $(grep -v '^#' "$ENV_FILE" | xargs)
else
  echo "Error: .env file not found at $ENV_FILE"
  exit 1
fi

# Check if PHONE_MAC is set
if [ -z "$PHONE_MAC" ]; then
  echo "Error: PHONE_MAC is not set. Please check your .env file."
  exit 1
fi

echo "Attempting to connect to Bluetooth PAN using MAC: $PHONE_MAC"

# Connect to the PAN using the MAC address stored in PHONE_MAC
if ! sudo bt-network -c "$PHONE_MAC" nap; then
  echo "Error: Failed to connect to Bluetooth PAN."
  exit 1
fi

echo "Connected to Bluetooth PAN. Attempting to obtain IP address..."

# Obtain an IP address for the Bluetooth network interface
if ! sudo dhclient bnep0; then
  echo "Error: Failed to obtain an IP address for bnep0."
  exit 1
fi

# Verify if bnep0 exists before displaying details
if ip link show bnep0 &> /dev/null; then
  echo "Network interface bnep0 details:"
  ip addr show bnep0
else
  echo "Error: bnep0 interface not found. Check Bluetooth connection."
  exit 1
fi

# Test the connectivity by pinging Google
echo "Testing internet connectivity..."
if ping -c 4 google.com &> /dev/null; then
  echo "Internet is accessible via Bluetooth PAN."
else
  echo "Error: No internet access via Bluetooth PAN."
  exit 1
fi

echo "Bluetooth PAN setup complete!"
