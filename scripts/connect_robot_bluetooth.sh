#!/bin/bash

# Script: connect_robot_bluetooth.sh
# 
# This script connects both your **MacBook and Raspberry Pi (Omega1)** to the iPhone's Personal Hotspot via Bluetooth PAN.
# Since macOS does not have `bt-network`, it uses `blueutil` instead.

# Determine project root. Allow override via OMEGA_CODE_ROOT
ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# Path to the .env file (can be overridden with ENV_FILE)
ENV_FILE="${ENV_FILE:-$ROOT_DIR/servers/robot-controller-backend/.env}"

# Load environment variables securely
if [ -f "$ENV_FILE" ]; then
  export $(grep -v '^#' "$ENV_FILE" | xargs)
else
  echo "❌ Error: .env file not found at $ENV_FILE"
  exit 1
fi

# Check if PHONE_MAC and PI_IP are set
if [ -z "$PHONE_MAC" ]; then
  echo "❌ Error: PHONE_MAC is not set. Please check your .env file."
  exit 1
fi
if [ -z "$PI_IP" ]; then
  echo "❌ Error: PI_IP is not set. Please check your .env file."
  exit 1
fi

echo "🔗 Attempting to connect MacBook to iPhone Bluetooth PAN..."

# Check if blueutil is installed
if ! command -v blueutil &> /dev/null; then
  echo "❌ Error: 'blueutil' is not installed. Install it with: brew install blueutil"
  exit 1
fi

# Pair with iPhone if not already paired
if ! blueutil --paired | grep -q "$PHONE_MAC"; then
  echo "🔗 Pairing with iPhone..."
  blueutil --pair "$PHONE_MAC"
fi

# Connect MacBook to iPhone Bluetooth PAN
if blueutil --connect "$PHONE_MAC"; then
  echo "✅ MacBook connected to iPhone Bluetooth PAN."
else
  echo "❌ Error: Failed to connect MacBook to Bluetooth PAN."
  exit 1
fi

# Obtain an IP address for the Bluetooth network interface
if ! sudo dhclient bnep0; then
  echo "❌ Error: Failed to obtain an IP address for MacBook on Bluetooth PAN."
  exit 1
fi

# Verify if bnep0 exists before displaying details
if ip link show bnep0 &> /dev/null; then
  echo "📡 Network interface bnep0 details:"
  ip addr show bnep0
else
  echo "❌ Error: bnep0 interface not found on MacBook. Check Bluetooth connection."
  exit 1
fi

# Test internet connectivity from MacBook
echo "🌍 Testing internet connectivity from MacBook..."
if ping -c 4 google.com &> /dev/null; then
  echo "✅ MacBook successfully connected to iPhone's internet."
else
  echo "❌ Error: No internet access on MacBook via Bluetooth PAN."
  exit 1
fi

# SSH into Raspberry Pi (Omega1) and connect it to Bluetooth PAN
echo "🔗 Attempting to SSH into Raspberry Pi (Omega1) at $PI_IP to connect it to Bluetooth PAN..."

if ssh "omega1@$PI_IP" "sudo blueutil --connect '$PHONE_MAC'"; then
  echo "✅ Raspberry Pi (Omega1) connected to iPhone Bluetooth PAN."
else
  echo "❌ Error: Failed to connect Raspberry Pi (Omega1) to Bluetooth PAN."
  exit 1
fi

# Check if Raspberry Pi (Omega1) has an IP on the Bluetooth network
echo "📡 Checking Raspberry Pi (Omega1)'s network status..."
if ssh "omega1@$PI_IP" "ip addr show bnep0"; then
  echo "✅ Raspberry Pi (Omega1) successfully connected to Bluetooth PAN."
else
  echo "❌ Error: Raspberry Pi (Omega1) bnep0 interface not found. Check Bluetooth connection."
  exit 1
fi

# Final SSH connection to Raspberry Pi (Omega1)
echo "🚀 All connections successful. SSHing into Raspberry Pi (Omega1)..."
ssh "omega1@$PI_IP"
