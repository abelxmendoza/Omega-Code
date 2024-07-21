# File: /Users/abel_elreaper/Desktop/Omega-Code/scripts/connect_pan.sh

#!/bin/bash

# This script connects to a Personal Area Network (PAN) using Bluetooth, obtains an IP address, and verifies the connection.
# The Bluetooth MAC address of the phone is stored in a .env file for security and convenience.

# Load environment variables from .env file
if [ -f "../.env" ]; then
  export $(cat ../.env | grep -v '#' | awk '/=/ {print $1}')
fi

# Check if PHONE_MAC is set
if [ -z "$PHONE_MAC" ]; then
  echo "PHONE_MAC is not set. Please check your .env file."
  exit 1
fi

# Connect to the PAN using the MAC address stored in PHONE_MAC
sudo bt-network -c $PHONE_MAC nap

# Obtain an IP address for the Bluetooth network interface
sudo dhclient bnep0

# Verify the connection by displaying network interface details
echo "Network interface bnep0 details:"
ip addr show bnep0

# Test the connectivity by pinging google.com
echo "Testing connectivity with ping:"
ping -c 4 google.com
