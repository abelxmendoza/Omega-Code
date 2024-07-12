#!/bin/bash

# Replace with your iPhone's Bluetooth MAC address
PHONE_MAC="C0:17:54:43:73:A3"

# Connect to the PAN
sudo bt-network -c $PHONE_MAC nap

# Obtain an IP address
sudo dhclient bnep0

# Verify the connection
echo "Network interface bnep0 details:"
ip addr show bnep0

echo "Testing connectivity with ping:"
ping -c 4 google.com
