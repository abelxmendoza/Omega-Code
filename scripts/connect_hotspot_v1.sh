#!/bin/bash

# This script automates the process of connecting a Mac to an iPhone hotspot,
# verifying internet connectivity, checking the Tailscale VPN status, and then 
# SSH-ing into a Raspberry Pi through the Tailscale network. 
# Environment variables for the iPhone SSID, iPhone password, Tailscale IP, 
# and Raspberry Pi user should be defined in a .env file.

# Load environment variables from .env file
export $(grep -v '^#' config/.env | xargs)

# Function to connect to iPhone hotspot
connect_to_hotspot() {
    echo "Connecting to iPhone hotspot..."
    networksetup -setairportnetwork en0 "$IPHONE_SSID" "$IPHONE_PASSWORD"
}

# Function to check internet connectivity
check_internet() {
    echo "Checking internet connection..."
    if ping -c 1 8.8.8.8 &> /dev/null; then
        echo "Internet connection is active."
    else
        echo "No internet connection. Please check your hotspot settings."
        exit 1
    fi
}

# Function to check Tailscale status
check_tailscale() {
    echo "Checking Tailscale connection..."
    if tailscale status | grep -q "$TAILSCALE_IP"; then
        echo "Tailscale is connected."
    else
        echo "Tailscale is not connected. Starting Tailscale..."
        sudo tailscale up
    fi
}

# Function to SSH into Raspberry Pi
ssh_into_pi() {
    echo "Connecting to Raspberry Pi via SSH..."
    ssh "$PI_USER@$TAILSCALE_IP"
}

# Main script
connect_to_hotspot
check_internet
check_tailscale
ssh_into_pi
