#!/bin/bash

# This script connects the MacBook, Ubuntu Laptop, or Alienware Aurora to the iPhone hotspot via USB-C,
# checks internet connectivity, verifies the Tailscale connection,
# and SSHs into the Raspberry Pi using Tailscale IP. It dynamically
# adjusts settings based on the hostname of the machine (Laptop1-hostname, Laptop2-hostname, scythe, or aurora-desktop).

# Determine project root. Allow override via OMEGA_CODE_ROOT
ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# Load environment variables from the specified .env file
ENV_FILE="${ENV_FILE:-$ROOT_DIR/config/.env.script2}"
if [ -f "$ENV_FILE" ]; then
    source "$ENV_FILE"
else
    echo "Environment file not found: $ENV_FILE" >&2
    exit 1
fi

# Get the hostname of the current machine
HOSTNAME=$(hostname)

# Function to check iPhone USB connection
check_iphone_usb() {
    echo "Checking iPhone USB connection..."
    if [[ "$(uname -s)" == "Darwin" ]]; then
        # macOS
        if networksetup -listallhardwareports | grep -q "iPhone USB"; then
            echo "iPhone USB connection found."
            networksetup -setnetworkserviceenabled "iPhone USB" on
        else
            echo "No iPhone USB connection found. Please connect your iPhone via USB-C."
            exit 1
        fi
    else
        # Linux - check for USB network interface
        if ip link show | grep -q "usb\|enp.*usb\|eth.*usb"; then
            echo "USB network interface detected."
        else
            echo "No USB network interface found. Please connect your iPhone via USB-C."
            echo "Note: On Linux, you may need to enable USB tethering on your iPhone."
            exit 1
        fi
    fi
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
    ssh "$PI_USER_PI@$TAILSCALE_IP_PI"
}

# Main script
if [ "$HOSTNAME" == "$HOSTNAME_LAPTOP1" ]; then
    PI_USER="$PI_USER_LAPTOP1"
    TAILSCALE_IP="$TAILSCALE_IP_LAPTOP1"
    check_iphone_usb
elif [ "$HOSTNAME" == "$HOSTNAME_LAPTOP2" ]; then
    PI_USER="$PI_USER_LAPTOP2"
    TAILSCALE_IP="$TAILSCALE_IP_LAPTOP2"
    check_iphone_usb  # Assuming both laptops will use the USB-C connection for the hotspot
elif [ "$HOSTNAME" == "scythe" ] || [ "$HOSTNAME" == "$HOSTNAME_LAPTOP3" ]; then
    # New laptop (scythe) - use LAPTOP3 config if available, otherwise use defaults
    PI_USER="${PI_USER_LAPTOP3:-${PI_USER_LAPTOP2:-pi}}"
    TAILSCALE_IP="${TAILSCALE_IP_LAPTOP3:-${TAILSCALE_IP_LAPTOP2:-}}"
    if [ -z "$TAILSCALE_IP" ]; then
        echo "Warning: TAILSCALE_IP not configured for this laptop. Please set HOSTNAME_LAPTOP3, PI_USER_LAPTOP3, and TAILSCALE_IP_LAPTOP3 in config/.env.script2"
    fi
    check_iphone_usb
elif [ "$HOSTNAME" == "$HOSTNAME_AURORA" ] || [ "$HOSTNAME" == "aurora-desktop" ]; then
    # Alienware Aurora - use AURORA config
    PI_USER="${PI_USER_AURORA:-pi}"
    TAILSCALE_IP="${TAILSCALE_IP_AURORA:-}"
    if [ -z "$TAILSCALE_IP" ]; then
        echo "Warning: TAILSCALE_IP not configured for Aurora. Please set HOSTNAME_AURORA, PI_USER_AURORA, and TAILSCALE_IP_AURORA in config/.env.script2"
    fi
    # Aurora is desktop, skip USB hotspot check (uses Ethernet/WiFi)
    echo "Alienware Aurora detected - skipping USB hotspot check (using Ethernet/WiFi)"
else
    echo "Unknown hostname: $HOSTNAME"
    echo "Supported hostnames: $HOSTNAME_LAPTOP1, $HOSTNAME_LAPTOP2, scythe (or set HOSTNAME_LAPTOP3), aurora-desktop (or set HOSTNAME_AURORA)"
    exit 1
fi

check_internet
check_tailscale
ssh_into_pi

