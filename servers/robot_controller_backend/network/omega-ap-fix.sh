#!/bin/bash
# Omega AP Fix — Guaranteed AP configuration for Omega-1 (NetworkManager)
#
# One-command AP mode setup that:
# - Deletes any broken AP config
# - Rebuilds a perfect NetworkManager AP profile
# - Configures DHCP/NAT ("shared" mode)
# - Sets SSID + password
# - Brings the AP online
#
# Usage: sudo omega-ap-fix

set -e

LOG_FILE="/var/log/omega-nettoggle.log"

# Ensure log directory exists
sudo mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true

log() {
    echo "[OMEGA-AP] $1"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [OMEGA-AP] $1" | sudo tee -a "$LOG_FILE" > /dev/null
}

error() {
    log "ERROR: $1"
    exit 1
}

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   error "This script must be run as root (use sudo)"
fi

AP_NAME="Omega1-AP"
AP_IP="192.168.4.1/24"
SSID="Omega1-AP"
PASSWORD="omegawifi123"

log "=========================================="
log "Fixing AP mode configuration..."
log "=========================================="

# Ensure WiFi radio is on
log "Enabling WiFi radio..."
nmcli radio wifi on || true
sleep 2

# Stop client mode if active
log "Disabling client mode if active..."
nmcli connection down Omega1-Client 2>/dev/null || true
sleep 1

# 1. Delete old profile if stuck
log "Removing any existing AP profile..."
nmcli connection delete "$AP_NAME" 2>/dev/null || true
sleep 1

# 2. Create clean AP profile
log "Creating new AP profile..."
nmcli connection add type wifi ifname wlan0 con-name "$AP_NAME" ssid "$SSID" || error "Failed to create AP profile"

# 3. Base AP WiFi configuration
log "Configuring WiFi AP settings..."
nmcli connection modify "$AP_NAME" \
  802-11-wireless.mode ap \
  802-11-wireless.band bg \
  802-11-wireless.channel 6 \
  802-11-wireless.hidden no \
  wifi.mac-address-randomization never || error "Failed to configure WiFi settings"

# 4. Set WPA2 password
log "Setting WPA2 security..."
nmcli connection modify "$AP_NAME" \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "$PASSWORD" || error "Failed to set password"

# 5. Set IPv4 "shared" mode so the Pi runs DHCP
log "Configuring IPv4 shared mode (DHCP/NAT)..."
nmcli connection modify "$AP_NAME" \
  ipv4.method shared \
  ipv4.addresses "$AP_IP" \
  ipv4.gateway 192.168.4.1 \
  ipv4.dns "8.8.8.8 1.1.1.1" \
  ipv6.method ignore || error "Failed to configure IPv4 shared mode"

# Reload connection
log "Reloading NetworkManager connections..."
nmcli connection reload

# Apply AP profile
log "Activating AP mode..."
nmcli connection down "$AP_NAME" 2>/dev/null || true
sleep 1

if nmcli connection up "$AP_NAME"; then
    log "=========================================="
    log "✅ Omega-1 Access Point is now ACTIVE"
    log "=========================================="
    log "SSID: $SSID"
    log "Password: $PASSWORD"
    log "AP IP: 192.168.4.1"
    log "Gateway: 192.168.4.1"
    log "DHCP Range: 192.168.4.x"
    log "=========================================="
    log "Connect from your device → WiFi → $SSID"
    log "Then access: http://192.168.4.1:8000"
    log "=========================================="
    
    # Wait a moment and verify
    sleep 3
    if nmcli connection show --active | grep -q "$AP_NAME"; then
        log "AP mode verified - Access Point is broadcasting"
    else
        log "WARNING: AP connection may not be fully active yet"
    fi
else
    error "Failed to activate AP mode"
fi

