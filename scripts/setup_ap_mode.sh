#!/usr/bin/env bash
# =============================================================================
# setup_ap_mode.sh — Configure Omega-1 WiFi AP (hotspot) mode
# =============================================================================
# Sets up the Pi so it can operate as a WiFi access point for field use —
# no router required. Your laptop connects directly to the Pi.
#
# What this does:
#   1. Creates the Omega1-AP NetworkManager connection profile
#   2. Installs polkit rules so omega1 user can switch modes without sudo
#   3. Installs the omega-ap toggle script (/usr/local/bin/omega-ap)
#   4. Installs the NM dispatcher for mid-session auto-fallback
#   5. Installs the boot check service for field-boot auto-fallback
#
# Usage:
#   ./scripts/setup_ap_mode.sh
#   ./scripts/setup_ap_mode.sh --wifi-ssid "MyHomeWiFi"   # override home SSID
#
# After setup:
#   omega-ap on      — switch to AP mode (field)
#   omega-ap off     — switch back to home WiFi
#   omega-ap status  — show current mode and IP
#
# Field workflow:
#   1. Power on Pi (anywhere)
#   2. Wait ~45 s — Omega1-AP appears automatically if no known WiFi found
#   3. Connect laptop to: Omega1-AP / omega1robot
#   4. Robot UI at: http://10.42.0.1:8000
#   5. SSH:         ssh omega1@10.42.0.1
# =============================================================================

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NETWORK_DIR="$REPO_ROOT/scripts/network"

# ── Config (override via args) ────────────────────────────────────────────────
AP_SSID="Omega1-AP"
AP_PASSWORD="omega1robot"
AP_IP="10.42.0.1/24"
AP_CHANNEL="6"
HOME_WIFI_CON="SpectrumSetup-89"   # NM connection name for home WiFi

for arg in "$@"; do
  case "$arg" in
    --wifi-ssid=*)   HOME_WIFI_CON="${arg#*=}" ;;
    --ap-password=*) AP_PASSWORD="${arg#*=}" ;;
  esac
done

echo "=== Omega-1 AP Mode Setup ==="
echo "AP SSID:    $AP_SSID"
echo "AP IP:      $AP_IP"
echo "Home WiFi:  $HOME_WIFI_CON"
echo ""

# ─────────────────────────────────────────────────────────────────────────────
# 1. Create NM AP connection profile (idempotent)
# ─────────────────────────────────────────────────────────────────────────────
echo "[1/5] Creating Omega1-AP NetworkManager profile..."
if nmcli con show "$AP_SSID" &>/dev/null; then
  echo "  Profile already exists — skipping."
else
  nmcli con add \
    type wifi ifname wlan0 con-name "$AP_SSID" ssid "$AP_SSID" \
    wifi.mode ap wifi.band bg wifi.channel "$AP_CHANNEL" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$AP_PASSWORD" \
    ipv4.method shared ipv4.addresses "$AP_IP" \
    ipv6.method disabled \
    connection.autoconnect no
  echo "  Profile created."
fi

# ─────────────────────────────────────────────────────────────────────────────
# 2. Polkit rules — allow omega1 user to manage connections without sudo
# ─────────────────────────────────────────────────────────────────────────────
echo "[2/5] Installing polkit rules..."
sudo tee /etc/polkit-1/localauthority/50-local.d/50-nm-omega.pkla > /dev/null << 'EOF'
[Allow omega1 network control]
Identity=unix-user:omega1
Action=org.freedesktop.NetworkManager.network-control
ResultAny=yes
ResultInactive=yes
ResultActive=yes

[Allow omega1 wifi share protected]
Identity=unix-user:omega1
Action=org.freedesktop.NetworkManager.wifi.share.protected
ResultAny=yes
ResultInactive=yes
ResultActive=yes

[Allow omega1 wifi share open]
Identity=unix-user:omega1
Action=org.freedesktop.NetworkManager.wifi.share.open
ResultAny=yes
ResultInactive=yes
ResultActive=yes
EOF
echo "  Polkit rules installed."

# ─────────────────────────────────────────────────────────────────────────────
# 3. omega-ap toggle script
# ─────────────────────────────────────────────────────────────────────────────
echo "[3/5] Installing omega-ap toggle script..."
sudo install -m 755 "$NETWORK_DIR/omega-ap" /usr/local/bin/omega-ap

# Patch the home WiFi connection name into the installed script
sudo sed -i "s/^WIFI_CON=.*/WIFI_CON=\"${HOME_WIFI_CON}\"/" /usr/local/bin/omega-ap
echo "  /usr/local/bin/omega-ap installed."

# ─────────────────────────────────────────────────────────────────────────────
# 4. NM dispatcher — mid-session auto-fallback
# ─────────────────────────────────────────────────────────────────────────────
echo "[4/5] Installing NM dispatcher script..."
sudo install -m 755 "$NETWORK_DIR/99-ap-fallback" \
  /etc/NetworkManager/dispatcher.d/99-ap-fallback
echo "  /etc/NetworkManager/dispatcher.d/99-ap-fallback installed."

# ─────────────────────────────────────────────────────────────────────────────
# 5. Boot fallback service
# ─────────────────────────────────────────────────────────────────────────────
echo "[5/5] Installing boot fallback service..."
sudo install -m 755 "$NETWORK_DIR/omega-ap-check" /usr/local/bin/omega-ap-check
sudo install -m 644 "$NETWORK_DIR/omega-ap-fallback.service" \
  /etc/systemd/system/omega-ap-fallback.service
sudo systemctl daemon-reload
sudo systemctl enable omega-ap-fallback.service
echo "  omega-ap-fallback.service enabled."

# ─────────────────────────────────────────────────────────────────────────────
# Done
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "============================================================"
echo "  AP mode setup complete!"
echo "============================================================"
echo ""
echo "  Commands:"
echo "    omega-ap on      switch to AP mode"
echo "    omega-ap off     switch back to home WiFi"
echo "    omega-ap status  show current mode"
echo ""
echo "  Field use:"
echo "    WiFi SSID:  $AP_SSID"
echo "    Password:   $AP_PASSWORD"
echo "    Robot IP:   10.42.0.1"
echo "    UI:         http://10.42.0.1:8000"
echo "    SSH:        ssh omega1@10.42.0.1"
echo ""
echo "  Auto-fallback: Pi activates AP ~45s after boot if home WiFi"
echo "  is not found. No manual intervention needed in the field."
echo ""
