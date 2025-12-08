#!/bin/bash
set -e

# Omega-NetToggle: Clean two-mode network recovery + AP mode script
# Works on Raspberry Pi OS Bookworm (NetworkManager default)
#
# Usage:
#   sudo ./omega-nettoggle.sh restore  # Restore normal WiFi mode
#   sudo ./omega-nettoggle.sh ap       # Enable AP mode
#   sudo ./omega-nettoggle.sh status   # Show network diagnostics

LOG="/var/log/omega-nettoggle.log"
BACKUP_DIR="/etc/omega-network/backups"

# Ensure log directory exists
sudo mkdir -p "$(dirname "$LOG")" 2>/dev/null || true
sudo mkdir -p "$BACKUP_DIR" 2>/dev/null || true

log() {
    echo -e "[OMEGA-NET] $1"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [OMEGA-NET] $1" | sudo tee -a "$LOG" > /dev/null
}

error() {
    log "ERROR: $1"
    exit 1
}

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   error "This script must be run as root (use sudo)"
fi

## ------------------------------------------------------------
## Helper: restore NetworkManager + WiFi normal mode
## ------------------------------------------------------------
restore_wifi_mode() {
    log "=========================================="
    log "Switching to NORMAL WiFi mode..."
    log "=========================================="

    # Unblock radio
    log "Unblocking WiFi radio..."
    sudo rfkill unblock wifi || true
    sleep 1

    # Stop AP services if present
    log "Stopping AP-related services..."
    sudo systemctl stop hostapd 2>/dev/null || true
    sudo systemctl disable hostapd 2>/dev/null || true
    sudo systemctl stop dnsmasq 2>/dev/null || true
    sudo systemctl disable dnsmasq 2>/dev/null || true

    # Restore dhcpcd (if AP modified it)
    if [[ -f /etc/dhcpcd.conf.backup-ap ]]; then
        log "Restoring dhcpcd.conf from backup..."
        sudo cp /etc/dhcpcd.conf.backup-ap /etc/dhcpcd.conf
        sudo systemctl restart dhcpcd
    fi

    # Reload WiFi driver stack
    log "Reloading brcmfmac WiFi driver..."
    sudo modprobe -r brcmfmac brcmutil cfg80211 2>/dev/null || true
    sleep 2
    sudo modprobe brcmfmac || error "Failed to reload WiFi driver"

    # Ensure NetworkManager is active
    log "Enabling NetworkManager..."
    sudo systemctl enable NetworkManager --now
    sleep 2

    # Restart networking
    log "Restarting NetworkManager..."
    sudo systemctl restart NetworkManager
    sleep 3

    # Bring up wlan0
    log "Bringing wlan0 UP..."
    sudo ip link set wlan0 down 2>/dev/null || true
    sleep 1
    sudo ip link set wlan0 up || error "Failed to bring wlan0 up"
    
    # Wait for NetworkManager to configure
    sleep 3

    log "=========================================="
    log "✅ NORMAL WiFi mode restored."
    log "=========================================="
    log "WiFi should now connect to your configured networks."
    log "Check status with: sudo ./omega-nettoggle.sh status"
}

## ------------------------------------------------------------
## Helper: enable AP mode cleanly
## ------------------------------------------------------------
enable_ap_mode() {
    log "=========================================="
    log "Enabling Access Point mode..."
    log "=========================================="

    SSID="Omega1-AP"
    PASSWORD="omegawifi123"
    AP_IP="192.168.4.1"
    DHCP_START="192.168.4.10"
    DHCP_END="192.168.4.200"

    # Backup current configs
    log "Creating backups..."
    sudo mkdir -p "$BACKUP_DIR"
    BACKUP_TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
    
    [[ -f /etc/dhcpcd.conf ]] && sudo cp /etc/dhcpcd.conf "$BACKUP_DIR/dhcpcd.conf.$BACKUP_TIMESTAMP"
    [[ -f /etc/hostapd/hostapd.conf ]] && sudo cp /etc/hostapd/hostapd.conf "$BACKUP_DIR/hostapd.conf.$BACKUP_TIMESTAMP" 2>/dev/null || true
    [[ -f /etc/dnsmasq.conf ]] && sudo cp /etc/dnsmasq.conf "$BACKUP_DIR/dnsmasq.conf.$BACKUP_TIMESTAMP" 2>/dev/null || true

    # Backup dhcpcd for restore
    sudo cp /etc/dhcpcd.conf /etc/dhcpcd.conf.backup-ap

    ## Disable NetworkManager control for wlan0
    log "Stopping NetworkManager..."
    sudo systemctl stop NetworkManager
    sleep 2

    ## Replace dhcpcd for AP mode
    log "Writing AP dhcpcd config..."
    sudo bash -c "cat >/etc/dhcpcd.conf <<EOF
# Omega-NetToggle AP Mode Configuration
# Backup saved at: /etc/dhcpcd.conf.backup-ap
# Restore with: sudo ./omega-nettoggle.sh restore

interface wlan0
    static ip_address=$AP_IP/24
    nohook wpa_supplicant
EOF"

    # Ensure hostapd directory exists
    sudo mkdir -p /etc/hostapd

    ## hostapd config
    log "Writing hostapd config..."
    sudo bash -c "cat >/etc/hostapd/hostapd.conf <<EOF
# Omega-NetToggle AP Mode Configuration
interface=wlan0
driver=nl80211
ssid=$SSID
country_code=US
hw_mode=g
channel=6
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=$PASSWORD
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF"

    ## hostapd default path
    log "Configuring hostapd defaults..."
    sudo mkdir -p /etc/default
    sudo bash -c 'echo "DAEMON_CONF=/etc/hostapd/hostapd.conf" > /etc/default/hostapd'

    ## dnsmasq config
    log "Writing dnsmasq config..."
    sudo bash -c "cat >/etc/dnsmasq.conf <<EOF
# Omega-NetToggle AP Mode Configuration
interface=wlan0
dhcp-range=$DHCP_START,$DHCP_END,255.255.255.0,24h
dhcp-option=3,$AP_IP
dhcp-option=6,$AP_IP
server=8.8.8.8
log-queries
log-dhcp
EOF"

    # Unblock WiFi
    log "Unblocking WiFi radio..."
    sudo rfkill unblock wifi || true
    sleep 1

    # Reload WiFi driver
    log "Reloading WiFi driver..."
    sudo modprobe -r brcmfmac brcmutil cfg80211 2>/dev/null || true
    sleep 2
    sudo modprobe brcmfmac || error "Failed to reload WiFi driver"
    sleep 2

    # Bring up wlan0
    log "Bringing wlan0 UP..."
    sudo ip link set wlan0 down 2>/dev/null || true
    sleep 1
    sudo ip link set wlan0 up || error "Failed to bring wlan0 up"
    sleep 2

    # Restart dhcpcd
    log "Restarting dhcpcd..."
    sudo systemctl restart dhcpcd
    sleep 3

    ## Restart services
    log "Starting AP services..."
    sudo systemctl enable hostapd --now
    sleep 2
    sudo systemctl enable dnsmasq --now
    sleep 2

    # Verify services
    if systemctl is-active --quiet hostapd && systemctl is-active --quiet dnsmasq; then
        log "=========================================="
        log "✅ AP mode active."
        log "=========================================="
        log "SSID: $SSID"
        log "Password: $PASSWORD"
        log "Pi IP: $AP_IP"
        log "Connect and SSH: omega1@$AP_IP"
        log "=========================================="
    else
        error "AP services failed to start. Check logs: sudo journalctl -u hostapd -u dnsmasq"
    fi
}

## ------------------------------------------------------------
## Helper: diagnostics
## ------------------------------------------------------------
show_status() {
    log "=========================================="
    log "=== OMEGA-NET STATUS ==="
    log "=========================================="
    echo

    echo "📡 WiFi Radio Status:"
    rfkill list wifi
    echo

    echo "🔌 Network Interfaces:"
    ip -br addr show
    echo

    echo "🌐 NetworkManager Status:"
    nmcli device status 2>/dev/null || echo "NetworkManager not available"
    echo

    echo "📶 WiFi Connection:"
    nmcli device wifi list 2>/dev/null | head -5 || echo "WiFi scan not available"
    echo

    echo "🔧 Service Status:"
    echo "  NetworkManager: $(systemctl is-active NetworkManager 2>/dev/null || echo 'unknown')"
    echo "  hostapd: $(systemctl is-active hostapd 2>/dev/null || echo 'inactive')"
    echo "  dnsmasq: $(systemctl is-active dnsmasq 2>/dev/null || echo 'inactive')"
    echo "  dhcpcd: $(systemctl is-active dhcpcd 2>/dev/null || echo 'unknown')"
    echo

    echo "📋 Current Mode Detection:"
    if systemctl is-active --quiet hostapd && systemctl is-active --quiet dnsmasq; then
        echo "  Mode: AP (Access Point)"
        echo "  SSID: $(grep -E '^ssid=' /etc/hostapd/hostapd.conf 2>/dev/null | cut -d'=' -f2 || echo 'unknown')"
        echo "  IP: $(ip -4 addr show wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || echo 'unknown')"
    elif systemctl is-active --quiet NetworkManager; then
        echo "  Mode: Client (Normal WiFi)"
        echo "  Connected to: $(nmcli -t -f active,ssid dev wifi | grep '^yes:' | cut -d':' -f2 || echo 'none')"
    else
        echo "  Mode: Unknown (services not running)"
    fi
    echo

    echo "📁 Backup Files:"
    if [[ -d "$BACKUP_DIR" ]]; then
        ls -lh "$BACKUP_DIR" 2>/dev/null | tail -5 || echo "  No backups found"
    else
        echo "  Backup directory not found"
    fi
    echo

    echo "📝 Recent Log Entries:"
    tail -10 "$LOG" 2>/dev/null || echo "  Log file not found"
    echo

    log "=========================================="
}

## ------------------------------------------------------------
## Main mode selector
## ------------------------------------------------------------
case "$1" in
    restore)
        restore_wifi_mode
        ;;
    ap)
        enable_ap_mode
        ;;
    status)
        show_status
        ;;
    *)
        echo "Omega-NetToggle: Clean network recovery + AP mode script"
        echo ""
        echo "Usage: sudo ./omega-nettoggle.sh {restore|ap|status}"
        echo ""
        echo "Modes:"
        echo "  restore  - Restore normal WiFi mode (recover from breakage)"
        echo "  ap       - Enable Access Point mode (Omega1-AP)"
        echo "  status   - Show network diagnostics"
        echo ""
        echo "Examples:"
        echo "  sudo ./omega-nettoggle.sh restore  # Go back to normal WiFi"
        echo "  sudo ./omega-nettoggle.sh ap       # Enable AP mode"
        echo "  sudo ./omega-nettoggle.sh status   # Check network status"
        echo ""
        exit 1
        ;;
esac

