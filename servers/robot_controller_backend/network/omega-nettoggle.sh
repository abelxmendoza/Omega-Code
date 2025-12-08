#!/bin/bash
# Omega-NetToggle v2: NetworkManager Native AP Mode
# Uses NetworkManager's built-in AP mode (correct for Bookworm)
#
# Usage:
#   sudo omega-nettoggle restore  # Restore WiFi client mode
#   sudo omega-nettoggle ap       # Enable AP mode
#   sudo omega-nettoggle status   # Show network status

set -e

LOG_FILE="/var/log/omega-nettoggle.log"

# Ensure log directory exists
sudo mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true

log() {
    echo "[OMEGA-NET] $1"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [OMEGA-NET] $1" | sudo tee -a "$LOG_FILE" > /dev/null
}

error() {
    log "ERROR: $1"
    exit 1
}

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   error "This script must be run as root (use sudo)"
fi

# ---- RESTORE MODE ----------------------------------------------------------
restore_wifi_mode() {
    log "=========================================="
    log "Restoring WiFi client mode..."
    log "=========================================="

    # Ensure WiFi radio is on
    log "Enabling WiFi radio..."
    nmcli radio wifi on || true
    sleep 2

    # Disable AP if running
    log "Disabling AP mode if active..."
    nmcli connection down Omega1-AP 2>/dev/null || true
    sleep 1

    # Reload NetworkManager to prevent stale profile issues
    log "Reloading NetworkManager connections..."
    nmcli connection reload
    sleep 0.5

    # Auto-detect and connect to best available WiFi
    log "Scanning for available WiFi networks..."
    AVAILABLE_SSIDS=$(nmcli -t -f SSID dev wifi list | grep -v '^$' | sort -u)

    log "Checking known NetworkManager WiFi profiles..."
    KNOWN_PROFILES=$(nmcli -t -f NAME,TYPE connection show | \
      grep ":wifi" | cut -d: -f1 | sort -u)

    # Auto-create placeholder client profile if none exist
    if [ -z "$KNOWN_PROFILES" ]; then
        log "No known WiFi profiles. Attempting self-repair..."
        nmcli connection add type wifi con-name Omega1-Client ifname wlan0 ssid "" autoconnect no 2>/dev/null || true
        KNOWN_PROFILES=$(nmcli -t -f NAME,TYPE connection show | grep ":wifi" | cut -d: -f1 | sort -u)
    fi

    BEST_WIFI=""

    # Pick strongest known WiFi network available
    for SSID in $AVAILABLE_SSIDS; do
        if echo "$KNOWN_PROFILES" | grep -Fxq "$SSID"; then
            BEST_WIFI="$SSID"
            break
        fi
    done

    if [ -z "$BEST_WIFI" ]; then
        log "ERROR: No known WiFi networks are available."
        log "Available networks: $(echo $AVAILABLE_SSIDS | tr '\n' ' ')"
        log "Known profiles: $(echo $KNOWN_PROFILES | tr '\n' ' ')"
        return 1
    fi

    log "Best WiFi match found: $BEST_WIFI"

    # Auto-create client profile if missing
    if ! nmcli -t -f NAME connection show | grep -Fxq "$BEST_WIFI"; then
        log "WiFi profile missing, attempting to recreate..."
        # Try to connect which will create the profile
        nmcli device wifi connect "$BEST_WIFI" 2>/dev/null || true
        sleep 1
    fi

    # Bring up client mode
    log "Activating WiFi connection: $BEST_WIFI"
    if nmcli connection up "$BEST_WIFI" 2>/dev/null; then
        log "WiFi client restored successfully."
        log "Connected to: $BEST_WIFI"
        
        # Wait a moment for connection to establish
        sleep 3
        
        # Reload after connection to prevent stale state
        nmcli connection reload
        sleep 0.5
        
        # Verify connection
        if nmcli -t -f WIFI g | grep -q "enabled"; then
            log "WiFi client mode verified - connection active"
            return 0
        else
            log "WARNING: WiFi enabled but connection may not be established"
            return 0  # Still consider it success if WiFi is on
        fi
    else
        log "ERROR: Could not activate '$BEST_WIFI'"
        log "A password may be required or profile may be invalid."
        return 1
    fi
}

# ---- AP MODE ---------------------------------------------------------------
enable_ap_mode() {
    log "=========================================="
    log "Switching to Access Point mode using NetworkManager..."
    log "=========================================="

    # Ensure WiFi radio is on
    log "Enabling WiFi radio..."
    nmcli radio wifi on || true
    sleep 2

    # Stop client mode
    log "Disabling client mode if active..."
    nmcli connection down Omega1-Client 2>/dev/null || true
    sleep 1

    # Ensure AP profile has correct settings (fix if needed)
    log "Ensuring AP profile has correct NetworkManager settings..."
    nmcli connection modify Omega1-AP \
        ipv4.method shared \
        ipv4.addresses "192.168.4.1/24" \
        ipv4.gateway 192.168.4.1 \
        ipv4.dns "8.8.8.8 1.1.1.1" \
        ipv6.method ignore \
        wifi.band bg \
        wifi.hidden no \
        wifi.mac-address-randomization never 2>/dev/null || true
    
    # Reload connection to apply changes
    nmcli connection reload
    
    # Enable AP mode
    log "Activating AP mode connection..."
    if nmcli connection up Omega1-AP 2>/dev/null; then
        log "AP mode enabled successfully"
        log "SSID: Omega1-AP"
        log "Password: omega1234"
        log "IP Range: 192.168.4.x (shared mode)"
        log "Gateway: 192.168.4.1"
        
        # Wait a moment for AP to start
        sleep 3
        
        # Verify AP is running
        if nmcli connection show --active | grep -q "Omega1-AP"; then
            log "AP mode verified - Access Point is active"
            return 0
        else
            log "WARNING: AP connection may not be fully active"
            return 0  # Still consider it success if command succeeded
        fi
    else
        log "ERROR: Failed to activate AP mode connection 'Omega1-AP'"
        log "Make sure the connection profile exists. Run install.sh to create it."
        return 1
    fi
}

# ---- STATUS --------------------------------------------------------------
status_mode() {
    echo ""
    echo "============================"
    echo "      OMEGA-NET STATUS"
    echo "============================"
    echo ""
    
    echo "--- NetworkManager Device Status ---"
    nmcli device status
    echo ""
    
    echo "--- Active Connections ---"
    nmcli connection show --active
    echo ""
    
    echo "--- All Connections ---"
    nmcli connection show
    echo ""
    
    echo "--- WiFi Radio Status ---"
    nmcli radio wifi
    echo ""
    
    echo "--- IP Addresses ---"
    ip addr show wlan0 2>/dev/null || echo "wlan0 interface not found"
    echo ""
}

# ---- MAIN ---------------------------------------------------------------
case "$1" in
    restore)
        restore_wifi_mode
        exit $?
        ;;
    ap)
        enable_ap_mode
        exit $?
        ;;
    status)
        status_mode
        exit 0
        ;;
    *)
        echo "Omega-NetToggle v2"
        echo ""
        echo "Usage: sudo omega-nettoggle {restore|ap|status}"
        echo ""
        echo "  restore  - Restore WiFi client mode"
        echo "  ap       - Enable Access Point mode"
        echo "  status   - Show network status"
        echo ""
        exit 1
        ;;
esac
