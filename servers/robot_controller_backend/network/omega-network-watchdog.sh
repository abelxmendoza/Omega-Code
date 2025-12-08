#!/bin/bash
# Omega Network Watchdog
# Monitors WiFi connectivity and auto-restores if connection is lost
#
# This watchdog runs continuously and checks WiFi every 30 seconds.
# If WiFi is down, it automatically runs restore_wifi_mode().

set -e

LOG="/var/log/omega-nettoggle.log"
NETTOGGLE_SCRIPT="/usr/local/bin/omega-nettoggle"

# Fallback to script location if symlink doesn't exist
if [ ! -f "$NETTOGGLE_SCRIPT" ]; then
    NETTOGGLE_SCRIPT="$(dirname "$0")/omega-nettoggle.sh"
fi

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [WATCHDOG] $1" | tee -a "$LOG"
}

check_wifi() {
    # Check if wlan0 interface exists
    if ! ip link show wlan0 >/dev/null 2>&1; then
        log "wlan0 interface not found"
        return 1
    fi
    
    # Check if interface is UP
    if ! ip link show wlan0 | grep -q "state UP"; then
        log "wlan0 interface is DOWN"
        return 1
    fi
    
    # Check if interface has an IP address
    if ! ip addr show wlan0 | grep -q "inet "; then
        log "wlan0 has no IP address"
        return 1
    fi
    
    # Check if we can reach default gateway
    GW=$(ip route | grep default | awk '{print $3}' | head -1)
    if [ -z "$GW" ]; then
        log "No default gateway found"
        return 1
    fi
    
    if ! ping -c1 -W1 "$GW" >/dev/null 2>&1; then
        log "Cannot ping default gateway: $GW"
        return 1
    fi
    
    return 0
}

log "Omega Network Watchdog started"
log "Monitoring WiFi connectivity every 30 seconds"

while true; do
    if ! check_wifi; then
        log "WiFi connectivity check FAILED - attempting restore..."
        
        # Run restore
        if [ -f "$NETTOGGLE_SCRIPT" ]; then
            "$NETTOGGLE_SCRIPT" restore
            RESTORE_CODE=$?
            
            if [ "$RESTORE_CODE" -eq 0 ]; then
                log "WiFi restore successful"
            else
                log "WiFi restore failed with exit code: $RESTORE_CODE"
            fi
        else
            log "ERROR: omega-nettoggle script not found at $NETTOGGLE_SCRIPT"
        fi
    else
        # WiFi is healthy - log occasionally (every 10 checks = 5 minutes)
        CHECK_COUNT=$((CHECK_COUNT + 1))
        if [ $((CHECK_COUNT % 10)) -eq 0 ]; then
            log "WiFi connectivity OK (check #$CHECK_COUNT)"
        fi
    fi
    
    sleep 30
done

