#!/bin/bash
# Omega Network Watchdog v2
# Monitors internet connectivity and auto-restores WiFi or falls back to AP mode
#
# This watchdog runs continuously and checks internet connectivity.
# If internet is down, it attempts to restore WiFi client mode.
# If restore fails, it enables AP mode as fallback.

set -e

LOG_FILE="/var/log/omega-nettoggle.log"
NETTOGGLE_SCRIPT="/usr/local/bin/omega-nettoggle"

# Fallback to script location if symlink doesn't exist
if [ ! -f "$NETTOGGLE_SCRIPT" ]; then
    NETTOGGLE_SCRIPT="$(dirname "$0")/omega-nettoggle.sh"
fi

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [WATCHDOG] $1" | tee -a "$LOG_FILE"
}

check_internet() {
    # Check internet connectivity by pinging Google DNS
    if ping -c1 -W1 8.8.8.8 >/dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

log "Omega Network Watchdog v2 started"
log "Monitoring internet connectivity every 30 seconds"

CHECK_COUNT=0

while true; do
    if ! check_internet; then
        log "Internet connectivity check FAILED - attempting restore..."
        
        # Run restore
        if [ -f "$NETTOGGLE_SCRIPT" ]; then
            "$NETTOGGLE_SCRIPT" restore
            RESTORE_CODE=$?
            
            # Wait for connection to establish
            sleep 10
            
            # Check if restore worked
            if check_internet; then
                log "WiFi restore successful - internet connectivity restored"
            else
                log "Restore failed - internet still down"
                log "Switching to AP mode as fallback..."
                
                # Fall back to AP mode
                "$NETTOGGLE_SCRIPT" ap
                AP_CODE=$?
                
                if [ "$AP_CODE" -eq 0 ]; then
                    log "AP mode enabled as fallback - robot accessible at Omega1-AP"
                else
                    log "AP mode enable failed (exit code: $AP_CODE)"
                fi
            fi
        else
            log "ERROR: omega-nettoggle script not found at $NETTOGGLE_SCRIPT"
        fi
    else
        # Internet is healthy - log occasionally (every 10 checks = 5 minutes)
        CHECK_COUNT=$((CHECK_COUNT + 1))
        if [ $((CHECK_COUNT % 10)) -eq 0 ]; then
            log "Internet connectivity OK (check #$CHECK_COUNT)"
        fi
    fi
    
    sleep 30
done
