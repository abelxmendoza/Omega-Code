#!/bin/bash
# Omega Network Boot Safety v2
# Runs on boot to ensure WiFi is restored or falls back to AP mode
#
# This script:
# 1. Attempts to restore WiFi client mode on boot
# 2. If WiFi fails, immediately enables AP mode as recovery

set -e

LOG_FILE="/var/log/omega-nettoggle.log"
NETTOGGLE_SCRIPT="/usr/local/bin/omega-nettoggle"

# Fallback to script location if symlink doesn't exist
if [ ! -f "$NETTOGGLE_SCRIPT" ]; then
    NETTOGGLE_SCRIPT="$(dirname "$0")/omega-nettoggle.sh"
fi

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [BOOT] $1" | tee -a "$LOG_FILE"
}

log "Omega Network Boot Safety v2 starting..."

# Wait for NetworkManager to be ready
log "Waiting for NetworkManager to be ready..."
sleep 5

# Attempt WiFi restore
log "Attempting WiFi restore..."
if [ -f "$NETTOGGLE_SCRIPT" ]; then
    "$NETTOGGLE_SCRIPT" restore
    RESTORE_CODE=$?
else
    log "ERROR: omega-nettoggle script not found at $NETTOGGLE_SCRIPT"
    RESTORE_CODE=1
fi

# Wait for connection to establish
sleep 8

# Check if WiFi is enabled
if nmcli -t -f WIFI g | grep -q "enabled"; then
    log "WiFi restore successful - WiFi radio is enabled"
    log "Boot safety check complete"
    exit 0
else
    log "WiFi failed on boot - WiFi radio is not enabled"
    log "Switching to AP mode as recovery..."
    
    if [ -f "$NETTOGGLE_SCRIPT" ]; then
        "$NETTOGGLE_SCRIPT" ap
        AP_CODE=$?
        
        if [ "$AP_CODE" -eq 0 ]; then
            log "AP mode enabled successfully - robot accessible at Omega1-AP"
        else
            log "AP mode enable failed (exit code: $AP_CODE)"
        fi
    else
        log "ERROR: Cannot enable AP mode - omega-nettoggle script not found"
    fi
fi

log "Boot safety check complete"
exit 0
