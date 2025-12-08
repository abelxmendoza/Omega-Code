#!/bin/bash
# Omega Network Boot Safety
# Runs on boot to ensure WiFi is restored or falls back to AP mode
#
# This script:
# 1. Attempts to restore WiFi on boot
# 2. Tracks boot failures in /etc/omega-net/bootcount
# 3. After 3 consecutive failures, enables AP mode as recovery

set -e

LOG="/var/log/omega-nettoggle.log"
STATE_FILE="/etc/omega-net/bootcount"
NETTOGGLE_SCRIPT="/usr/local/bin/omega-nettoggle"

# Fallback to script location if symlink doesn't exist
if [ ! -f "$NETTOGGLE_SCRIPT" ]; then
    NETTOGGLE_SCRIPT="$(dirname "$0")/omega-nettoggle.sh"
fi

# Ensure state directory exists
mkdir -p /etc/omega-net

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [BOOT-SAFETY] $1" | tee -a "$LOG"
}

log "Omega Network Boot Safety starting..."

# Read current failure count
COUNT=0
if [ -f "$STATE_FILE" ]; then
    COUNT=$(cat "$STATE_FILE" 2>/dev/null || echo "0")
fi

log "Previous boot failure count: $COUNT"

# Attempt WiFi restore
log "Attempting WiFi restore..."
if [ -f "$NETTOGGLE_SCRIPT" ]; then
    "$NETTOGGLE_SCRIPT" restore
    RESTORE_CODE=$?
else
    log "ERROR: omega-nettoggle script not found at $NETTOGGLE_SCRIPT"
    RESTORE_CODE=1
fi

if [ "$RESTORE_CODE" -eq 0 ]; then
    # Success - reset counter
    log "WiFi restore successful - resetting failure counter"
    echo 0 > "$STATE_FILE"
    log "Boot safety check complete - WiFi restored"
    exit 0
else
    # Failure - increment counter
    COUNT=$((COUNT + 1))
    echo $COUNT > "$STATE_FILE"
    log "WiFi restore failed (exit code: $RESTORE_CODE) - failure count: $COUNT"
fi

# If failed 3+ times, enable AP mode as recovery
if [ "$COUNT" -ge 3 ]; then
    log "Boot failure count >= 3 - enabling AP mode as recovery"
    
    if [ -f "$NETTOGGLE_SCRIPT" ]; then
        "$NETTOGGLE_SCRIPT" ap
        AP_CODE=$?
        
        if [ "$AP_CODE" -eq 0 ]; then
            log "AP mode enabled successfully - robot accessible at omega1@192.168.4.1"
            # Reset counter after successful AP mode
            echo 0 > "$STATE_FILE"
        else
            log "AP mode enable failed (exit code: $AP_CODE)"
        fi
    else
        log "ERROR: Cannot enable AP mode - omega-nettoggle script not found"
    fi
fi

log "Boot safety check complete"
exit 0

