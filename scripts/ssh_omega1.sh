#!/usr/bin/env bash
# SSH helper script for omega1
# Usage: ./scripts/ssh_omega1.sh [tailscale|lan|local]

set -euo pipefail

PROFILE="${1:-tailscale}"
PROFILE=$(echo "$PROFILE" | tr '[:upper:]' '[:lower:]')

# Load environment variables if available
ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
if [ -f "$ROOT_DIR/.env" ]; then
    set -o allexport
    source "$ROOT_DIR/.env"
    set +o allexport
fi

# Determine host based on profile
case "$PROFILE" in
    tailscale)
        HOST="${OMEGA1_TAILSCALE_HOST:-omega1-tailscale}"
        echo "🔗 Connecting to omega1 via Tailscale..."
        ;;
    lan)
        HOST="${OMEGA1_LAN_HOST:-omega1}"
        echo "🔗 Connecting to omega1 via LAN..."
        ;;
    local|*)
        HOST="${OMEGA1_LOCAL_HOST:-omega1}"
        echo "🔗 Connecting to omega1 (local)..."
        ;;
esac

# Check if hostname or IP
if [[ "$HOST" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "📍 Using IP: $HOST"
else
    echo "📍 Using hostname: $HOST"
fi

# SSH with common options
SSH_USER="${OMEGA1_SSH_USER:-omega1}"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"

echo "🚀 Connecting as $SSH_USER@$HOST..."
echo ""

# If additional arguments provided, execute them remotely
if [ $# -gt 1 ]; then
    shift  # Remove profile argument
    exec ssh $SSH_OPTS "$SSH_USER@$HOST" "$@"
else
    # Interactive SSH session
    exec ssh $SSH_OPTS "$SSH_USER@$HOST"
fi

