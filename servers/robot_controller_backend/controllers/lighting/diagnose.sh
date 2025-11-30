#!/usr/bin/env bash
# Diagnostic script for lighting server troubleshooting

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🔍 Lighting Server Diagnostics"
echo "================================"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_pass() {
    echo -e "${GREEN}✅${NC} $1"
}

check_fail() {
    echo -e "${RED}❌${NC} $1"
}

check_warn() {
    echo -e "${YELLOW}⚠️${NC} $1"
}

# 1. Check script permissions
echo "1. Checking script permissions..."
if [ -x "$SCRIPT_DIR/run_led.sh" ]; then
    check_pass "run_led.sh is executable"
else
    check_fail "run_led.sh is NOT executable (run: chmod +x run_led.sh)"
fi
echo ""

# 2. Check Python dependencies
echo "2. Checking Python dependencies..."
PARENT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
PYTHON_CMD=""
if [ -f "$PARENT_DIR/venv/bin/python3" ]; then
    PYTHON_CMD="$PARENT_DIR/venv/bin/python3"
    check_pass "Using venv Python: $PYTHON_CMD"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_CMD="python3"
    check_warn "Using system Python (venv not found)"
else
    check_fail "Python3 not found"
    echo ""
    exit 1
fi

if "$PYTHON_CMD" -c "import rpi_ws281x" 2>/dev/null; then
    check_pass "rpi-ws281x module installed"
else
    check_fail "rpi-ws281x module NOT installed (run: ./install_led_deps.sh)"
fi
echo ""

# 3. Check hardware (Raspberry Pi)
echo "3. Checking hardware..."
if [ -f /proc/cpuinfo ] && grep -q "Raspberry Pi" /proc/cpuinfo; then
    check_pass "Running on Raspberry Pi"
    MODEL=$(grep "Model" /proc/cpuinfo | head -1)
    echo "   $MODEL"
else
    check_warn "Not running on Raspberry Pi (hardware may not be available)"
fi
echo ""

# 4. Check server process
echo "4. Checking server process..."
if pgrep -f "main_lighting.go" > /dev/null; then
    PID=$(pgrep -f "main_lighting.go")
    check_pass "Server is running (PID: $PID)"
else
    check_fail "Server is NOT running"
fi
echo ""

# 5. Check port listening
echo "5. Checking port 8082..."
if netstat -tln 2>/dev/null | grep -q ":8082" || ss -tln 2>/dev/null | grep -q ":8082"; then
    check_pass "Port 8082 is listening"
    if command -v netstat >/dev/null 2>&1; then
        netstat -tln | grep ":8082" || true
    elif command -v ss >/dev/null 2>&1; then
        ss -tln | grep ":8082" || true
    fi
else
    check_fail "Port 8082 is NOT listening"
fi
echo ""

# 6. Check firewall
echo "6. Checking firewall..."
if command -v ufw >/dev/null 2>&1; then
    if ufw status | grep -q "8082"; then
        check_pass "Firewall rule for 8082 found"
    else
        check_warn "No firewall rule for 8082 (may need: sudo ufw allow 8082/tcp)"
    fi
elif command -v iptables >/dev/null 2>&1; then
    if iptables -L -n | grep -q "8082"; then
        check_pass "Firewall rule for 8082 found"
    else
        check_warn "No firewall rule for 8082 found"
    fi
else
    check_warn "Firewall tool not found (ufw/iptables)"
fi
echo ""

# 7. Check Tailscale
echo "7. Checking Tailscale..."
if command -v tailscale >/dev/null 2>&1; then
    if tailscale status >/dev/null 2>&1; then
        check_pass "Tailscale is running"
        TS_IP=$(tailscale ip -4 2>/dev/null || echo "unknown")
        echo "   Tailscale IP: $TS_IP"
    else
        check_fail "Tailscale is NOT running"
    fi
else
    check_warn "Tailscale not installed"
fi
echo ""

# 8. Test connectivity
echo "8. Testing connectivity..."
TS_IP=$(tailscale ip -4 2>/dev/null || echo "")
LOCAL_IP=$(hostname -I | awk '{print $1}' || echo "")

if [ -n "$TS_IP" ]; then
    if curl -s --max-time 2 "http://$TS_IP:8082/health" >/dev/null 2>&1; then
        check_pass "Health check via Tailscale IP ($TS_IP:8082) - OK"
    else
        check_fail "Health check via Tailscale IP ($TS_IP:8082) - FAILED"
    fi
fi

if [ -n "$LOCAL_IP" ]; then
    if curl -s --max-time 2 "http://$LOCAL_IP:8082/health" >/dev/null 2>&1; then
        check_pass "Health check via local IP ($LOCAL_IP:8082) - OK"
    else
        check_warn "Health check via local IP ($LOCAL_IP:8082) - FAILED (may be expected on remote access)"
    fi
fi
echo ""

# 9. Check recent server logs
echo "9. Recent server activity..."
if pgrep -f "main_lighting.go" > /dev/null; then
    echo "   Server is running - check logs for recent activity"
    echo "   Look for: CONNECTED, CMD Received, PONG messages"
else
    check_fail "Cannot check logs - server not running"
fi
echo ""

echo "================================"
echo "Diagnostics complete!"
echo ""
echo "💡 Next steps:"
echo "   - Review any ❌ failures above"
echo "   - Check server logs for connection attempts"
echo "   - Verify frontend is sending commands (browser console)"
echo "   - See TROUBLESHOOTING.md for detailed solutions"

