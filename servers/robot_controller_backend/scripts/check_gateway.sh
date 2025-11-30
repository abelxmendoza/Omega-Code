#!/usr/bin/env bash
# Check if gateway service is running and diagnose issues

set -euo pipefail

PORT=${PORT:-7070}
HOST=${HOST:-localhost}

echo "🔍 Gateway Service Diagnostics (Port $PORT)"
echo "=============================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

check_pass() {
    echo -e "${GREEN}✅${NC} $1"
}

check_fail() {
    echo -e "${RED}❌${NC} $1"
}

check_warn() {
    echo -e "${YELLOW}⚠️${NC} $1"
}

# 1. Check if port is listening
echo "1. Checking if port $PORT is listening..."
if netstat -tln 2>/dev/null | grep -q ":$PORT " || ss -tln 2>/dev/null | grep -q ":$PORT "; then
    check_pass "Port $PORT is listening"
    if command -v netstat >/dev/null 2>&1; then
        netstat -tln | grep ":$PORT " || true
    elif command -v ss >/dev/null 2>&1; then
        ss -tln | grep ":$PORT " || true
    fi
else
    check_fail "Port $PORT is NOT listening (gateway not running)"
fi
echo ""

# 2. Check gateway process
echo "2. Checking gateway process..."
if pgrep -f "gateway_api" > /dev/null || pgrep -f "uvicorn.*gateway" > /dev/null; then
    PID=$(pgrep -f "gateway_api\|uvicorn.*gateway" | head -1)
    check_pass "Gateway process found (PID: $PID)"
else
    check_fail "Gateway process NOT found"
fi
echo ""

# 3. Test health endpoint
echo "3. Testing health endpoint..."
if curl -s --max-time 2 "http://$HOST:$PORT/health" >/dev/null 2>&1; then
    HEALTH=$(curl -s --max-time 2 "http://$HOST:$PORT/health")
    if [ "$HEALTH" = "ok" ]; then
        check_pass "Health endpoint responding: $HEALTH"
    else
        check_warn "Health endpoint returned: $HEALTH"
    fi
else
    check_fail "Health endpoint not responding"
fi
echo ""

# 4. Test performance metrics endpoint
echo "4. Testing performance metrics endpoint..."
if curl -s --max-time 2 "http://$HOST:$PORT/api/performance/metrics" >/dev/null 2>&1; then
    check_pass "Performance metrics endpoint responding"
else
    check_fail "Performance metrics endpoint not responding"
fi
echo ""

# 5. Check Python dependencies
echo "5. Checking Python dependencies..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -f "$BACKEND_DIR/venv/bin/python3" ]; then
    PYTHON_CMD="$BACKEND_DIR/venv/bin/python3"
    check_pass "Using venv Python: $PYTHON_CMD"
    
    if "$PYTHON_CMD" -c "import fastapi" 2>/dev/null; then
        check_pass "fastapi installed"
    else
        check_fail "fastapi NOT installed"
    fi
    
    if "$PYTHON_CMD" -c "import uvicorn" 2>/dev/null; then
        check_pass "uvicorn installed"
    else
        check_fail "uvicorn NOT installed"
    fi
else
    check_warn "Virtual environment not found"
fi
echo ""

# 6. Check environment variables
echo "6. Checking environment configuration..."
if [ -f "$BACKEND_DIR/.env" ]; then
    check_pass ".env file exists"
    if grep -q "DS_MOVE_WS\|DS_LIGHT_WS\|DS_ULTRA_WS\|DS_LINE_WS" "$BACKEND_DIR/.env" 2>/dev/null; then
        check_pass "Downstream WebSocket URLs configured"
    else
        check_warn "Downstream WebSocket URLs not configured (services may use mock/echo mode)"
    fi
else
    check_warn ".env file not found"
fi
echo ""

# 7. Check downstream services
echo "7. Checking downstream services..."
if [ -f "$BACKEND_DIR/.env" ]; then
    source "$BACKEND_DIR/.env" 2>/dev/null || true
    
    if [ -n "${DS_MOVE_WS:-}" ]; then
        DS_HOST=$(echo "$DS_MOVE_WS" | sed 's|ws://||; s|:.*||')
        DS_PORT=$(echo "$DS_MOVE_WS" | sed 's|.*:||; s|/.*||')
        if timeout 1 bash -c "echo > /dev/tcp/$DS_HOST/$DS_PORT" 2>/dev/null; then
            check_pass "Movement service ($DS_MOVE_WS) reachable"
        else
            check_warn "Movement service ($DS_MOVE_WS) not reachable"
        fi
    fi
    
    if [ -n "${DS_LIGHT_WS:-}" ]; then
        DS_HOST=$(echo "$DS_LIGHT_WS" | sed 's|ws://||; s|:.*||')
        DS_PORT=$(echo "$DS_LIGHT_WS" | sed 's|.*:||; s|/.*||')
        if timeout 1 bash -c "echo > /dev/tcp/$DS_HOST/$DS_PORT" 2>/dev/null; then
            check_pass "Lighting service ($DS_LIGHT_WS) reachable"
        else
            check_warn "Lighting service ($DS_LIGHT_WS) not reachable"
        fi
    fi
fi
echo ""

echo "=============================================="
echo "Diagnostics complete!"
echo ""
echo "💡 To start the gateway:"
echo "   cd $BACKEND_DIR"
echo "   ./scripts/run_gateway.sh"
echo ""
echo "💡 Or manually:"
echo "   source venv/bin/activate"
echo "   uvicorn servers.gateway_api:app --host 0.0.0.0 --port 7070"

