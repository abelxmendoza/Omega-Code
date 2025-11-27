#!/usr/bin/env bash
# Comprehensive integration verification script
# Tests frontend-backend-hardware connectivity
# Usage: ./scripts/verify_integration.sh [profile] [--ssh]

set -euo pipefail

PROFILE="${1:-tailscale}"
SKIP_SSH="${2:-}"
ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Load environment variables
set -o allexport
[ -f "$ROOT_DIR/servers/robot_controller_backend/.env" ] && source "$ROOT_DIR/servers/robot_controller_backend/.env"
[ -f "$ROOT_DIR/ui/robot-controller-ui/.env.local" ] && source "$ROOT_DIR/ui/robot-controller-ui/.env.local"
set +o allexport

# Determine host based on profile
case "$PROFILE" in
    tailscale)
        HOST="${OMEGA1_TAILSCALE_HOST:-${NEXT_PUBLIC_ROBOT_HOST_TAILSCALE:-100.93.225.61}}"
        ;;
    lan)
        HOST="${OMEGA1_LAN_HOST:-${NEXT_PUBLIC_ROBOT_HOST_LAN:-192.168.1.107}}"
        ;;
    local|*)
        HOST="localhost"
        ;;
esac

GATEWAY_PORT="${NEXT_PUBLIC_GATEWAY_PORT:-7070}"
GATEWAY_URL="http://${HOST}:${GATEWAY_PORT}"

echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Omega Robot Integration Verification${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "Profile: ${GREEN}$PROFILE${NC}"
echo -e "Host: ${GREEN}$HOST${NC}"
echo -e "Gateway: ${GREEN}$GATEWAY_URL${NC}"
echo ""

# Test counter
TESTS_PASSED=0
TESTS_FAILED=0

# Test function
test_endpoint() {
    local name="$1"
    local url="$2"
    local method="${3:-GET}"
    local expected_status="${4:-200}"
    
    echo -n "Testing $name... "
    
    if [ "$method" = "GET" ]; then
        HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" --max-time 5 "$url" || echo "000")
    else
        HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" -X "$method" --max-time 5 "$url" || echo "000")
    fi
    
    if [ "$HTTP_CODE" = "$expected_status" ]; then
        echo -e "${GREEN}✅ PASS${NC} (HTTP $HTTP_CODE)"
        ((TESTS_PASSED++))
        return 0
    else
        echo -e "${RED}❌ FAIL${NC} (HTTP $HTTP_CODE, expected $expected_status)"
        ((TESTS_FAILED++))
        return 1
    fi
}

# Test WebSocket connectivity
test_websocket() {
    local name="$1"
    local url="$2"
    
    echo -n "Testing $name WebSocket... "
    
    # Try to connect with timeout
    if command -v wscat >/dev/null 2>&1; then
        timeout 3 wscat -c "$url" >/dev/null 2>&1 && echo -e "${GREEN}✅ PASS${NC}" && ((TESTS_PASSED++)) && return 0 || echo -e "${RED}❌ FAIL${NC}" && ((TESTS_FAILED++)) && return 1
    else
        echo -e "${YELLOW}⚠️  SKIP${NC} (wscat not installed)"
        return 0
    fi
}

# Test TCP port
test_port() {
    local name="$1"
    local host="$2"
    local port="$3"
    
    echo -n "Testing $name port $port... "
    
    if command -v nc >/dev/null 2>&1; then
        if nc -z -w2 "$host" "$port" >/dev/null 2>&1; then
            echo -e "${GREEN}✅ PASS${NC}"
            ((TESTS_PASSED++))
            return 0
        else
            echo -e "${RED}❌ FAIL${NC}"
            ((TESTS_FAILED++))
            return 1
        fi
    else
        # Fallback to /dev/tcp
        if (echo >/dev/tcp/"$host"/"$port") >/dev/null 2>&1; then
            echo -e "${GREEN}✅ PASS${NC}"
            ((TESTS_PASSED++))
            return 0
        else
            echo -e "${RED}❌ FAIL${NC}"
            ((TESTS_FAILED++))
            return 1
        fi
    fi
}

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}1. Network Connectivity${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

# Ping test (skip if localhost)
if [ "$HOST" != "localhost" ]; then
    echo -n "Pinging $HOST... "
    if ping -c2 -W2 "$HOST" >/dev/null 2>&1; then
        echo -e "${GREEN}✅ PASS${NC}"
        ((TESTS_PASSED++))
    else
        echo -e "${YELLOW}⚠️  FAIL${NC} (may be blocked by firewall)"
        ((TESTS_FAILED++))
    fi
else
    echo -e "${YELLOW}⚠️  SKIP${NC} (localhost)"
fi

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}2. Gateway API Endpoints${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

test_endpoint "Gateway Health" "$GATEWAY_URL/health"
test_endpoint "Network Summary" "$GATEWAY_URL/api/net/summary"
test_endpoint "Performance Metrics" "$GATEWAY_URL/api/performance/metrics"
test_endpoint "System Info" "$GATEWAY_URL/api/performance/system"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}3. WebSocket Endpoints${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

# Convert http:// to ws://
WS_GATEWAY_URL=$(echo "$GATEWAY_URL" | sed 's|http://|ws://|')

test_websocket "Movement" "${WS_GATEWAY_URL}/ws/movement"
test_websocket "Ultrasonic" "${WS_GATEWAY_URL}/ws/ultrasonic"
test_websocket "Line Tracker" "${WS_GATEWAY_URL}/ws/line"
test_websocket "Lighting" "${WS_GATEWAY_URL}/ws/lighting"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}4. Service Ports${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

test_port "Gateway" "$HOST" "$GATEWAY_PORT"
test_port "Video Server" "$HOST" "${VIDEO_PORT:-5000}"
test_port "Movement WS" "$HOST" "${PORT_MOVEMENT:-8081}"
test_port "Ultrasonic WS" "$HOST" "${PORT_ULTRASONIC:-8080}"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}5. ROS2 Integration${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

test_endpoint "ROS2 Status" "$GATEWAY_URL/api/ros/status"
test_endpoint "ROS2 Topics" "$GATEWAY_URL/api/ros/topics"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}6. Hardware Status${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

# Get performance metrics which includes hardware status
echo -n "Fetching hardware status... "
HW_STATUS=$(curl -s --max-time 5 "$GATEWAY_URL/api/performance/metrics" 2>/dev/null || echo "{}")

if [ "$HW_STATUS" != "{}" ]; then
    echo -e "${GREEN}✅ PASS${NC}"
    echo ""
    echo "Hardware Status Summary:"
    echo "$HW_STATUS" | python3 -m json.tool 2>/dev/null | grep -E "(piSpecific|robotTelemetry)" -A 10 || echo "$HW_STATUS"
    ((TESTS_PASSED++))
else
    echo -e "${RED}❌ FAIL${NC}"
    ((TESTS_FAILED++))
fi

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Test Summary${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "Tests Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Tests Failed: ${RED}$TESTS_FAILED${NC}"
echo ""

if [ "$TESTS_FAILED" -eq 0 ]; then
    echo -e "${GREEN}✅ All tests passed! Integration is working correctly.${NC}"
    exit 0
else
    echo -e "${RED}❌ Some tests failed. Please check the services on $HOST${NC}"
    echo ""
    echo "Troubleshooting steps:"
    echo "1. Ensure backend services are running on omega1:"
    echo "   - Gateway API (port $GATEWAY_PORT)"
    echo "   - Movement WebSocket (port ${PORT_MOVEMENT:-8081})"
    echo "   - Video Server (port ${VIDEO_PORT:-5000})"
    echo ""
    echo "2. Check environment variables:"
    echo "   - Backend: servers/robot_controller_backend/.env"
    echo "   - Frontend: ui/robot-controller-ui/.env.local"
    echo ""
    echo "3. SSH into omega1 and check services:"
    echo "   ./scripts/ssh_omega1.sh $PROFILE"
    exit 1
fi

