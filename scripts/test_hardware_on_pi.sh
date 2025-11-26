#!/usr/bin/env bash
# Hardware test script to run on omega1 (Raspberry Pi)
# Tests GPIO, sensors, motors, and camera connectivity
# Usage: Run this script directly on the Pi

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Omega Robot Hardware Test${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""

TESTS_PASSED=0
TESTS_FAILED=0

test_check() {
    local name="$1"
    local command="$2"
    
    echo -n "Testing $name... "
    if eval "$command" >/dev/null 2>&1; then
        echo -e "${GREEN}✅ PASS${NC}"
        ((TESTS_PASSED++))
        return 0
    else
        echo -e "${RED}❌ FAIL${NC}"
        ((TESTS_FAILED++))
        return 1
    fi
}

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}1. System Information${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

echo "Hostname: $(hostname)"
echo "OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
echo "Kernel: $(uname -r)"
echo "Architecture: $(uname -m)"
echo ""

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}2. GPIO Access${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

test_check "GPIO device exists" "[ -e /dev/gpiomem ]"
test_check "GPIO permissions" "[ -r /dev/gpiomem ] && [ -w /dev/gpiomem ]"

# Check if user is in gpio group
if groups | grep -q gpio; then
    echo -e "User in gpio group: ${GREEN}✅ YES${NC}"
    ((TESTS_PASSED++))
else
    echo -e "User in gpio group: ${RED}❌ NO${NC} (run: sudo usermod -a -G gpio \$USER)"
    ((TESTS_FAILED++))
fi

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}3. Python GPIO Libraries${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

test_check "lgpio available" "python3 -c 'import lgpio'"
test_check "RPi.GPIO available" "python3 -c 'import RPi.GPIO' 2>/dev/null || python3 -c 'import gpiozero'"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}4. Camera${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

if command -v vcgencmd >/dev/null 2>&1; then
    CAMERA_STATUS=$(vcgencmd get_camera 2>/dev/null || echo "detected=0")
    if echo "$CAMERA_STATUS" | grep -q "detected=1"; then
        echo -e "Camera detected: ${GREEN}✅ YES${NC}"
        ((TESTS_PASSED++))
    else
        echo -e "Camera detected: ${RED}❌ NO${NC}"
        ((TESTS_FAILED++))
    fi
else
    echo -e "vcgencmd: ${YELLOW}⚠️  NOT AVAILABLE${NC}"
fi

test_check "libcamera available" "command -v libcamera-hello >/dev/null 2>&1 || command -v raspistill >/dev/null 2>&1"
test_check "picamera2 available" "python3 -c 'import picamera2' 2>/dev/null || python3 -c 'from picamera2 import Picamera2' 2>/dev/null"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}5. I2C/SPI Interfaces${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

test_check "I2C device exists" "[ -e /dev/i2c-1 ]"
test_check "SPI device exists" "[ -e /dev/spidev0.0 ] || [ -e /dev/spidev0.1 ]"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}6. Backend Services${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

test_check "Gateway API running" "curl -s http://localhost:7070/health | grep -q ok"
test_check "Movement WS port open" "nc -z localhost 8081 2>/dev/null || (echo >/dev/tcp/localhost/8081) 2>/dev/null"
test_check "Video server port open" "nc -z localhost 5000 2>/dev/null || (echo >/dev/tcp/localhost/5000) 2>/dev/null"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}7. Python Dependencies${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

test_check "websockets library" "python3 -c 'import websockets'"
test_check "fastapi library" "python3 -c 'import fastapi'"
test_check "uvicorn library" "python3 -c 'import uvicorn'"
test_check "opencv library" "python3 -c 'import cv2' 2>/dev/null"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}8. Network Connectivity${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

# Get IP addresses
LAN_IP=$(hostname -I | awk '{print $1}')
TAILSCALE_IP=$(ip addr show tailscale0 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1 || echo "Not configured")

echo "LAN IP: $LAN_IP"
echo "Tailscale IP: ${TAILSCALE_IP:-Not configured}"

test_check "Internet connectivity" "ping -c1 -W2 8.8.8.8 >/dev/null 2>&1"

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Test Summary${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "Tests Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Tests Failed: ${RED}$TESTS_FAILED${NC}"
echo ""

if [ "$TESTS_FAILED" -eq 0 ]; then
    echo -e "${GREEN}✅ All hardware tests passed!${NC}"
    exit 0
else
    echo -e "${RED}❌ Some hardware tests failed.${NC}"
    echo ""
    echo "Common fixes:"
    echo "1. Add user to gpio group: sudo usermod -a -G gpio \$USER"
    echo "2. Enable camera: sudo raspi-config -> Interface Options -> Camera"
    echo "3. Enable I2C/SPI: sudo raspi-config -> Interface Options"
    echo "4. Install missing Python packages: pip3 install -r requirements.txt"
    exit 1
fi

