#!/bin/bash
#
# Omega Profile Application Script
#
# Detects system capabilities and applies appropriate profile.
# Can be run standalone or called by launch files.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TEMP_DIR="${TMPDIR:-/tmp}"
CAPABILITY_FILE="$TEMP_DIR/omega_capabilities.json"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔍 Omega Capability Detection${NC}"
echo "================================"

# Detect Jetson
is_jetson() {
    if [ -f /proc/device-tree/model ]; then
        grep -qi "NVIDIA\|Jetson" /proc/device-tree/model && return 0
    fi
    [ -f /sys/devices/soc0/family ] && grep -qi "Tegra" /sys/devices/soc0/family && return 0
    [ -n "$JETSON" ] || [ -n "$JETSON_MODEL" ] && return 0
    return 1
}

# Check CUDA
check_cuda() {
    command -v nvcc >/dev/null 2>&1 && return 0
    python3 -c "import torch; exit(0 if torch.cuda.is_available() else 1)" 2>/dev/null && return 0
    return 1
}

# Check ROS2
check_ros2() {
    command -v ros2 >/dev/null 2>&1 && return 0
    return 1
}

# Detect OS
detect_os() {
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "mac"
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if is_jetson; then
            echo "jetson"
        else
            echo "linux"
        fi
    else
        echo "unknown"
    fi
}

# Main detection
OS_TYPE=$(detect_os)
IS_JETSON=false
HAS_CUDA=false
HAS_ROS2=false

if is_jetson; then
    IS_JETSON=true
    echo -e "${GREEN}✅ Jetson hardware detected${NC}"
fi

if check_cuda; then
    HAS_CUDA=true
    echo -e "${GREEN}✅ CUDA available${NC}"
else
    echo -e "${YELLOW}⚠️  CUDA not available${NC}"
fi

if check_ros2; then
    HAS_ROS2=true
    echo -e "${GREEN}✅ ROS2 tools available${NC}"
else
    echo -e "${YELLOW}⚠️  ROS2 tools not found${NC}"
fi

# Determine profile
PROFILE_MODE="mac"
ML_CAPABLE=false
SLAM_CAPABLE=false
MAX_RES="640x480"
MAX_FPS=30

if [ "$IS_JETSON" = true ] && [ "$HAS_CUDA" = true ]; then
    PROFILE_MODE="jetson"
    ML_CAPABLE=true
    SLAM_CAPABLE=true
    MAX_RES="1920x1080"
    MAX_FPS=60
    echo -e "${GREEN}🎯 Profile: OMEGA MODE (Jetson)${NC}"
elif [ "$OS_TYPE" = "linux" ] && [ "$HAS_ROS2" = true ]; then
    PROFILE_MODE="lenovo"
    SLAM_CAPABLE=true
    MAX_RES="1280x720"
    MAX_FPS=25
    echo -e "${YELLOW}🎯 Profile: DEV MODE (Lenovo Linux)${NC}"
else
    PROFILE_MODE="mac"
    MAX_RES="640x480"
    MAX_FPS=20
    echo -e "${BLUE}🎯 Profile: LIGHT MODE (MacBook)${NC}"
fi

# Create capability JSON
cat > "$CAPABILITY_FILE" <<EOF
{
  "device": "$(hostname)",
  "arch": "$(uname -m)",
  "os": "$OS_TYPE",
  "is_jetson": $IS_JETSON,
  "cuda": $HAS_CUDA,
  "ros2_dev": $HAS_ROS2,
  "ml_capable": $ML_CAPABLE,
  "slam_capable": $SLAM_CAPABLE,
  "tracking": true,
  "aruco": true,
  "motion_detection": true,
  "face_recognition": $ML_CAPABLE,
  "yolo": $ML_CAPABLE,
  "max_resolution": "$MAX_RES",
  "max_fps": $MAX_FPS,
  "profile_mode": "$PROFILE_MODE",
  "gpu_available": $HAS_CUDA,
  "cpu_count": $(nproc 2>/dev/null || echo 1)
}
EOF

echo ""
echo -e "${GREEN}✅ Capability profile saved to: $CAPABILITY_FILE${NC}"
echo ""
echo "Profile Summary:"
echo "  Mode: $PROFILE_MODE"
echo "  ML Capable: $ML_CAPABLE"
echo "  SLAM Capable: $SLAM_CAPABLE"
echo "  Max Resolution: $MAX_RES"
echo "  Max FPS: $MAX_FPS"
echo ""

# Export for use in other scripts
export OMEGA_PROFILE="$PROFILE_MODE"
export OMEGA_CAPABILITY_FILE="$CAPABILITY_FILE"

echo -e "${BLUE}📋 Profile applied. Ready to launch ROS2 nodes.${NC}"

