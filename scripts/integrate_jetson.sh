#!/bin/bash
# integrate_jetson.sh
# Helper script to integrate Jetson Orin Nano into Omega-Code
# Run this from your laptop/MacBook

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Jetson configuration
JETSON_IP="100.107.112.110"
JETSON_USER="omega1"
JETSON_HOST="${JETSON_USER}@${JETSON_IP}"

echo "🚀 Jetson Orin Nano Integration Helper"
echo "======================================"
echo ""
echo "Jetson: $JETSON_HOST"
echo ""

# Test SSH connection
echo "🔍 Testing SSH connection..."
if ssh -o ConnectTimeout=5 -o BatchMode=yes "$JETSON_HOST" "echo 'SSH OK'" 2>/dev/null; then
    echo "✅ SSH connection successful"
else
    echo "❌ SSH connection failed"
    echo ""
    echo "Please ensure:"
    echo "  1. Tailscale is connected"
    echo "  2. SSH keys are set up: ssh-copy-id $JETSON_HOST"
    echo "  3. SSH service is running on Jetson"
    echo ""
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Get Jetson system info
echo ""
echo "📊 Jetson System Information:"
ssh "$JETSON_HOST" "uname -a && echo '---' && hostname && echo '---' && nvcc --version 2>/dev/null || echo 'CUDA not found'"

# Check if Omega-Code exists on Jetson
echo ""
echo "🔍 Checking Omega-Code on Jetson..."
if ssh "$JETSON_HOST" "[ -d ~/Omega-Code ]"; then
    echo "✅ Omega-Code directory found"
    JETSON_HAS_CODE=true
else
    echo "⚠️  Omega-Code not found on Jetson"
    read -p "Copy Omega-Code to Jetson? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "📦 Copying Omega-Code to Jetson..."
        rsync -avz --exclude 'venv' --exclude 'node_modules' --exclude '.git' \
            "$PROJECT_ROOT/" "$JETSON_HOST:~/Omega-Code/"
        echo "✅ Copy complete"
        JETSON_HAS_CODE=true
    else
        JETSON_HAS_CODE=false
    fi
fi

# Copy CycloneDDS config
echo ""
echo "📡 Copying CycloneDDS configuration..."
CYCLONEDDS_CONFIG="$PROJECT_ROOT/docker/ros2_robot/config/cyclonedds.xml"
if [ -f "$CYCLONEDDS_CONFIG" ]; then
    ssh "$JETSON_HOST" "mkdir -p ~/omega_ws/config"
    scp "$CYCLONEDDS_CONFIG" "$JETSON_HOST:~/omega_ws/config/"
    echo "✅ CycloneDDS config copied"
else
    echo "⚠️  CycloneDDS config not found at $CYCLONEDDS_CONFIG"
fi

# Run setup script on Jetson
if [ "$JETSON_HAS_CODE" = true ]; then
    echo ""
    read -p "Run Jetson setup script? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "🛠️  Running setup script on Jetson..."
        ssh "$JETSON_HOST" "cd ~/Omega-Code && chmod +x scripts/setup_jetson_orin.sh && ./scripts/setup_jetson_orin.sh"
    fi
fi

# Test ROS2 communication
echo ""
read -p "Test ROS2 communication? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🧪 Testing ROS2 communication..."
    echo ""
    echo "Terminal 1: Run this on Jetson:"
    echo "  ssh $JETSON_HOST"
    echo "  source ~/.ros2_jetson_setup.bash"
    echo "  ros2 run demo_nodes_cpp listener"
    echo ""
    echo "Terminal 2: Run this on your laptop:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  export ROS_DOMAIN_ID=0"
    echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    echo "  export CYCLONEDDS_URI=file://$PROJECT_ROOT/docker/ros2_robot/config/cyclonedds.xml"
    echo "  ros2 run demo_nodes_cpp talker"
    echo ""
fi

# Summary
echo ""
echo "======================================"
echo "✅ Integration helper complete!"
echo ""
echo "📋 Next steps:"
echo ""
echo "1. Review integration guide:"
echo "   cat $PROJECT_ROOT/JETSON_ORIN_INTEGRATION.md"
echo ""
echo "2. Test SSH connection:"
echo "   ssh $JETSON_HOST"
echo ""
echo "3. Run setup on Jetson:"
echo "   ssh $JETSON_HOST"
echo "   cd ~/Omega-Code"
echo "   ./scripts/setup_jetson_orin.sh"
echo ""
echo "4. Build workspace on Jetson:"
echo "   ssh $JETSON_HOST"
echo "   cd ~/omega_ws"
echo "   colcon build --symlink-install"
echo ""
echo "======================================"

