#!/bin/bash
# quick_start_ros2.sh
# Quick start script for ROS2 multi-device development
# Usage: ./scripts/quick_start_ros2.sh [laptop|pi|orin]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DEVICE_TYPE=${1:-laptop}

echo "🚀 Quick Start: ROS2 Multi-Device Setup"
echo "========================================="
echo "Device: $DEVICE_TYPE"
echo ""

# Check if .env exists
ENV_FILE="$PROJECT_ROOT/.env.ros2.multidevice"
if [ ! -f "$ENV_FILE" ]; then
    echo "📝 Creating environment file..."
    if [ -f "$PROJECT_ROOT/.env.ros2.multidevice.example" ]; then
        cp "$PROJECT_ROOT/.env.ros2.multidevice.example" "$ENV_FILE"
        echo "✅ Created $ENV_FILE"
        echo ""
        echo "⚠️  Please edit $ENV_FILE with your device IPs:"
        echo "   - LAPTOP_IP"
        echo "   - PI_IP"
        echo "   - ORIN_IP"
        echo ""
        read -p "Press Enter after updating the file..."
    else
        echo "❌ .env.ros2.multidevice.example not found"
        exit 1
    fi
fi

# Source environment
source "$ENV_FILE"

# Set ROS2 environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

CYCLONEDDS_CONFIG="$PROJECT_ROOT/docker/ros2_robot/config/cyclonedds.xml"
if [ -f "$CYCLONEDDS_CONFIG" ]; then
    export CYCLONEDDS_URI=file://$CYCLONEDDS_CONFIG
else
    echo "⚠️  CycloneDDS config not found. Run setup_multidevice_ros2.sh first."
    exit 1
fi

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ ROS2 Humble not installed. Run setup_ros2_laptop.sh first."
    exit 1
fi

# Source workspace if it exists
if [ -f "$HOME/omega_ws/install/setup.bash" ]; then
    source "$HOME/omega_ws/install/setup.bash"
fi

echo "✅ ROS2 environment loaded"
echo "   Domain ID: $ROS_DOMAIN_ID"
echo "   RMW: $RMW_IMPLEMENTATION"
echo ""

# Device-specific actions
case $DEVICE_TYPE in
    laptop)
        echo "🖥️  Laptop Mode - Development Cockpit"
        echo ""
        echo "Available commands:"
        echo "  ros2 topic list              # List all topics"
        echo "  ros2 node list               # List all nodes"
        echo "  ros2 topic echo /omega/telemetry  # Monitor telemetry"
        echo "  rviz2                        # Launch RViz"
        echo "  ros2 launch omega_robot multidevice_setup.launch.py  # Launch multi-device setup"
        echo ""
        echo "Quick test:"
        echo "  Terminal 1: ros2 run demo_nodes_cpp talker"
        echo "  Terminal 2: ros2 run demo_nodes_cpp listener"
        ;;
    
    pi)
        echo "🍓 Pi 4B Mode - Hardware Controller"
        echo ""
        echo "Available commands:"
        echo "  ros2 run omega_robot telemetry_publisher  # Publish telemetry"
        echo "  ros2 topic list                           # List topics"
        echo ""
        echo "Note: Run publisher on Pi, listener on laptop"
        ;;
    
    orin)
        echo "🚀 Orin Nano Mode - AI Compute"
        echo ""
        echo "Available commands:"
        echo "  ros2 run omega_robot vision_processor  # Run vision processing"
        echo "  ros2 topic list                        # List topics"
        echo ""
        echo "Note: Run vision nodes on Orin, monitor on laptop"
        ;;
    
    *)
        echo "❌ Unknown device type: $DEVICE_TYPE"
        echo "Usage: $0 [laptop|pi|orin]"
        exit 1
        ;;
esac

echo ""
echo "========================================="
echo "✅ Ready for ROS2 development!"
echo ""

