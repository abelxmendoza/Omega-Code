#!/bin/bash
# setup_multidevice_ros2.sh
# Complete multi-device ROS2 setup script
# Configures Laptop, Pi 4B, and Jetson Orin Nano for distributed ROS2

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "🚀 Multi-Device ROS2 Setup for Omega-1"
echo "========================================"
echo ""
echo "This script will configure:"
echo "  - Laptop (Ubuntu): Development cockpit"
echo "  - Pi 4B: Hardware IO controller"
echo "  - Jetson Orin Nano: AI compute engine"
echo ""

# Check if .env.ros2.multidevice exists
ENV_FILE="$PROJECT_ROOT/.env.ros2.multidevice"
if [ ! -f "$ENV_FILE" ]; then
    echo "📝 Creating environment configuration..."
    if [ -f "$PROJECT_ROOT/.env.ros2.multidevice.example" ]; then
        cp "$PROJECT_ROOT/.env.ros2.multidevice.example" "$ENV_FILE"
        echo "✅ Created $ENV_FILE"
        echo "⚠️  Please edit this file with your device IPs before continuing"
        read -p "Press Enter after updating the file..."
    else
        echo "❌ .env.ros2.multidevice.example not found"
        exit 1
    fi
fi

# Source environment
source "$ENV_FILE"

# Detect current device
CURRENT_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "📍 Current device IP: $CURRENT_IP"

# Determine which device we're setting up
DEVICE_TYPE="unknown"
if [ "$CURRENT_IP" = "$LAPTOP_IP" ] || [ -n "$DISPLAY" ]; then
    DEVICE_TYPE="laptop"
    echo "🖥️  Detected: Laptop (Development Cockpit)"
elif [ "$CURRENT_IP" = "$PI_IP" ] || hostname | grep -qi "raspberry\|pi"; then
    DEVICE_TYPE="pi"
    echo "🍓 Detected: Raspberry Pi 4B (Hardware Controller)"
elif [ "$CURRENT_IP" = "$ORIN_IP" ] || hostname | grep -qi "jetson\|orin"; then
    DEVICE_TYPE="orin"
    echo "🚀 Detected: Jetson Orin Nano (AI Compute)"
else
    echo "⚠️  Could not auto-detect device type"
    read -p "Device type (laptop/pi/orin): " DEVICE_TYPE
fi

# Setup based on device type
case $DEVICE_TYPE in
    laptop)
        echo ""
        echo "🔧 Setting up Laptop..."
        
        # Run laptop setup script
        if [ -f "$SCRIPT_DIR/setup_ros2_laptop.sh" ]; then
            bash "$SCRIPT_DIR/setup_ros2_laptop.sh"
        else
            echo "❌ setup_ros2_laptop.sh not found"
            exit 1
        fi
        
        # Run optimization
        if [ -f "$SCRIPT_DIR/optimize_laptop_ros2.sh" ]; then
            echo ""
            read -p "Run laptop optimization? (y/N): " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                bash "$SCRIPT_DIR/optimize_laptop_ros2.sh"
            fi
        fi
        ;;
    
    pi)
        echo ""
        echo "🔧 Setting up Raspberry Pi 4B..."
        echo "⚠️  Pi setup script not yet implemented"
        echo "   Please run setup_ros2_laptop.sh on Pi (it works for Ubuntu-based Pi OS)"
        ;;
    
    orin)
        echo ""
        echo "🔧 Setting up Jetson Orin Nano..."
        echo "⚠️  Orin setup script not yet implemented"
        echo "   Please run setup_ros2_laptop.sh on Orin (it works for Ubuntu-based Orin OS)"
        ;;
    
    *)
        echo "❌ Unknown device type: $DEVICE_TYPE"
        exit 1
        ;;
esac

# Generate CycloneDDS config
echo ""
echo "📝 Generating CycloneDDS configuration..."
CYCLONEDDS_DIR="$PROJECT_ROOT/docker/ros2_robot/config"
mkdir -p "$CYCLONEDDS_DIR"

CYCLONEDDS_CONFIG="$CYCLONEDDS_DIR/cyclonedds.xml"
if [ -f "$CYCLONEDDS_DIR/cyclonedds.xml.template" ]; then
    envsubst < "$CYCLONEDDS_DIR/cyclonedds.xml.template" > "$CYCLONEDDS_CONFIG"
    echo "✅ Generated $CYCLONEDDS_CONFIG"
    echo "   Laptop: $LAPTOP_IP"
    echo "   Pi 4B: $PI_IP"
    echo "   Orin: $ORIN_IP"
else
    echo "⚠️  Template not found, creating basic config..."
    cat > "$CYCLONEDDS_CONFIG" << EOF
<CycloneDDS>
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="$LAPTOP_IP"/>
        <Peer address="$PI_IP"/>
        <Peer address="$ORIN_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
    echo "✅ Created $CYCLONEDDS_CONFIG"
fi

# Create workspace symlink if on laptop
if [ "$DEVICE_TYPE" = "laptop" ]; then
    echo ""
    echo "📁 Setting up workspace..."
    if [ ! -d "$HOME/omega_ws/src/omega_robot" ]; then
        mkdir -p "$HOME/omega_ws/src"
        if [ -d "$PROJECT_ROOT/ros/src/omega_robot" ]; then
            ln -s "$PROJECT_ROOT/ros/src/omega_robot" "$HOME/omega_ws/src/omega_robot"
            echo "✅ Linked Omega-Code packages to workspace"
        fi
    fi
fi

# Summary
echo ""
echo "========================================"
echo "✅ Multi-device ROS2 setup complete!"
echo ""
echo "📋 Next steps:"
echo ""
echo "1. Verify CycloneDDS config:"
echo "   cat $CYCLONEDDS_CONFIG"
echo ""
echo "2. Set environment variables:"
echo "   source $ENV_FILE"
echo "   export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "   export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "   export CYCLONEDDS_URI=file://$CYCLONEDDS_CONFIG"
echo ""
if [ "$DEVICE_TYPE" = "laptop" ]; then
    echo "3. Build workspace:"
    echo "   cd ~/omega_ws && colcon build"
    echo ""
    echo "4. Test ROS2 communication:"
    echo "   # Terminal 1:"
    echo "   ros2 run demo_nodes_cpp talker"
    echo "   # Terminal 2 (on Pi or Orin):"
    echo "   ros2 run demo_nodes_cpp listener"
    echo ""
    echo "5. Launch multi-device setup:"
    echo "   ros2 launch omega_robot multidevice_setup.launch.py"
fi
echo ""
echo "6. Copy CycloneDDS config to other devices:"
echo "   scp $CYCLONEDDS_CONFIG $PI_SSH_USER@$PI_IP:~/omega_ws/config/"
echo "   scp $CYCLONEDDS_CONFIG $ORIN_SSH_USER@$ORIN_IP:~/omega_ws/config/"
echo ""
echo "========================================"

