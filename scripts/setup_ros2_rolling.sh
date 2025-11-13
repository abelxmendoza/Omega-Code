#!/bin/bash
# setup_ros2_rolling.sh
# ROS2 Rolling setup script for Ubuntu laptop (development cockpit)
# Assumes ROS2 Rolling is already installed

set -e

echo "🚀 Setting up ROS2 Rolling on Ubuntu Laptop (Development Cockpit)"
echo "=================================================================="

# Check if ROS2 Rolling is installed
if [ ! -f /opt/ros/rolling/setup.bash ]; then
    echo "❌ ROS2 Rolling not found at /opt/ros/rolling"
    echo "   Please install ROS2 Rolling first"
    exit 1
fi

echo "✅ ROS2 Rolling found"

# Install additional packages if needed
echo ""
echo "📦 Checking for additional ROS2 packages..."
sudo apt update
sudo apt install -y \
    ros-rolling-rmw-cyclonedds-cpp \
    ros-rolling-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool || echo "⚠️  Some packages may already be installed"

# Initialize rosdep if needed
echo ""
echo "🔧 Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || echo "⚠️  rosdep may already be initialized"
fi
rosdep update || echo "⚠️  rosdep update may have failed (continuing...)"

# Setup environment
echo ""
echo "⚙️  Setting up environment..."
ROS_SETUP_FILE="$HOME/.ros2_rolling_setup.bash"
cat > "$ROS_SETUP_FILE" << 'EOF'
#!/bin/bash
# ROS2 Rolling environment setup for laptop

# Clean up any Jazzy environment variables first
unset ROS_DISTRO
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset SANDBOX_LD_LIBRARY_PATH

# Source ROS2 Rolling (this will set ROS_DISTRO=rolling)
source /opt/ros/rolling/setup.bash

# Clean up any other workspace paths that might have Jazzy
unset COLCON_PREFIX_PATH

# Source workspace if it exists (only omega_ws, not other workspaces)
if [ -f "$HOME/omega_ws/install/setup.bash" ]; then
    # Source only omega_ws, not other workspaces that might have Jazzy
    source "$HOME/omega_ws/install/setup.bash"
fi

# Set ROS2 domain ID (default: 0)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Use CycloneDDS for multi-device communication (if installed)
# If CycloneDDS is not installed, use default RMW
if [ -f "/opt/ros/rolling/lib/librmw_cyclonedds_cpp.so" ] || dpkg -l | grep -q "ros-rolling-rmw-cyclonedds"; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
else
    echo "⚠️  CycloneDDS RMW not installed, using default RMW"
    # Don't set RMW_IMPLEMENTATION, let ROS2 use default
fi

# Set CycloneDDS config (update path as needed)
if [ -f "$HOME/Omega-Code/docker/ros2_robot/config/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file://$HOME/Omega-Code/docker/ros2_robot/config/cyclonedds.xml
fi

# Optimize for laptop (light RViz settings)
export OGRE_RTT_MODE=Copy
export LIBGL_ALWAYS_SOFTWARE=0

echo "✅ ROS2 Rolling environment loaded"
echo "   Domain ID: $ROS_DOMAIN_ID"
echo "   RMW: $RMW_IMPLEMENTATION"
EOF

chmod +x "$ROS_SETUP_FILE"

# Add to bashrc if not already present
if ! grep -q "ros2_rolling_setup.bash" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc"
    echo "# ROS2 Rolling setup" >> "$HOME/.bashrc"
    echo "source $ROS_SETUP_FILE" >> "$HOME/.bashrc"
    echo "Added ROS2 Rolling setup to ~/.bashrc"
fi

# Create workspace directory
echo ""
echo "📁 Creating ROS2 workspace..."
mkdir -p "$HOME/omega_ws/src"
cd "$HOME/omega_ws"

# Link Omega-Code ROS packages if not already present
if [ ! -d "$HOME/omega_ws/src/omega_robot" ]; then
    echo "📥 Linking Omega-Code ROS packages..."
    if [ -d "$HOME/Omega-Code/ros/src/omega_robot" ]; then
        ln -s "$HOME/Omega-Code/ros/src/omega_robot" "$HOME/omega_ws/src/omega_robot"
        echo "✅ Linked Omega-Code ROS packages"
    else
        echo "⚠️  Warning: Omega-Code ROS packages not found at ~/Omega-Code/ros/src/omega_robot"
    fi
fi

# Build workspace
echo ""
echo "🔨 Building ROS2 workspace..."
source /opt/ros/rolling/setup.bash
if [ -d "$HOME/omega_ws/src/omega_robot" ]; then
    cd "$HOME/omega_ws"
    colcon build --symlink-install
    echo "✅ Workspace built successfully"
else
    echo "⚠️  Skipping build - no packages found"
fi

# Summary
echo ""
echo "=================================================================="
echo "✅ ROS2 Rolling setup complete!"
echo ""
echo "📋 Next steps:"
echo "   1. Source the environment:"
echo "      source $ROS_SETUP_FILE"
echo "      # or restart your terminal"
echo ""
echo "   2. Update CycloneDDS config with device IPs:"
echo "      ~/Omega-Code/docker/ros2_robot/config/cyclonedds.xml"
echo ""
echo "   3. Set device IPs in environment:"
echo "      export LAPTOP_IP=\$(hostname -I | awk '{print \$1}')"
echo "      export PI_IP=192.168.1.107"
echo "      export ORIN_IP=192.168.1.XXX"
echo ""
echo "   4. Test ROS2:"
echo "      ros2 run demo_nodes_cpp talker"
echo "      # In another terminal:"
echo "      ros2 run demo_nodes_cpp listener"
echo ""
echo "   5. Launch RViz:"
echo "      rviz2"
echo ""
echo "   6. Build Omega workspace:"
echo "      cd ~/omega_ws && colcon build"
echo "=================================================================="

