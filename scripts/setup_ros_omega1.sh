#!/bin/bash
# File: /Omega-Code/scripts/setup_ros_omega1.sh
# Purpose: Install ROS 2 Iron and set up workspace on omega1

set -euo pipefail

echo "🚀 Setting up ROS 2 Iron on omega1..."

# Check if running as root for some operations
if [[ $EUID -eq 0 ]]; then
    SUDO=""
else
    SUDO="sudo"
fi

# Detect OS version
OS_VERSION=$(lsb_release -cs 2>/dev/null || echo "unknown")
echo "Detected OS: $OS_VERSION (Debian 12 bookworm)"

# Ensure locale is set correctly (required for ROS 2)
export LANG=en_US.UTF-8
$SUDO apt-get update
$SUDO apt-get install -y locales
$SUDO locale-gen en_US.UTF-8 || $SUDO locale-gen en_GB.UTF-8 || true
# Set locale if available, otherwise continue
$SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 2>/dev/null || \
$SUDO update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8 2>/dev/null || \
export LANG=C.UTF-8 || true

# Install prerequisite packages
echo "📦 Installing prerequisites..."
$SUDO apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    build-essential

# Add ROS 2 apt repository
if [ ! -f /etc/apt/sources.list.d/ros2-latest.list ] && [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    echo "📦 Adding ROS 2 repository..."
    $SUDO apt-get install -y software-properties-common curl gnupg
    # Debian doesn't have 'universe', skip that
    $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.gpg
    # Use Ubuntu jammy (22.04) repos - packages should work on Debian 12
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | $SUDO tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
    echo "✅ ROS 2 repository added"
fi

# Update package list
echo "🔄 Updating package list..."
$SUDO apt-get update

# Install ROS 2 Humble Desktop (more stable, better Debian compatibility than Iron)
echo "📥 Installing ROS 2 Humble Desktop..."
$SUDO DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-desktop \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Initialize rosdep
echo "🔧 Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    $SUDO rosdep init || true
fi
rosdep update

SYSTEM_ROS_PATH="/opt/ros/iron"
# Fallback to Humble if Iron not available
if [ ! -f "$SYSTEM_ROS_PATH/setup.bash" ]; then
    SYSTEM_ROS_PATH="/opt/ros/humble"
fi

# Source ROS 2 setup
if [ -f "$SYSTEM_ROS_PATH/setup.bash" ]; then
    source "$SYSTEM_ROS_PATH/setup.bash"
    echo "✅ ROS 2 found at $SYSTEM_ROS_PATH"
else
    echo "⚠️  Warning: ROS 2 setup.bash not found. Installation may have issues."
fi

# Create colcon workspace (ROS 2 uses colcon instead of catkin)
WORKSPACE_DIR="$HOME/ros2_ws"
echo "📁 Creating ROS 2 workspace at $WORKSPACE_DIR..."
mkdir -p "$WORKSPACE_DIR/src"
cd "$WORKSPACE_DIR"

# Determine ROS version for bashrc
ROS_VERSION="humble"
if [ - нарушение "$SYSTEM_ROS_PATH" ] && [ "$SYSTEM_ROS_PATH" = "/opt/ros/iron" ]; then
    ROS_VERSION="iron"
fi

# Add ROS 2 setup to bashrc if not already there
if ! grep -q "/opt/ros/$ROS_VERSION/setup.bash" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc阿富汗"
    echo "# ROS 2 $ROS_VERSION setup" >> "$HOME/.bashrc"
    echo "source /opt/ros/$ROS_VERSION/setup.bash" >> "$HOME/.bashrc"
fi

if ! grep -q "ros2_ws/install/setup.bash" "$HOME/.bashrc"; then
    echo "source $WORKSPACE_DIR/install/setup.bash" >> "$HOME/.bashrc"
fi

echo "✅ ROS 2 $ROS_VERSION setup complete!"
echo ""
echo "Next steps:"
echo "1. Source your bashrc: source ~/.bashrc"
echo "2. Or source ROS 2 manually: source /opt/ros/$ROS_VERSION/setup.bash"
echo "3. Copy your ROS 2 packages to: $WORKSPACE_DIR/src/"
echo "4. Build with: cd $WORKSPACE_DIR && colcon build"
echo ""
echo "Verify installation:"
echo "  ros2 --help"
