#!/bin/bash
# setup_jetson_orin.sh
# Complete setup script for Jetson Orin Nano
# Installs ROS2 Humble, CUDA tools, and configures for AI/vision workloads

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "🚀 Jetson Orin Nano Setup for Omega-Code"
echo "========================================="
echo ""
echo "This script will configure your Jetson Orin Nano for:"
echo "  - ROS2 Humble (AI/vision processing)"
echo "  - CUDA acceleration"
echo "  - Multi-device ROS2 communication"
echo ""

# Check if running on Jetson
if ! grep -qi "jetson\|orin" /proc/device-tree/model 2>/dev/null; then
    echo "⚠️  Warning: This script is designed for Jetson devices"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Detect Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    UBUNTU_VERSION=$VERSION_ID
    echo "Detected Ubuntu version: $UBUNTU_VERSION"
else
    echo "❌ Cannot detect Ubuntu version"
    exit 1
fi

# Check CUDA
echo ""
echo "🔍 Checking CUDA installation..."
if command -v nvcc &> /dev/null; then
    CUDA_VERSION=$(nvcc --version | grep "release" | awk '{print $5}' | cut -d',' -f1)
    echo "✅ CUDA found: $CUDA_VERSION"
else
    echo "⚠️  CUDA not found. Jetson devices should have CUDA pre-installed."
    echo "   If missing, install JetPack SDK"
fi

# Check if ROS2 is already installed
if [ -f /opt/ros/humble/setup.bash ]; then
    echo ""
    echo "✅ ROS2 Humble is already installed"
    read -p "Reinstall ROS2? (y/N): " -n 1 -r
    echo
    REINSTALL_ROS2=$REPLY
else
    REINSTALL_ROS2="y"
fi

# Install prerequisites
echo ""
echo "📦 Installing prerequisites..."
sudo apt update
sudo apt install -y \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    wget \
    python3-pip \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-opencv \
    libopencv-dev

# Install CUDA development tools (if not present)
if ! command -v nvcc &> /dev/null; then
    echo ""
    echo "📦 Installing CUDA development tools..."
    sudo apt install -y \
        nvidia-cuda-toolkit \
        cuda-toolkit-12-0
fi

# Add ROS2 repository
echo ""
echo "📥 Adding ROS2 Humble repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble
if [[ $REINSTALL_ROS2 =~ ^[Yy]$ ]]; then
    echo ""
    echo "📦 Installing ROS2 Humble..."
    sudo apt update
    sudo apt install -y \
        ros-humble-desktop \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-vision-msgs \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs
fi

# Install colcon build tools
echo ""
echo "🔨 Installing colcon build tools..."
sudo apt install -y python3-colcon-common-extensions

# Initialize rosdep
echo ""
echo "🔧 Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Setup environment
echo ""
echo "⚙️  Setting up environment..."
ROS_SETUP_FILE="$HOME/.ros2_jetson_setup.bash"
cat > "$ROS_SETUP_FILE" << 'EOF'
#!/bin/bash
# ROS2 Humble environment setup for Jetson Orin Nano

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "$HOME/omega_ws/install/setup.bash" ]; then
    source "$HOME/omega_ws/install/setup.bash"
fi

# Set ROS2 domain ID (default: 0)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Use CycloneDDS for multi-device communication
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set CycloneDDS config
if [ -f "$HOME/Omega-Code/docker/ros2_robot/config/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file://$HOME/Omega-Code/docker/ros2_robot/config/cyclonedds.xml
elif [ -f "$HOME/omega_ws/config/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file://$HOME/omega_ws/config/cyclonedds.xml
fi

# CUDA Configuration
export CUDA_VISIBLE_DEVICES=0

# Jetson-specific optimizations
export JETSON_MODE=MAXN  # Maximum performance mode
export JETSON_CLOCKS=on   # Enable clock scaling

# Performance tuning for ROS2
export RCUTILS_LOGGING_SEVERITY=WARN  # Reduce logging overhead
export RCUTILS_COLORIZED_OUTPUT=0     # Disable color output for performance
EOF

# Add to .bashrc if not already present
if ! grep -q "ros2_jetson_setup.bash" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc"
    echo "# ROS2 Jetson Setup" >> "$HOME/.bashrc"
    echo "source $ROS_SETUP_FILE" >> "$HOME/.bashrc"
    echo "✅ Added ROS2 setup to .bashrc"
fi

# Create workspace directory
echo ""
echo "📁 Setting up workspace..."
mkdir -p "$HOME/omega_ws/src"
mkdir -p "$HOME/omega_ws/config"

# Link Omega-Code packages if available
if [ -d "$PROJECT_ROOT/ros/src/omega_robot" ]; then
    if [ ! -L "$HOME/omega_ws/src/omega_robot" ]; then
        ln -s "$PROJECT_ROOT/ros/src/omega_robot" "$HOME/omega_ws/src/omega_robot"
        echo "✅ Linked Omega-Code packages to workspace"
    else
        echo "✅ Workspace link already exists"
    fi
else
    echo "⚠️  Omega-Code packages not found at $PROJECT_ROOT/ros/src/omega_robot"
    echo "   You may need to clone or copy the packages manually"
fi

# Copy CycloneDDS config if available
if [ -f "$PROJECT_ROOT/docker/ros2_robot/config/cyclonedds.xml" ]; then
    cp "$PROJECT_ROOT/docker/ros2_robot/config/cyclonedds.xml" \
       "$HOME/omega_ws/config/cyclonedds.xml"
    echo "✅ Copied CycloneDDS configuration"
fi

# Install Python dependencies for vision/AI
echo ""
echo "📦 Installing Python dependencies for vision/AI..."
pip3 install --upgrade pip setuptools wheel

# Install basic dependencies first
echo "Installing basic dependencies..."
pip3 install \
    numpy \
    opencv-python \
    pillow \
    scipy

# Install PyTorch using Jetson-specific script
echo ""
echo "🔥 Installing PyTorch for Jetson (this may take time)..."
if [ -f "$SCRIPT_DIR/install_pytorch_jetson.sh" ]; then
    bash "$SCRIPT_DIR/install_pytorch_jetson.sh"
else
    echo "⚠️  PyTorch installer script not found"
    echo "   Please run: ./scripts/install_pytorch_jetson.sh manually"
    read -p "Continue without PyTorch? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Install ROS2 Python packages
echo ""
echo "📦 Installing ROS2 Python packages..."
pip3 install \
    rclpy

# Note: sensor-msgs, cv-bridge, image-transport are ROS2 packages, install via apt
echo ""
echo "📦 Installing ROS2 message packages..."
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs || echo "⚠️  Some ROS2 packages may need manual installation"

# Configure Jetson performance mode
echo ""
echo "⚡ Configuring Jetson performance mode..."
if command -v jetson_clocks &> /dev/null; then
    echo "   Jetson clocks utility found"
    echo "   Run 'sudo jetson_clocks' to enable maximum performance"
fi

if [ -f /sys/devices/soc0/tegra_id ]; then
    echo "   Detected NVIDIA Tegra SoC"
    echo "   Consider setting power mode: sudo nvpmodel -m 0  # MAXN mode"
fi

# Summary
echo ""
echo "========================================="
echo "✅ Jetson Orin Nano setup complete!"
echo ""
echo "📋 Next steps:"
echo ""
echo "1. Source the environment:"
echo "   source $ROS_SETUP_FILE"
echo "   # Or restart your terminal"
echo ""
echo "2. Build the workspace:"
echo "   cd ~/omega_ws"
echo "   colcon build --symlink-install"
echo ""
echo "3. Test ROS2 communication:"
echo "   # Terminal 1 (on Jetson):"
echo "   ros2 run demo_nodes_cpp talker"
echo "   # Terminal 2 (on laptop/Pi):"
echo "   ros2 run demo_nodes_cpp listener"
echo ""
echo "4. Run vision/AI nodes:"
echo "   ros2 run omega_robot vision_processor"
echo ""
echo "5. Enable maximum performance (optional):"
echo "   sudo jetson_clocks"
echo "   sudo nvpmodel -m 0  # MAXN mode"
echo ""
echo "========================================="

