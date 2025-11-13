#!/bin/bash
# setup_ros2_laptop.sh
# ROS2 Humble setup script for Ubuntu laptop (development cockpit)
# Supports: i5-8265U + 8GB RAM (colcon, RViz, SSH, Docker management)

set -e

echo "🚀 Setting up ROS2 Humble on Ubuntu Laptop (Development Cockpit)"
echo "================================================================"

# Detect Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
echo "Detected Ubuntu version: $UBUNTU_VERSION"

# Check if ROS2 is already installed
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "✅ ROS2 Humble is already installed"
    read -p "Reinstall? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Skipping ROS2 installation"
        exit 0
    fi
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
    python3-rosinstall-generator

# Add ROS2 repository
echo ""
echo "📥 Adding ROS2 Humble repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble
echo ""
echo "📦 Installing ROS2 Humble..."
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rviz2 \
    ros-humble-turtlebot3 \
    ros-humble-gazebo-ros-pkgs

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
ROS_SETUP_FILE="$HOME/.ros2_setup.bash"
cat > "$ROS_SETUP_FILE" << 'EOF'
#!/bin/bash
# ROS2 Humble environment setup for laptop

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

# Set CycloneDDS config (update path as needed)
if [ -f "$HOME/Omega-Code/docker/ros2_robot/config/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file://$HOME/Omega-Code/docker/ros2_robot/config/cyclonedds.xml
fi

# Optimize for laptop (light RViz settings)
export OGRE_RTT_MODE=Copy
export LIBGL_ALWAYS_SOFTWARE=0

echo "✅ ROS2 Humble environment loaded"
echo "   Domain ID: $ROS_DOMAIN_ID"
echo "   RMW: $RMW_IMPLEMENTATION"
EOF

chmod +x "$ROS_SETUP_FILE"

# Add to bashrc if not already present
if ! grep -q "ros2_setup.bash" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc"
    echo "# ROS2 Humble setup" >> "$HOME/.bashrc"
    echo "source $ROS_SETUP_FILE" >> "$HOME/.bashrc"
    echo "Added ROS2 setup to ~/.bashrc"
fi

# Create workspace directory
echo ""
echo "📁 Creating ROS2 workspace..."
mkdir -p "$HOME/omega_ws/src"
cd "$HOME/omega_ws"

# Clone Omega-Code ROS packages if not already present
if [ ! -d "$HOME/omega_ws/src/omega_robot" ]; then
    echo "📥 Linking Omega-Code ROS packages..."
    if [ -d "$HOME/Omega-Code/ros/src/omega_robot" ]; then
        ln -s "$HOME/Omega-Code/ros/src/omega_robot" "$HOME/omega_ws/src/omega_robot"
    else
        echo "⚠️  Warning: Omega-Code ROS packages not found at ~/Omega-Code/ros/src/omega_robot"
    fi
fi

# Build workspace
echo ""
echo "🔨 Building ROS2 workspace..."
source /opt/ros/humble/setup.bash
if [ -d "$HOME/omega_ws/src/omega_robot" ]; then
    cd "$HOME/omega_ws"
    colcon build --symlink-install
    echo "✅ Workspace built successfully"
else
    echo "⚠️  Skipping build - no packages found"
fi

# Install Docker (if not installed)
echo ""
if ! command -v docker &> /dev/null; then
    echo "🐳 Installing Docker..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    rm get-docker.sh
    echo "✅ Docker installed (logout/login required for group changes)"
else
    echo "✅ Docker is already installed"
fi

# Install Docker Compose (if not installed)
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "🐳 Installing Docker Compose..."
    sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
    echo "✅ Docker Compose installed"
else
    echo "✅ Docker Compose is already installed"
fi

# Summary
echo ""
echo "================================================================"
echo "✅ ROS2 Humble setup complete!"
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
echo "================================================================"

