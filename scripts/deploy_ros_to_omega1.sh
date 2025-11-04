#!/bin/bash
# File: /Omega-Code/scripts/deploy_ros_to_omega1.sh
# Purpose: Deploy ROS packages to omega1 and set up workspace

set -euo pipefail

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_DIR="$PROJECT_ROOT/ros"

# SSH target
SSH_TARGET="omega1-tailscale"

# Workspace directory on omega1 (ROS 2 uses different structure)
WORKSPACE_DIR="$HOME/ros2_ws"
OMEGA_ROS_PKG_DIR="$WORKSPACE_DIR/src/omega_robot"

echo "📦 Deploying ROS 2 packages to omega1..."

# Create package directory structure on omega1
echo "📁 Creating package structure..."
ssh "$SSH_TARGET" << 'EOF'
mkdir -p ~/ros2_ws/src/omega_robot/omega_robot
mkdir -p ~/ros2_ws/src/omega_robot/launch
chmod -R 755 ~/ros2_ws
EOF

# Copy scripts to ROS 2 package directory
echo "📤 Copying ROS 2 scripts..."
scp -r "$ROS_DIR/scripts/"* "$SSH_TARGET:$OMEGA_ROS_PKG_DIR/omega_robot/"

# Copy launch files (ROS 2 uses .py launch files, but XML format is also supported)
echo "📤 Copying launch files..."
ssh "$SSH_TARGET" "mkdir -p $OMEGA_ROS_PKG_DIR/launch"
scp -r "$ROS_DIR/launch/"*.launch "$SSH_TARGET:$OMEGA_ROS_PKG_DIR/launch/"

# Create ROS 2 package.xml and setup.py for Python package
echo "📤 Creating ROS 2 package files..."
ssh "$SSH_TARGET" << 'EOFPKG'
cd ~/ros2_ws/src/omega_robot

# Create ROS 2 package.xml (format="3" for ROS 2)
cat > package.xml << 'PKGXML'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>omega_robot</name>
  <version>0.0.1</version>
  <description>The omega_robot package</description>
  <maintainer email="abelxmendoza@gmail.com">Abel Mendoza</maintainer>
  <license>BSD</license>
  
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
PKGXML

# Create setup.py for Python package
cat > setup.py << 'SETUPPY'
from setuptools import find_packages, setup

package_name = 'omega_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/' + f for f in 
            ['full_setup.launch', 'macbook_only.launch', 'macbook_rpi.launch'] 
            if __import__('os').path.exists('launch/' + f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abel Mendoza',
    maintainer_email='abelxmendoza@gmail.com',
    description='The omega_robot package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add entry points if you want to run nodes directly
            # 'camera_publisher = omega_robot.camera_publisher:main',
        ],
    },
)
SETUPPY

# Create resource marker file
mkdir -p resource
touch resource/omega_robot

# Create __init__.py for Python package
touch omega_robot/__init__.py

# Make scripts executable
chmod +x omega_robot/*.py
EOFPKG

# Install Python dependencies and build workspace
echo "🔧 Installing dependencies and building workspace..."
ssh "$SSH_TARGET" << 'EOFBUILD'
set -e

source /opt/ros/iron/setup.bash 2>/dev/null || {
    echo "⚠️  ROS 2 not yet installed. Please run setup_ros_omega1.sh first!"
    exit 1
}

# Install Python dependencies
echo "📥 Installing Python dependencies..."
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    ros-iron-cv-bridge \
    ros-iron-sensor-msgs \
    ros-iron-std-msgs \
    ros-iron-geometry-msgs \
    ros-iron-image-transport \
    ros-iron-launch-ros

# For Pi camera (if using picamera - consider picamera2 for newer Pi)
# sudo apt-get install -y python3-picamera2 || echo "picamera2 installation skipped"

# Install pip packages
pip3 install --user \
    rclpy \
    setuptools \
    opencv-python \
    numpy || true

# Install ROS dependencies
echo "📦 Installing ROS 2 package dependencies..."
cd ~/ros2_ws
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || echo "Some dependencies may need manual installation"

# Build workspace with colcon (ROS 2 build tool)
echo "🔨 Building ROS 2 workspace with colcon..."
source /opt/ros/iron/setup.bash
cd ~/ros2_ws
colcon build --symlink-install

echo "✅ Build complete!"
echo ""
echo "To use ROS 2:"
echo "  source ~/.bashrc"
echo "  # or"
echo "  source /opt/ros/iron/setup.bash"
echo "  source ~/ros2_ws/install/setup.bash"
echo ""
echo "Test with:"
echo "  ros2 pkg list | grep omega_robot"
EOFBUILD

echo ""
echo "✅ ROS deployment complete!"
echo ""
echo "Next steps on omega1:"
echo "  1. SSH: ssh omega1-tailscale"
echo "  2. Source ROS 2: source ~/.bashrc"
echo "  3. Test: ros2 pkg list | grep omega_robot"

