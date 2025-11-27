#!/bin/bash

set -e

echo "🚀 Omega ROS2 Development Container Starting..."
echo "================================================"

# Source ROS 2 setup
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble sourced"
else
    echo "⚠️  Warning: ROS2 Humble not found"
fi

# Set CycloneDDS environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

# Set CycloneDDS config if available
if [ -f /workspace/Omega-Code/docker/ros2_robot/config/cyclonedds.xml ]; then
    export CYCLONEDDS_URI=file:///workspace/Omega-Code/docker/ros2_robot/config/cyclonedds.xml
    echo "✅ CycloneDDS config loaded"
fi

# Link ROS workspace if Omega-Code is mounted
if [ -d /workspace/Omega-Code/ros/src/omega_robot ]; then
    echo "📁 Setting up ROS2 workspace..."
    
    # Create symlink to ROS packages
    if [ ! -L /root/omega_ws/src/omega_robot ]; then
        ln -sf /workspace/Omega-Code/ros/src/omega_robot /root/omega_ws/src/omega_robot
        echo "✅ Linked omega_robot package to workspace"
    fi
    
    # Build workspace if needed
    if [ -f /root/omega_ws/install/setup.bash ]; then
        source /root/omega_ws/install/setup.bash
        echo "✅ ROS2 workspace sourced"
    else
        echo "📦 Building ROS2 workspace (first time)..."
        cd /root/omega_ws
        rosdep update || true
        rosdep install --from-paths src --ignore-src -r -y || true
        colcon build --symlink-install
        source /root/omega_ws/install/setup.bash
        echo "✅ ROS2 workspace built and sourced"
    fi
else
    echo "⚠️  Warning: Omega-Code ROS packages not found at /workspace/Omega-Code/ros/src/omega_robot"
fi

# Create helper aliases
cat >> ~/.bashrc << 'EOF'
# Omega ROS2 Development Aliases
alias rebuild-ws='cd /root/omega_ws && colcon build --symlink-install && source install/setup.bash'
alias sync-code='cd /workspace/Omega-Code && git pull origin master'
alias ros-source='source /opt/ros/humble/setup.bash && source /root/omega_ws/install/setup.bash'
alias ros-nodes='ros2 node list'
alias ros-topics='ros2 topic list'
alias ros-echo='ros2 topic echo'
EOF

echo ""
echo "✅ Container ready!"
echo ""
echo "📋 Quick Commands:"
echo "  rebuild-ws    - Rebuild ROS2 workspace"
echo "  sync-code     - Pull latest code from GitHub"
echo "  ros-source    - Source ROS2 environment"
echo "  ros-nodes     - List ROS2 nodes"
echo "  ros-topics    - List ROS2 topics"
echo ""
echo "🔧 To rebuild workspace after code changes:"
echo "  cd /root/omega_ws && colcon build --symlink-install && source install/setup.bash"
echo ""

# Execute the command passed to the container
exec "$@"

