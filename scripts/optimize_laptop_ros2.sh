#!/bin/bash
# optimize_laptop_ros2.sh
# System optimization for Ubuntu laptop running ROS2 development
# Optimizes for i5-8265U + 8GB RAM (colcon, RViz, SSH, Docker)

set -e

echo "⚙️  Optimizing Ubuntu laptop for ROS2 development"
echo "=================================================="

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
    echo "❌ Please run as regular user (not root)"
    exit 1
fi

# 1. Disable unnecessary services
echo ""
echo "🔧 Disabling unnecessary services..."
sudo systemctl disable bluetooth.service 2>/dev/null || true
sudo systemctl disable snapd.service 2>/dev/null || true
sudo systemctl disable snapd.socket 2>/dev/null || true

# 2. Optimize swap for development
echo ""
echo "💾 Optimizing swap..."
SWAP_SIZE="4G"
if ! swapon --show | grep -q "swapfile"; then
    echo "Creating swap file..."
    sudo fallocate -l $SWAP_SIZE /swapfile || sudo dd if=/dev/zero of=/swapfile bs=1024 count=4194304
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    echo "✅ Swap file created"
else
    echo "✅ Swap already configured"
fi

# 3. Optimize swappiness for development
echo ""
echo "⚙️  Optimizing swappiness..."
if ! grep -q "vm.swappiness" /etc/sysctl.conf; then
    echo "vm.swappiness=10" | sudo tee -a /etc/sysctl.conf
    sudo sysctl vm.swappiness=10
    echo "✅ Swappiness optimized (10 for development)"
else
    echo "✅ Swappiness already configured"
fi

# 4. Increase file descriptor limits
echo ""
echo "📁 Increasing file descriptor limits..."
if ! grep -q "ros2_limits" /etc/security/limits.conf; then
    echo "# ROS2 development limits" | sudo tee -a /etc/security/limits.conf
    echo "* soft nofile 65536" | sudo tee -a /etc/security/limits.conf
    echo "* hard nofile 65536" | sudo tee -a /etc/security/limits.conf
    echo "✅ File descriptor limits increased"
else
    echo "✅ File descriptor limits already configured"
fi

# 5. Optimize Docker for development
echo ""
echo "🐳 Optimizing Docker..."
if [ -f "$HOME/.docker/daemon.json" ]; then
    echo "✅ Docker daemon.json exists"
else
    mkdir -p "$HOME/.docker"
    cat > "$HOME/.docker/daemon.json" << 'EOF'
{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  },
  "storage-driver": "overlay2",
  "default-ulimits": {
    "nofile": {
      "Name": "nofile",
      "Hard": 65536,
      "Soft": 65536
    }
  }
}
EOF
    echo "✅ Docker daemon.json created"
fi

# 6. Set CPU governor to performance (if available)
echo ""
echo "⚡ Optimizing CPU governor..."
if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
    if ! grep -q "performance" /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null; then
        echo "Setting CPU governor to performance..."
        for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
            echo performance | sudo tee $cpu > /dev/null 2>&1 || true
        done
        echo "✅ CPU governor set to performance"
    else
        echo "✅ CPU governor already set to performance"
    fi
else
    echo "⚠️  CPU governor not available (may require acpi-cpufreq driver)"
fi

# 7. Optimize network for ROS2 DDS
echo ""
echo "🌐 Optimizing network for ROS2 DDS..."
if ! grep -q "ros2_network" /etc/sysctl.conf; then
    cat << 'EOF' | sudo tee -a /etc/sysctl.conf

# ROS2 DDS network optimizations
net.core.rmem_max = 134217728
net.core.wmem_max = 134217728
net.ipv4.udp_rmem_min = 4096
net.ipv4.udp_wmem_min = 4096
net.core.netdev_max_backlog = 5000
EOF
    sudo sysctl -p
    echo "✅ Network optimized for ROS2 DDS"
else
    echo "✅ Network already optimized"
fi

# 8. Create ROS2 workspace optimization script
echo ""
echo "📝 Creating ROS2 workspace optimization..."
OPTIMIZE_SCRIPT="$HOME/.ros2_optimize_build.sh"
cat > "$OPTIMIZE_SCRIPT" << 'EOF'
#!/bin/bash
# Optimized colcon build for laptop (8GB RAM)
# Uses parallel builds with memory limits

# Limit parallel jobs to prevent OOM
JOBS=$(($(nproc) - 1))
if [ $JOBS -lt 1 ]; then
    JOBS=1
fi

# Use ccache if available
export CCACHE_DIR="$HOME/.ccache"
export CMAKE_C_COMPILER_LAUNCHER=ccache
export CMAKE_CXX_COMPILER_LAUNCHER=ccache

# Build with limited parallelism
colcon build \
    --symlink-install \
    --parallel-workers $JOBS \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    "$@"
EOF
chmod +x "$OPTIMIZE_SCRIPT"
echo "✅ Build optimization script created: $OPTIMIZE_SCRIPT"

# 9. Optimize RViz settings
echo ""
echo "🎨 Optimizing RViz settings..."
RVIZ_CONFIG="$HOME/.rviz2/default.rviz"
mkdir -p "$(dirname "$RVIZ_CONFIG")"
if [ ! -f "$RVIZ_CONFIG" ]; then
    cat > "$RVIZ_CONFIG" << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
      Splitter Ratio: 0.5
    Tree Height: 523
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Preferences:
  PromptSaveOnExit: true
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    enabled: true
    Global Options:
      Background Color: 48; 48; 48
      Fixed Frame: map
      Frame Rate: 30
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122427
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 1.5697963237762451
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002b0fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002b0000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530063007200650065006e02000000e6000000d2000003ee0000030c000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002b0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000023f000002b000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1920
  X: 72
  Y: 27
EOF
    echo "✅ RViz configuration created"
else
    echo "✅ RViz configuration already exists"
fi

# 10. Create system monitoring script
echo ""
echo "📊 Creating system monitoring script..."
MONITOR_SCRIPT="$HOME/.ros2_monitor.sh"
cat > "$MONITOR_SCRIPT" << 'EOF'
#!/bin/bash
# Quick system resource monitor for ROS2 development

echo "=== ROS2 Development System Monitor ==="
echo ""
echo "CPU:"
echo "  Cores: $(nproc)"
echo "  Load: $(uptime | awk -F'load average:' '{print $2}')"
echo ""
echo "Memory:"
free -h | grep -E "Mem|Swap"
echo ""
echo "Disk:"
df -h / | tail -1
echo ""
echo "ROS2 Processes:"
ps aux | grep -E "ros2|rviz|gazebo" | grep -v grep | wc -l | xargs echo "  Active:"
echo ""
echo "Docker:"
docker ps --format "table {{.Names}}\t{{.Status}}" 2>/dev/null || echo "  Docker not running"
echo ""
echo "Network (ROS2 DDS):"
ifconfig | grep -E "inet |RX|TX" | head -6
EOF
chmod +x "$MONITOR_SCRIPT"
echo "✅ Monitoring script created: $MONITOR_SCRIPT"

# Summary
echo ""
echo "=================================================="
echo "✅ Laptop optimization complete!"
echo ""
echo "📋 Created scripts:"
echo "   - $OPTIMIZE_SCRIPT (optimized colcon build)"
echo "   - $MONITOR_SCRIPT (system monitoring)"
echo ""
echo "📋 Next steps:"
echo "   1. Reboot to apply all changes:"
echo "      sudo reboot"
echo ""
echo "   2. Use optimized build:"
echo "      $OPTIMIZE_SCRIPT"
echo ""
echo "   3. Monitor system:"
echo "      $MONITOR_SCRIPT"
echo ""
echo "   4. Check swap:"
echo "      free -h"
echo ""
echo "=================================================="

