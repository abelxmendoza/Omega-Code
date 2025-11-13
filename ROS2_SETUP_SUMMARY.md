# ROS2 Multi-Device Setup - Quick Reference

## 📋 What Was Created

### Setup Scripts
1. **`scripts/setup_ros2_laptop.sh`**
   - Installs ROS2 Humble on Ubuntu
   - Sets up workspace, colcon, RViz
   - Configures environment variables
   - Installs Docker if needed

2. **`scripts/optimize_laptop_ros2.sh`**
   - Optimizes Ubuntu laptop for ROS2 development
   - Configures swap, CPU governor, network
   - Creates optimized build script
   - Sets up RViz configuration

3. **`scripts/setup_multidevice_ros2.sh`**
   - Complete multi-device setup orchestrator
   - Auto-detects device type (laptop/pi/orin)
   - Generates CycloneDDS configuration
   - Links workspace and packages

4. **`scripts/quick_start_ros2.sh`**
   - Quick environment setup
   - Device-specific command reference
   - Fast testing workflow

### Configuration Files
1. **`.env.ros2.multidevice.example`**
   - Template for multi-device environment
   - Device IPs, SSH config, workspace paths
   - Performance tuning options

2. **`docker/ros2_robot/config/cyclonedds.xml.template`**
   - CycloneDDS peer discovery template
   - Uses environment variable substitution

3. **`docker/ros2_robot/docker-compose.multidevice.yml`**
   - Multi-device Docker Compose configuration
   - Profiles for laptop, Pi, Orin
   - Network host mode for DDS

### Launch Files
1. **`ros/launch/multidevice_setup.launch.py`**
   - Python launch file for multi-device orchestration
   - Configurable device IPs
   - Node distribution across devices

### Documentation
1. **`ROS2_MULTIDEVICE_SETUP.md`**
   - Comprehensive setup guide
   - Architecture overview
   - Troubleshooting section
   - Best practices

2. **Updated `README.md`**
   - Added multi-device section
   - Quick start instructions
   - Links to detailed docs

## 🚀 Quick Start (3 Steps)

### Step 1: Configure
```bash
cd ~/Omega-Code
cp .env.ros2.multidevice.example .env.ros2.multidevice
# Edit with your device IPs
```

### Step 2: Setup
```bash
# On each device (laptop, Pi, Orin):
./scripts/setup_multidevice_ros2.sh
```

### Step 3: Test
```bash
# Terminal 1 (Laptop):
ros2 run demo_nodes_cpp talker

# Terminal 2 (Pi or Orin):
ros2 run demo_nodes_cpp listener
```

## 📁 File Structure

```
Omega-Code/
├── scripts/
│   ├── setup_ros2_laptop.sh          # Laptop ROS2 installation
│   ├── optimize_laptop_ros2.sh       # Laptop optimization
│   ├── setup_multidevice_ros2.sh     # Multi-device orchestrator
│   └── quick_start_ros2.sh           # Quick start helper
├── docker/ros2_robot/
│   ├── config/
│   │   ├── cyclonedds.xml            # Current config (update manually)
│   │   └── cyclonedds.xml.template   # Template with variables
│   └── docker-compose.multidevice.yml # Multi-device Docker setup
├── ros/
│   └── launch/
│       └── multidevice_setup.launch.py  # Multi-device launch file
├── .env.ros2.multidevice.example     # Environment template
├── ROS2_MULTIDEVICE_SETUP.md         # Full documentation
└── ROS2_SETUP_SUMMARY.md              # This file
```

## 🎯 Device Roles

| Device | Role | Capabilities |
|--------|------|-------------|
| **Laptop** | Dev Cockpit | colcon, RViz, SSH, Docker, Git |
| **Pi 4B** | Hardware IO | Motors, GPIO, Sensors, Micro-ROS |
| **Orin Nano** | AI Compute | Vision, SLAM, ML, CUDA |

## 🔧 Common Commands

### Environment Setup
```bash
source .env.ros2.multidevice
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/Omega-Code/docker/ros2_robot/config/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/omega_ws/install/setup.bash
```

### Build Workspace
```bash
cd ~/omega_ws
colcon build --symlink-install

# Optimized build (laptop):
~/.ros2_optimize_build.sh
```

### Launch Multi-Device
```bash
ros2 launch omega_robot multidevice_setup.launch.py
```

### Monitor System
```bash
~/.ros2_monitor.sh
```

## 🔍 Troubleshooting Quick Fixes

### Topics Not Visible
```bash
# 1. Check domain ID matches
echo $ROS_DOMAIN_ID

# 2. Restart daemon
ros2 daemon stop
ros2 daemon start

# 3. Verify config
cat $CYCLONEDDS_URI
```

### Slow RViz
```bash
# Run optimization
./scripts/optimize_laptop_ros2.sh

# Reduce FPS
export RVIZ_FPS=15
```

### Build Fails
```bash
# Reduce parallel jobs
colcon build --parallel-workers 2

# Check memory
free -h
```

## 📚 Next Steps

1. **Read Full Guide**: `ROS2_MULTIDEVICE_SETUP.md`
2. **Configure Devices**: Update `.env.ros2.multidevice`
3. **Run Setup**: `./scripts/setup_multidevice_ros2.sh`
4. **Test Communication**: Use demo_nodes_cpp
5. **Develop Nodes**: Create packages in `ros/src/omega_robot/`

## 🆘 Need Help?

- Check `ROS2_MULTIDEVICE_SETUP.md` troubleshooting section
- Review ROS2 logs: `~/.ros2/log/`
- Verify network connectivity: `ping device_ip`
- Check environment: `env | grep ROS`

---

**Created**: 2024  
**ROS2 Version**: Humble Hawksbill  
**Tested On**: Ubuntu 22.04

