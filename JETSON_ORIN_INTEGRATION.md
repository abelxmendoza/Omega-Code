# Jetson Orin Nano Integration Guide

Complete guide for integrating your Jetson Orin Nano into the Omega-Code ROS2 ecosystem.

## 🎯 Quick Start

Your Jetson Orin Nano is accessible via:
- **SSH**: `omega1@100.107.112.110` (Tailscale IP)
- **Role**: AI Compute Engine (Vision, SLAM, ML Models, CUDA)

## 📋 Prerequisites

- [ ] SSH access to Jetson: `ssh omega1@100.107.112.110`
- [ ] Jetson running Ubuntu 22.04 (JetPack 5.x)
- [ ] CUDA installed (should come with JetPack)
- [ ] Network connectivity (Tailscale or local network)

## 🚀 Step 1: Test SSH Connection

```bash
# Test SSH connectivity
ssh omega1@100.107.112.110

# Once connected, check system info
uname -a
nvcc --version  # Check CUDA
hostname
```

## 🔧 Step 2: Configure Environment

### On Your Local Machine (MacBook/Laptop)

1. **Create environment file**:
```bash
cd ~/Omega-Code
cp .env.ros2.multidevice.example .env.ros2.multidevice
```

2. **Edit `.env.ros2.multidevice`** with your Jetson IP:
```bash
ORIN_IP=100.107.112.110
ORIN_SSH_USER=omega1
```

### On Jetson Orin Nano

1. **Clone or copy Omega-Code** (if not already present):
```bash
# Via SSH
ssh omega1@100.107.112.110
cd ~
git clone <your-repo-url> Omega-Code
# OR copy via rsync from your laptop
```

## 🛠️ Step 3: Run Jetson Setup Script

**On the Jetson** (via SSH):

```bash
ssh omega1@100.107.112.110
cd ~/Omega-Code
chmod +x scripts/setup_jetson_orin.sh
./scripts/setup_jetson_orin.sh
```

This script will:
- ✅ Install ROS2 Humble Desktop
- ✅ Configure CUDA environment
- ✅ Set up CycloneDDS for multi-device communication
- ✅ Create workspace structure
- ✅ Install Python dependencies for vision/AI
- ✅ Configure Jetson performance settings

## 📡 Step 4: Configure CycloneDDS

The CycloneDDS config has been updated with your Jetson IP. Verify it:

**On your laptop**:
```bash
cat docker/ros2_robot/config/cyclonedds.xml
```

Should show:
```xml
<Peer address="100.107.112.110"/> <!-- Jetson Orin Nano -->
```

**Copy config to Jetson**:
```bash
# From your laptop
scp docker/ros2_robot/config/cyclonedds.xml \
    omega1@100.107.112.110:~/omega_ws/config/
```

## 🧪 Step 5: Test ROS2 Communication

### Terminal 1: Laptop
```bash
# Source ROS2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/docker/ros2_robot/config/cyclonedds.xml

# Run talker
ros2 run demo_nodes_cpp talker
```

### Terminal 2: Jetson (via SSH)
```bash
ssh omega1@100.107.112.110
source ~/.ros2_jetson_setup.bash

# Run listener
ros2 run demo_nodes_cpp listener
```

You should see messages being received! 🎉

## 🎨 Step 6: Build Workspace on Jetson

```bash
ssh omega1@100.107.112.110
cd ~/omega_ws

# Build ROS2 packages
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## 🤖 Step 7: Run Vision/AI Nodes

The Jetson is configured for AI workloads. Example nodes:

### Vision Processor Node
```bash
# On Jetson
ros2 run omega_robot vision_processor
```

### Object Detection Node
```bash
# On Jetson
ros2 run omega_robot object_detector
```

### SLAM Node
```bash
# On Jetson
ros2 run omega_robot slam_processor
```

## 📊 Step 8: Monitor System

### Check ROS2 Topics (from any device)
```bash
# List all topics
ros2 topic list

# Echo telemetry
ros2 topic echo /omega/telemetry

# Check node info
ros2 node list
ros2 node info /vision_processor
```

### Monitor Jetson Performance
```bash
# On Jetson
sudo tegrastats  # System stats
nvidia-smi       # GPU stats
htop             # CPU/Memory
```

## ⚡ Performance Optimization

### Enable Maximum Performance Mode
```bash
# On Jetson
sudo jetson_clocks          # Enable max clocks
sudo nvpmodel -m 0          # MAXN mode (max performance)
```

### Verify CUDA
```bash
nvcc --version
python3 -c "import torch; print(torch.cuda.is_available())"
```

## 🔄 Multi-Device Workflow

### Development Workflow

1. **Code on Laptop**: Edit ROS packages in `ros/src/omega_robot/`
2. **Build Locally**: `cd ~/omega_ws && colcon build`
3. **Sync to Jetson**:
```bash
rsync -avz ~/omega_ws/ omega1@100.107.112.110:~/omega_ws/
```
4. **Build on Jetson**:
```bash
ssh omega1@100.107.112.110
cd ~/omega_ws && colcon build
```

### Launch Multi-Device System

**Option 1: Launch File** (from laptop):
```bash
ros2 launch omega_robot multidevice_setup.launch.py
```

**Option 2: Manual Launch**:
```bash
# Terminal 1: Laptop (monitoring)
ros2 run omega_robot telemetry_listener
rviz2

# Terminal 2: Jetson (vision processing)
ssh omega1@100.107.112.110
ros2 run omega_robot vision_processor

# Terminal 3: Pi (hardware control)
ssh pi@<pi_ip>
ros2 run omega_robot sensor_data_publisher
```

## 🐛 Troubleshooting

### Issue: Topics not visible across devices

**Solution**:
1. Verify `ROS_DOMAIN_ID` matches on all devices (default: 0)
2. Check CycloneDDS config has all device IPs
3. Verify firewall allows UDP 7400-7500:
```bash
sudo ufw allow 7400:7500/udp
```
4. Restart ROS2 daemon:
```bash
ros2 daemon stop
ros2 daemon start
```

### Issue: SSH connection fails

**Solution**:
1. Verify Tailscale is connected: `tailscale status`
2. Test connectivity: `ping 100.107.112.110`
3. Check SSH keys: `ssh-copy-id omega1@100.107.112.110`
4. Verify SSH service: `ssh omega1@100.107.112.110 "systemctl status ssh"`

### Issue: CUDA not available

**Solution**:
1. Verify JetPack installation: `cat /etc/nv_tegra_release`
2. Check CUDA: `nvcc --version`
3. Test PyTorch CUDA: `python3 -c "import torch; print(torch.cuda.is_available())"`
4. Install CUDA toolkit if missing: `sudo apt install nvidia-cuda-toolkit`

### Issue: PyTorch/torchvision installation fails

**Solution**:
1. Use Jetson-specific installer: `./scripts/install_pytorch_jetson.sh`
2. See detailed troubleshooting: [JETSON_PYTORCH_TROUBLESHOOTING.md](JETSON_PYTORCH_TROUBLESHOOTING.md)
3. Common causes:
   - Wrong architecture (need ARM64, not x86_64)
   - Missing CUDA dependencies
   - Insufficient disk space
   - Wrong PyTorch version for JetPack

### Issue: Build fails on Jetson

**Solution**:
1. Check available memory: `free -h`
2. Reduce parallel jobs: `colcon build --parallel-workers 2`
3. Build packages individually
4. Check disk space: `df -h`

## 📚 Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│              Omega-Code Multi-Device ROS2                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐ │
│  │   Laptop     │    │   Pi 4B      │    │ Orin Nano    │ │
│  │ (Ubuntu)     │    │              │    │              │ │
│  │              │    │              │    │              │ │
│  │ Dev Cockpit  │◄───┤ Hardware IO  │◄───┤ AI Compute   │ │
│  │              │    │              │    │              │ │
│  │ • colcon     │    │ • Motors     │    │ • Vision     │ │
│  │ • RViz       │    │ • GPIO       │    │ • SLAM       │ │
│  │ • SSH mgmt   │    │ • Sensors    │    │ • ML Models  │ │
│  │ • Docker     │    │ • Micro-ROS  │    │ • CUDA       │ │
│  └──────────────┘    └──────────────┘    └──────────────┘ │
│                                                             │
│              ROS2 DDS (CycloneDDS) Network                 │
│              Domain ID: 0                                  │
│              Tailscale: 100.107.112.110                     │
└─────────────────────────────────────────────────────────────┘
```

## 🎯 Jetson-Specific Features

### CUDA Acceleration
- GPU-accelerated vision processing
- ML model inference
- Real-time SLAM processing

### Performance Modes
- **MAXN Mode**: Maximum performance (power hungry)
- **Balanced Mode**: Power-efficient with good performance
- **Power Save Mode**: Minimal power consumption

### Recommended Workloads
- ✅ Computer vision (OpenCV + CUDA)
- ✅ Object detection (YOLO, TensorRT)
- ✅ SLAM (ORB-SLAM, RTAB-Map)
- ✅ Deep learning inference
- ✅ Image processing pipelines

## 📝 Next Steps

1. **Create Vision Nodes**: Implement camera processing nodes
2. **Add ML Models**: Deploy YOLO or custom models
3. **SLAM Integration**: Set up mapping and localization
4. **Performance Tuning**: Optimize for your specific workloads
5. **Monitoring**: Set up system monitoring dashboards

## 🔗 Related Documentation

- [ROS2_MULTIDEVICE_SETUP.md](ROS2_MULTIDEVICE_SETUP.md) - Full multi-device guide
- [ROS2_ARCHITECTURE.md](ROS2_ARCHITECTURE.md) - System architecture
- [ROS2_OPENCV_INTEGRATION.md](ROS2_OPENCV_INTEGRATION.md) - OpenCV integration

---

**Last Updated**: 2024  
**Jetson IP**: 100.107.112.110 (Tailscale)  
**SSH User**: omega1

