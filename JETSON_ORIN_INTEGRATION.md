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

## 📦 PyTorch Installation

### Quick Installation

**On Jetson**:
```bash
cd ~/Omega-Code
chmod +x scripts/install_pytorch_jetson.sh scripts/verify_pytorch_jetson.sh
./scripts/install_pytorch_jetson.sh
```

**What the installer does**:
- Detects your JetPack and CUDA versions
- Downloads appropriate PyTorch wheel from NVIDIA
- Installs torchvision
- Verifies CUDA support
- Configures library paths

**Installation time**: 5-15 minutes (depending on download speed)

### Manual Installation

If you already have a PyTorch wheel file:

```bash
# Install the wheel file
pip3 install torch-2.4.0a0+3bcc3cddb5.nv24.07.16234504-cp310-cp310-linux_aarch64.whl

# Install torchvision
pip3 install torchvision --no-deps
pip3 install pillow

# Verify installation
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"
```

### Copying Scripts to Jetson

**From your laptop**:
```bash
# Copy PyTorch installation scripts
scp scripts/install_pytorch_jetson.sh omega1@100.107.112.110:~/Omega-Code/scripts/
scp scripts/verify_pytorch_jetson.sh omega1@100.107.112.110:~/Omega-Code/scripts/

# Or copy entire project via rsync
rsync -avz --exclude 'venv' --exclude 'node_modules' --exclude '.git' \
  ~/Desktop/Omega-Code/ omega1@100.107.112.110:~/Omega-Code/
```

### PyTorch Installation Status

**Current System Info**:
- **JetPack**: R36 (Revision 4.7)
- **CUDA**: 12.6.68
- **cuDNN**: 9.3.0
- **Python**: 3.10

**Known Issues**:
- PyTorch wheels from PyPI are for x86_64, not ARM64 - use Jetson-specific wheels
- cuDNN version mismatches (system has cuDNN 9, some wheels require cuDNN 8)
- Missing libraries: `libcusparseLt.so.0`, `libcudnn.so.8`

**Solution**: Use NVIDIA pre-built wheels matching your JetPack version.

### Verifying PyTorch Installation

```bash
# Check PyTorch version
python3 -c "import torch; print(f'PyTorch: {torch.__version__}')"

# Check CUDA availability
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"

# Test GPU tensor
python3 << EOF
import torch
if torch.cuda.is_available():
    x = torch.randn(3, 3).cuda()
    print(f"✅ GPU tensor test passed: {x.shape}")
    print(f"✅ Device: {torch.cuda.get_device_name(0)}")
else:
    print("❌ CUDA not available")
EOF
```

## 🔧 cuDNN Library Path Configuration

### Issue: cuDNN Library Not Found

**Error**: `ImportError: libcudnn.so.8: cannot open shared object file`

### Solution

**Step 1: Find cuDNN library location**
```bash
# Search for cuDNN libraries
find /usr -name "libcudnn.so*" 2>/dev/null
```

**Step 2: Add to library path**

Add to `~/.bashrc`:
```bash
# Add cuDNN to library path
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH

# CUDA environment
export CUDA_HOME=/usr/local/cuda-12.6
export PATH=$CUDA_HOME/bin:$PATH
```

**Step 3: Reload and test**
```bash
source ~/.bashrc
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"
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
5. Check library paths: `echo $LD_LIBRARY_PATH`

### Issue: PyTorch/torchvision installation fails

**Common Causes**:
- Wrong architecture (need ARM64, not x86_64)
- Missing CUDA dependencies
- Insufficient disk space
- Wrong PyTorch version for JetPack
- cuDNN version mismatch

**Solutions**:

1. **Use Jetson-specific installer**:
```bash
cd ~/Omega-Code
./scripts/install_pytorch_jetson.sh
```

2. **Clear pip cache and reinstall**:
```bash
pip3 cache purge
pip3 install --no-cache-dir torch torchvision
```

3. **Increase swap space** (if out of memory):
```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

4. **Install with limited parallelism**:
```bash
export MAX_JOBS=1
pip3 install torch torchvision --no-cache-dir
```

5. **Check system requirements**:
```bash
# Run diagnostic
cat /etc/nv_tegra_release  # JetPack version
nvcc --version              # CUDA version
python3 --version          # Python version
df -h /                     # Disk space
free -h                     # Memory
```

### Issue: Build fails on Jetson

**Solution**:
1. Check available memory: `free -h`
2. Reduce parallel jobs: `colcon build --parallel-workers 2`
3. Build packages individually
4. Check disk space: `df -h`
5. Increase swap space if needed (see above)

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

## 📋 Installation Checklist

### Initial Setup
- [ ] SSH access configured
- [ ] Omega-Code repository cloned/copied
- [ ] ROS2 Humble Desktop installed
- [ ] CUDA environment configured
- [ ] CycloneDDS configured for multi-device

### PyTorch Setup
- [ ] PyTorch installation script run
- [ ] PyTorch CUDA support verified
- [ ] torchvision installed
- [ ] Library paths configured
- [ ] GPU tensor test passed

### ROS2 Integration
- [ ] Workspace created (`~/omega_ws`)
- [ ] Packages built (`colcon build`)
- [ ] Multi-device communication tested
- [ ] Topics visible across devices

### Performance Optimization
- [ ] Maximum performance mode enabled
- [ ] System monitoring tools installed
- [ ] Performance benchmarks run

## 🔗 Related Documentation

- [ROS2_MULTIDEVICE_SETUP.md](ROS2_MULTIDEVICE_SETUP.md) - Full multi-device guide
- [ROS2_ARCHITECTURE.md](ROS2_ARCHITECTURE.md) - System architecture
- [ROS2_OPENCV_INTEGRATION.md](ROS2_OPENCV_INTEGRATION.md) - OpenCV integration

## 📚 Additional Resources

- **NVIDIA PyTorch Forum**: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
- **JetPack Documentation**: https://developer.nvidia.com/embedded/jetpack
- **PyTorch Official Docs**: https://pytorch.org/get-started/locally/

---

**Last Updated**: 2024  
**Jetson IP**: 100.107.112.110 (Tailscale)  
**SSH User**: omega1  
**JetPack**: R36 (Revision 4.7)  
**CUDA**: 12.6.68

