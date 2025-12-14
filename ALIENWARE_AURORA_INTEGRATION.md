# Alienware Aurora Integration Guide

Complete guide for adding your new **Alienware Aurora** desktop into the Omega-1 robotics development ecosystem.

## Overview

The Alienware Aurora will serve as a **high-performance development workstation** in your multi-device setup, providing:
- **Heavy computation**: GPU-accelerated ML training, simulation, and processing
- **Development hub**: Primary coding environment with maximum resources
- **Visualization**: High-end graphics for RViz, Gazebo, and visualization tools
- **Build server**: Fast compilation of ROS2 packages and Docker images

## Device Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              Omega-1 Multi-Device Ecosystem                   │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐   │
│  │ Alienware    │    │  Raspberry   │    │ Jetson Orin  │   │
│  │ Aurora       │    │  Pi 4B      │    │ Nano         │   │
│  │              │    │              │    │              │   │
│  │ Dev Workstation│  │ Hardware IO  │    │ AI Compute   │   │
│  │              │    │              │    │              │   │
│  │ • GPU ML     │    │ • Motors     │    │ • Vision     │   │
│  │ • Training   │    │ • GPIO       │    │ • SLAM       │   │
│  │ • Simulation │    │ • Sensors    │    │ • ML Models  │   │
│  │ • RViz/Gazebo│    │ • Micro-ROS  │    │ • CUDA       │   │
│  │ • Docker     │    │ • Real-time  │    │              │   │
│  └──────────────┘    └──────────────┘    └──────────────┘   │
│                                                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐   │
│  │ Laptop 1     │    │ Laptop 2     │    │ Laptop 3     │   │
│  │ (MacBook)    │    │ (Lenovo)     │    │ (scythe)     │   │
│  │              │    │              │    │              │   │
│  │ Mobile Dev   │    │ Mobile Dev   │    │ Mobile Dev   │   │
│  └──────────────┘    └──────────────┘    └──────────────┘   │
│                                                               │
│              ROS2 DDS Network (CycloneDDS)                   │
│              Tailscale VPN (Remote Access)                   │
└─────────────────────────────────────────────────────────────┘
```

## Device Roles

### Alienware Aurora - Primary Development Workstation
**Hardware**: High-end desktop (GPU, multi-core CPU, 16GB+ RAM)  
**Role**: Heavy computation, training, simulation, visualization

**Capabilities**:
- ✅ GPU-accelerated ML training (PyTorch, TensorFlow)
- ✅ Heavy ROS2 simulation (Gazebo, Ignition)
- ✅ High-end visualization (RViz with complex scenes)
- ✅ Fast package compilation (colcon build)
- ✅ Docker container orchestration
- ✅ Multi-camera processing
- ✅ SLAM algorithm development
- ✅ Real-time video processing
- ✅ CUDA/OpenCL workloads (if NVIDIA GPU)

### Existing Devices
- **Raspberry Pi 4B**: Hardware IO controller (motors, sensors, GPIO)
- **Jetson Orin Nano**: AI compute engine (inference, vision, SLAM)
- **Laptops**: Mobile development, SSH management, testing

## Prerequisites

### Hardware Requirements
- ✅ Alienware Aurora powered on and connected to network
- ✅ Ubuntu 22.04 LTS (or compatible Linux distribution)
- ✅ NVIDIA GPU drivers installed (if applicable)
- ✅ Network connectivity (same subnet or VPN)

### Software Requirements
- Python 3.9+
- Git
- Docker (optional, for containerized development)
- SSH access configured
- Tailscale installed (for remote access)

## Step 1: Network Configuration

### 1.1 Get Device Information

On the Alienware Aurora, run:

```bash
# Get hostname
hostname

# Get IP address
hostname -I | awk '{print $1}'

# Get MAC address (for static IP reservation)
ip link show | grep -A 1 "state UP" | grep -oP 'link/ether \K[^ ]+'
```

**Example Output:**
```
Hostname: aurora-desktop
IP: 192.168.1.150
MAC: aa:bb:cc:dd:ee:ff
```

### 1.2 Configure Static IP (Recommended)

Edit your network configuration to assign a static IP:

```bash
# Ubuntu 22.04+ (Netplan)
sudo nano /etc/netplan/01-netcfg.yaml
```

Add configuration:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:  # or your interface name
      dhcp4: false
      addresses:
        - 192.168.1.150/24
      gateway4: 192.168.1.1
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4
```

Apply changes:
```bash
sudo netplan apply
```

### 1.3 Update Router Configuration

1. Log into your router admin panel
2. Reserve IP `192.168.1.150` for the Alienware Aurora's MAC address
3. This ensures consistent IP assignment

### 1.4 Configure Tailscale (Optional but Recommended)

```bash
# Install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh

# Start Tailscale
sudo tailscale up

# Get Tailscale IP
tailscale ip -4
```

**Note**: Save the Tailscale IP for configuration files.

## Step 2: Development Environment Setup

### 2.1 Clone Repository

```bash
cd ~
git clone https://github.com/abelxmendoza/Omega-Code.git
cd Omega-Code
```

### 2.2 Install Dependencies

```bash
# System dependencies
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-venv \
    git \
    curl \
    build-essential \
    cmake \
    vim \
    net-tools

# Python dependencies
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
cd servers/robot_controller_backend
pip install -r requirements.txt
```

### 2.3 Install ROS2 (If Needed)

```bash
# Install ROS2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update
sudo apt install ros-humble-desktop -y

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.4 Install Docker (Optional)

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Install Docker Compose
sudo apt install docker-compose -y

# Log out and back in for group changes to take effect
```

## Step 3: Configuration Files

### 3.1 Update `config/.env.script2`

Create or update the configuration file for the Alienware Aurora:

```bash
cd ~/Omega-Code
mkdir -p config
nano config/.env.script2
```

Add configuration:
```bash
# Alienware Aurora Configuration
HOSTNAME_AURORA=aurora-desktop
AURORA_IP=192.168.1.150
AURORA_TAILSCALE_IP=100.x.x.x  # Replace with actual Tailscale IP
PI_USER_AURORA=pi
TAILSCALE_IP_AURORA=100.x.x.x  # Replace with actual Tailscale IP

# Existing configurations (keep these)
HOSTNAME_LAPTOP1=your-laptop1-hostname
HOSTNAME_LAPTOP2=your-laptop2-hostname
HOSTNAME_LAPTOP3=scythe
PI_USER_LAPTOP1=pi
PI_USER_LAPTOP2=pi
PI_USER_LAPTOP3=pi
TAILSCALE_IP_LAPTOP1=100.x.x.x
TAILSCALE_IP_LAPTOP2=100.x.x.x
TAILSCALE_IP_LAPTOP3=100.x.x.x
TAILSCALE_IP_PI=100.x.x.x
PI_USER_PI=pi
```

### 3.2 Update `connect_hotspot_v2.sh`

The script already supports dynamic hostname detection. Add support for Aurora:

```bash
# Edit the script
nano scripts/connect_hotspot_v2.sh
```

Add this section in the main script (around line 84):

```bash
elif [ "$HOSTNAME" == "$HOSTNAME_AURORA" ] || [ "$HOSTNAME" == "aurora-desktop" ]; then
    # Alienware Aurora - use AURORA config
    PI_USER="${PI_USER_AURORA:-pi}"
    TAILSCALE_IP="${TAILSCALE_IP_AURORA:-}"
    if [ -z "$TAILSCALE_IP" ]; then
        echo "Warning: TAILSCALE_IP not configured for Aurora. Please set HOSTNAME_AURORA, PI_USER_AURORA, and TAILSCALE_IP_AURORA in config/.env.script2"
    fi
    # Aurora is desktop, skip USB check
    echo "Alienware Aurora detected - skipping USB hotspot check"
elif [ "$HOSTNAME" == "scythe" ] || [ "$HOSTNAME" == "$HOSTNAME_LAPTOP3" ]; then
    # ... existing laptop3 code ...
```

### 3.3 Update ROS2 Multi-Device Configuration

Edit `.env.ros2.multidevice`:

```bash
cd ~/Omega-Code
nano .env.ros2.multidevice
```

Add Aurora configuration:
```bash
# Device IPs
AURORA_IP=192.168.1.150
LAPTOP_IP=192.168.1.100
PI_IP=192.168.1.107
ORIN_IP=192.168.1.200

# ROS2 Domain ID
ROS_DOMAIN_ID=0

# Device roles
AURORA_ROLE=workstation
LAPTOP_ROLE=dev_cockpit
PI_ROLE=hardware_io
ORIN_ROLE=ai_compute
```

### 3.4 Update Frontend Environment (If Running UI)

If you plan to run the frontend on Aurora:

```bash
cd ~/Omega-Code/ui/robot-controller-ui
nano .env.local
```

Add Aurora-specific configuration:
```bash
# Aurora Development Configuration
NEXT_PUBLIC_NETWORK_PROFILE=lan

# Video Base URL (pointing to Pi)
NEXT_PUBLIC_VIDEO_BASE_URL=http://192.168.1.107:5000
NEXT_PUBLIC_SYSTEM_BASE_URL=http://192.168.1.107:5000

# Gateway (Pi)
NEXT_PUBLIC_GATEWAY_HOST=192.168.1.107
NEXT_PUBLIC_GATEWAY_PORT=7070
NEXT_PUBLIC_GATEWAY_SCHEME=http

# Robot Host
NEXT_PUBLIC_ROBOT_HOST_LAN=192.168.1.107
```

## Step 4: ROS2 Multi-Device Setup

### 4.1 Update Launch Files

Edit `ros/launch/multidevice_setup.launch.py` to include Aurora:

```python
# Get device IPs from environment
aurora_ip = os.getenv('AURORA_IP', '192.168.1.150')
laptop_ip = os.getenv('LAPTOP_IP', '192.168.1.100')
pi_ip = os.getenv('PI_IP', '192.168.1.107')
orin_ip = os.getenv('ORIN_IP', '192.168.1.200')
ros_domain_id = os.getenv('ROS_DOMAIN_ID', '0')
```

### 4.2 Configure CycloneDDS

Edit `docker/ros2_robot/config/cyclonedds.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
    <Domain>
        <Id>0</Id>
        <General>
            <NetworkInterfaceAddress>192.168.1.150</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
```

### 4.3 Test ROS2 Connectivity

On Aurora:
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Set domain ID
export ROS_DOMAIN_ID=0

# Test communication
ros2 topic list
```

On Pi (via SSH):
```bash
ssh pi@192.168.1.107
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 topic list
```

You should see the same topics on both devices.

## Step 5: GPU Configuration (If NVIDIA GPU)

### 5.1 Install NVIDIA Drivers

```bash
# Check GPU
lspci | grep -i nvidia

# Install drivers (Ubuntu)
sudo ubuntu-drivers autoinstall

# Reboot
sudo reboot
```

### 5.2 Install CUDA (Optional)

```bash
# Download CUDA toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda

# Add to PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 5.3 Install PyTorch with CUDA

```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### 5.4 Verify GPU Access

```python
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}'); print(f'GPU: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"N/A\"}')"
```

## Step 6: SSH Configuration

### 6.1 Generate SSH Key (If Needed)

```bash
ssh-keygen -t ed25519 -C "aurora-desktop@omega-robot"
cat ~/.ssh/id_ed25519.pub
```

### 6.2 Add SSH Key to Pi and Other Devices

Copy the public key to authorized_keys on each device:

```bash
# On Pi
ssh-copy-id pi@192.168.1.107

# On Jetson (if applicable)
ssh-copy-id user@192.168.1.200
```

### 6.3 Configure SSH Config

Edit `~/.ssh/config`:

```
Host omega1
    HostName 192.168.1.107
    User pi
    IdentityFile ~/.ssh/id_ed25519

Host omega1-tailscale
    HostName 100.x.x.x  # Pi Tailscale IP
    User pi
    IdentityFile ~/.ssh/id_ed25519

Host orin
    HostName 192.168.1.200
    User jetson
    IdentityFile ~/.ssh/id_ed25519
```

Now you can SSH with:
```bash
ssh omega1
ssh omega1-tailscale
ssh orin
```

## Step 7: Docker Setup (Optional)

### 7.1 Build ROS2 Docker Image

```bash
cd ~/Omega-Code/docker/ros2_robot
docker-compose build
```

### 7.2 Run ROS2 Nodes in Docker

```bash
# Start ROS2 core
docker-compose up -d

# View logs
docker-compose logs -f
```

## Step 8: Verification

### 8.1 Network Connectivity

```bash
# Ping Pi
ping -c 3 192.168.1.107

# Ping Jetson (if applicable)
ping -c 3 192.168.1.200

# Test Tailscale
tailscale ping omega1-1.hartley-ghost.ts.net
```

### 8.2 ROS2 Communication

```bash
# On Aurora
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 topic echo /omega/telemetry

# On Pi (via SSH)
ssh omega1
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 topic pub /omega/telemetry std_msgs/String "data: 'test'"
```

### 8.3 Backend API Access

```bash
# Test Pi backend
curl http://192.168.1.107:5000/health
curl http://192.168.1.107:8000/health
```

### 8.4 GPU Verification

```bash
# NVIDIA GPU
nvidia-smi

# CUDA
nvcc --version

# PyTorch CUDA
python3 -c "import torch; print(torch.cuda.is_available())"
```

## Step 9: Development Workflow

### 9.1 Typical Development Session

```bash
# 1. Start on Aurora
cd ~/Omega-Code

# 2. Source environment
source venv/bin/activate
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# 3. Build ROS2 packages
cd ros
colcon build

# 4. Launch development nodes
ros2 launch omega_robot omega_camera.launch.py

# 5. In another terminal, run visualization
rviz2
```

### 9.2 Remote Development

You can develop on Aurora and test on Pi:

```bash
# Build on Aurora
cd ~/Omega-Code/ros
colcon build

# Copy to Pi
scp -r install omega1:~/omega_ws/

# SSH to Pi and run
ssh omega1
cd ~/omega_ws
source install/setup.bash
ros2 run omega_robot camera_publisher
```

## Step 10: Capability Profile

The Alienware Aurora should be detected as a high-performance workstation. Update the capability detection:

### 10.1 Check Current Profile

```bash
cd ~/Omega-Code
./scripts/apply_profile.sh
cat /tmp/omega_capabilities.json
```

### 10.2 Expected Profile

```json
{
  "profile_mode": "aurora",
  "ml_capable": true,
  "slam_capable": true,
  "tracking": true,
  "aruco": true,
  "motion_detection": true,
  "face_recognition": true,
  "yolo": true,
  "max_resolution": "1920x1080",
  "max_fps": 60,
  "gpu_available": true,
  "gpu_name": "NVIDIA GeForce RTX XXX",
  "device_type": "workstation"
}
```

## Troubleshooting

### Network Issues

**Problem**: Can't ping Pi from Aurora
```bash
# Check network interface
ip addr show

# Check routing
ip route

# Test connectivity
ping -c 3 192.168.1.107
```

**Solution**: Ensure both devices are on the same subnet and firewall allows communication.

### ROS2 Communication Issues

**Problem**: Topics not visible across devices
```bash
# Check domain ID matches
echo $ROS_DOMAIN_ID

# Check CycloneDDS config
cat docker/ros2_robot/config/cyclonedds.xml

# Test with explicit interface
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

**Solution**: Ensure ROS_DOMAIN_ID matches on all devices and network interface is correctly configured.

### GPU Not Detected

**Problem**: CUDA not available
```bash
# Check NVIDIA driver
nvidia-smi

# Check CUDA installation
nvcc --version

# Check PyTorch CUDA
python3 -c "import torch; print(torch.cuda.is_available())"
```

**Solution**: Reinstall NVIDIA drivers and CUDA toolkit if needed.

### SSH Connection Issues

**Problem**: Can't SSH to Pi
```bash
# Test SSH
ssh -v pi@192.168.1.107

# Check SSH keys
ls -la ~/.ssh/

# Regenerate keys if needed
ssh-keygen -t ed25519 -C "aurora-desktop"
ssh-copy-id pi@192.168.1.107
```

## Next Steps

1. **Configure IDE**: Set up VS Code or your preferred IDE on Aurora
2. **Set up Git**: Configure git user and credentials
3. **Install Development Tools**: Docker, VS Code extensions, etc.
4. **Create Development Scripts**: Automation scripts for common tasks
5. **Set up CI/CD**: If using GitHub Actions or similar
6. **Document Workflows**: Create team documentation for using Aurora

## Configuration Summary

| Setting | Value |
|---------|-------|
| Hostname | `aurora-desktop` |
| IP Address | `192.168.1.150` |
| Tailscale IP | `100.x.x.x` (get from `tailscale ip -4`) |
| ROS2 Domain ID | `0` |
| Device Role | `workstation` |
| GPU | NVIDIA (if applicable) |
| OS | Ubuntu 22.04 LTS |

## Files Modified/Created

- `config/.env.script2` - Added Aurora configuration
- `scripts/connect_hotspot_v2.sh` - Added Aurora hostname support
- `.env.ros2.multidevice` - Added Aurora IP and role
- `ros/launch/multidevice_setup.launch.py` - Added Aurora nodes
- `docker/ros2_robot/config/cyclonedds.xml` - Updated network interface
- `~/.ssh/config` - Added SSH shortcuts

## Support

For issues or questions:
1. Check existing documentation in `/docs`
2. Review ROS2 multi-device setup guide
3. Check network connectivity first
4. Verify all configuration files are updated

---

**Status**: ✅ Integration Guide Complete  
**Last Updated**: 2024  
**Compatible With**: Ubuntu 22.04+, ROS2 Humble/Rolling

