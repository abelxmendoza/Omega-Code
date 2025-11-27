# ROS2 Multi-Device Setup Guide

Complete guide for setting up ROS2 Humble across **Laptop (Ubuntu)**, **Raspberry Pi 4B**, and **Jetson Orin Nano** for Omega-1 robot development.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Omega-1 Multi-Device ROS2                │
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
│              Domain ID: 0 (configurable)                   │
└─────────────────────────────────────────────────────────────┘
```

## Device Roles

### 1. Laptop (Ubuntu) - Development Cockpit
**Hardware**: i5-8265U + 8GB RAM  
**Role**: Development, visualization, orchestration

**Capabilities**:
- ✅ Writing ROS2 nodes
- ✅ Building packages with colcon
- ✅ Running RViz (light settings)
- ✅ Coding, Git, debugging
- ✅ SSH into Pi + Orin
- ✅ Managing Docker containers
- ❌ Heavy simulation (move to Orin)
- ❌ GPU workloads (move to Orin)

### 2. Jetson Orin Nano - AI Compute Engine
**Hardware**: NVIDIA Jetson Orin Nano  
**Role**: Heavy computation, AI, vision

**Capabilities**:
- ✅ AI vision processing
- ✅ Object detection
- ✅ SLAM (Simultaneous Localization and Mapping)
- ✅ Heavy simulation
- ✅ CUDA-accelerated processing
- ✅ ML model inference
- ✅ High-level ROS2 nodes (perception stack)

### 3. Raspberry Pi 4B - Hardware IO Controller
**Hardware**: Raspberry Pi 4B  
**Role**: Low-level hardware control

**Capabilities**:
- ✅ Motor drivers
- ✅ GPIO sensors
- ✅ Micro-ROS or lightweight ROS2 nodes
- ✅ Serial, I2C, SPI, PWM
- ✅ Real-time hardware control

## Prerequisites

### All Devices
- Ubuntu 22.04 (or compatible)
- Network connectivity (same subnet or VPN)
- SSH access between devices
- Docker installed (optional, for containerized nodes)

### Laptop Specific
- 8GB+ RAM recommended
- i5 or equivalent CPU
- X11 display server (for RViz)

## Quick Start

### Step 1: Configure Environment

```bash
cd ~/Omega-Code
cp .env.ros2.multidevice.example .env.ros2.multidevice
```

Edit `.env.ros2.multidevice` with your device IPs:

```bash
LAPTOP_IP=192.168.1.100
PI_IP=192.168.1.107
ORIN_IP=192.168.1.200
ROS_DOMAIN_ID=0
```

### Step 2: Run Setup Script

On **each device** (laptop, Pi, Orin):

```bash
cd ~/Omega-Code
./scripts/setup_multidevice_ros2.sh
```

The script will:
- Auto-detect device type
- Install ROS2 Humble
- Configure CycloneDDS
- Set up workspace
- Optimize system (laptop only)

### Step 3: Generate CycloneDDS Config

The setup script automatically generates `docker/ros2_robot/config/cyclonedds.xml` with all device IPs.

**Manual generation**:

```bash
cd ~/Omega-Code
source .env.ros2.multidevice
envsubst < docker/ros2_robot/config/cyclonedds.xml.template > \
  docker/ros2_robot/config/cyclonedds.xml
```

### Step 4: Copy Config to All Devices

```bash
# Copy to Pi
scp docker/ros2_robot/config/cyclonedds.xml \
  pi@${PI_IP}:~/omega_ws/config/cyclonedds.xml

# Copy to Orin
scp docker/ros2_robot/config/cyclonedds.xml \
  jetson@${ORIN_IP}:~/omega_ws/config/cyclonedds.xml
```

### Step 5: Set Environment Variables

On **each device**, add to `~/.bashrc`:

```bash
# ROS2 Multi-Device Setup
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/Omega-Code/docker/ros2_robot/config/cyclonedds.xml

# Source ROS2
source /opt/ros/humble/setup.bash
if [ -f "$HOME/omega_ws/install/setup.bash" ]; then
    source "$HOME/omega_ws/install/setup.bash"
fi
```

### Step 6: Test Communication

**Terminal 1 (Laptop)**:
```bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 (Pi or Orin)**:
```bash
ros2 run demo_nodes_cpp listener
```

You should see messages being received across devices!

## Detailed Setup

### Laptop Setup

#### 1. Install ROS2 Humble

```bash
./scripts/setup_ros2_laptop.sh
```

This installs:
- ROS2 Humble Desktop
- CycloneDDS RMW
- RViz2
- colcon build tools
- Docker (if not installed)

#### 2. Optimize System

```bash
./scripts/optimize_laptop_ros2.sh
```

Optimizations include:
- Swap configuration (4GB)
- CPU governor (performance mode)
- Network tuning for DDS
- File descriptor limits
- Docker optimization
- RViz settings

#### 3. Build Workspace

```bash
mkdir -p ~/omega_ws/src
cd ~/omega_ws/src
ln -s ~/Omega-Code/ros/src/omega_robot .
cd ~/omega_ws
colcon build --symlink-install
```

**Optimized build** (for 8GB RAM):
```bash
~/.ros2_optimize_build.sh
```

### Pi 4B Setup

#### 1. Install ROS2 Humble

Pi OS is Ubuntu-based, so use the same laptop script:

```bash
./scripts/setup_ros2_laptop.sh
```

#### 2. Configure for Hardware

```bash
# Enable GPIO
sudo usermod -aG gpio $USER

# Enable I2C/SPI if needed
sudo raspi-config
# Navigate to Interface Options → Enable I2C/SPI
```

#### 3. Build Workspace

```bash
mkdir -p ~/omega_ws/src
cd ~/omega_ws/src
# Clone or link Omega-Code packages
cd ~/omega_ws
colcon build --symlink-install
```

### Jetson Orin Nano Setup

#### 1. Install ROS2 Humble

Orin runs Ubuntu, so use the laptop script:

```bash
./scripts/setup_ros2_laptop.sh
```

#### 2. Configure CUDA

```bash
# Verify CUDA
nvcc --version

# Set CUDA environment
export CUDA_VISIBLE_DEVICES=0
```

#### 3. Build Workspace

```bash
mkdir -p ~/omega_ws/src
cd ~/omega_ws/src
# Clone or link Omega-Code packages
cd ~/omega_ws
colcon build --symlink-install
```

## Launching Multi-Device System

### Option 1: Launch File

```bash
# On laptop
ros2 launch omega_robot multidevice_setup.launch.py
```

### Option 2: Docker Compose

```bash
cd ~/Omega-Code/docker/ros2_robot
docker-compose -f docker-compose.multidevice.yml --profile laptop up -d
```

### Option 3: Manual Node Launch

**Laptop**:
```bash
ros2 run omega_robot telemetry_listener
rviz2
```

**Pi 4B** (via SSH):
```bash
ssh pi@${PI_IP}
ros2 run omega_robot telemetry_publisher
```

**Orin Nano** (via SSH):
```bash
ssh jetson@${ORIN_IP}
ros2 run omega_robot vision_processor
```

## Network Configuration

### CycloneDDS Peer Discovery

The `cyclonedds.xml` file configures peer discovery:

```xml
<Peers>
  <Peer address="192.168.1.100"/>  <!-- Laptop -->
  <Peer address="192.168.1.107"/>  <!-- Pi 4B -->
  <Peer address="192.168.1.200"/>  <!-- Orin Nano -->
</Peers>
```

### Firewall Rules

**Ubuntu (all devices)**:
```bash
sudo ufw allow 7400:7500/udp  # DDS discovery
sudo ufw allow 22/tcp         # SSH
```

### Network Troubleshooting

```bash
# Check DDS discovery
ros2 daemon stop
ros2 daemon start
ros2 topic list

# Test connectivity
ping ${PI_IP}
ping ${ORIN_IP}

# Check CycloneDDS config
echo $CYCLONEDDS_URI
cat $CYCLONEDDS_URI
```

## Development Workflow

### 1. Code on Laptop

```bash
cd ~/Omega-Code
# Edit ROS packages in ros/src/omega_robot/
```

### 2. Build Locally

```bash
cd ~/omega_ws
colcon build --symlink-install
```

### 3. Deploy to Devices

```bash
# Sync workspace to Pi
rsync -avz ~/omega_ws/ pi@${PI_IP}:~/omega_ws/

# Sync workspace to Orin
rsync -avz ~/omega_ws/ jetson@${ORIN_IP}:~/omega_ws/
```

### 4. Build on Remote Devices

```bash
# On Pi
ssh pi@${PI_IP}
cd ~/omega_ws && colcon build

# On Orin
ssh jetson@${ORIN_IP}
cd ~/omega_ws && colcon build
```

## Monitoring & Debugging

### System Monitoring

```bash
# Laptop monitoring script
~/.ros2_monitor.sh
```

### ROS2 Topics

```bash
# List all topics (across all devices)
ros2 topic list

# Echo topic from any device
ros2 topic echo /omega/telemetry

# Check topic info
ros2 topic info /omega/telemetry
```

### Node Status

```bash
# List all nodes
ros2 node list

# Node info
ros2 node info /node_name
```

### Performance Monitoring

```bash
# CPU/Memory
htop

# ROS2 processes
ps aux | grep ros2

# Network traffic
sudo iftop
```

## Troubleshooting

### Issue: Topics not visible across devices

**Solution**:
1. Verify `ROS_DOMAIN_ID` matches on all devices
2. Check CycloneDDS config has all device IPs
3. Verify firewall allows UDP 7400-7500
4. Restart ROS2 daemon: `ros2 daemon stop && ros2 daemon start`

### Issue: RViz slow on laptop

**Solution**:
1. Run optimization script: `./scripts/optimize_laptop_ros2.sh`
2. Reduce RViz frame rate: `export RVIZ_FPS=15`
3. Disable anti-aliasing in RViz settings
4. Close unnecessary applications

### Issue: Build fails on Pi/Orin

**Solution**:
1. Check available memory: `free -h`
2. Reduce parallel jobs: `colcon build --parallel-workers 2`
3. Build packages individually
4. Check disk space: `df -h`

### Issue: SSH connection issues

**Solution**:
1. Verify SSH keys: `ssh-copy-id user@device_ip`
2. Test connectivity: `ping device_ip`
3. Check SSH config: `~/.ssh/config`
4. Verify user permissions on remote device

## Best Practices

### 1. Version Control
- Keep ROS packages in Git
- Use tags for stable releases
- Branch per feature

### 2. Workspace Management
- Use symlink install: `colcon build --symlink-install`
- Keep source separate from install
- Use workspace overlays

### 3. Network Security
- Use VPN for remote access
- Restrict firewall rules
- Use SSH keys (no passwords)

### 4. Resource Management
- Monitor CPU/memory on all devices
- Use Docker for isolation
- Limit parallel builds on Pi

### 5. Development Workflow
- Develop on laptop
- Test on target device
- Deploy via rsync/SSH
- Use launch files for orchestration

## Advanced Topics

### Custom RMW Implementation

```bash
# Use FastRTPS instead of CycloneDDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Multiple Domain IDs

```bash
# Isolate different robot systems
export ROS_DOMAIN_ID=1  # Robot 1
export ROS_DOMAIN_ID=2  # Robot 2
```

### Docker Deployment on Raspberry Pi

**Architecture**:
```
┌──────────────┐         ┌──────────────┐
│   Laptop     │         │  Raspberry   │
│  (Ubuntu)    │         │     Pi 4B    │
│              │         │              │
│ Native ROS2  │◄───────►│ Docker ROS2  │
│   Rolling    │  DDS    │   Humble     │
│              │ Network │              │
│ • rclpy      │         │ • Containers │
│ • Direct     │         │ • Pi OS      │
└──────────────┘         └──────────────┘
```

**Pi Setup (Raspberry Pi OS)**:

1. **Install Docker**:
```bash
# On Pi
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker pi
```

2. **Build ROS2 Docker Image**:
```bash
# On Pi
cd ~/Omega-Code/docker/ros2_robot
sudo docker build -t omega_robot:latest -f Dockerfile ../..
```

3. **Configure CycloneDDS**:
Update `docker/ros2_robot/config/cyclonedds.xml` on Pi:
```xml
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer address="192.168.1.100"/> <!-- Laptop IP -->
        <Peer address="192.168.1.200"/> <!-- Jetson IP -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

4. **Run Docker Compose**:
```bash
cd ~/Omega-Code/docker/ros2_robot
docker-compose up -d

# Check status
docker-compose ps

# View logs
docker-compose logs -f telemetry_publisher
```

5. **Execute ROS2 Commands in Container**:
```bash
# List topics
docker exec -it ros2_robot_telemetry_publisher_1 \
  bash -c "source /opt/ros/humble/setup.bash && \
           source /root/omega_ws/install/setup.bash && \
           ros2 topic list"

# Run nodes
docker exec -it ros2_robot_telemetry_publisher_1 \
  bash -c "source /opt/ros/humble/setup.bash && \
           source /root/omega_ws/install/setup.bash && \
           ros2 run omega_robot sensor_data_publisher"
```

**Communication Between Laptop and Pi**:

- Laptop uses **native ROS2** (direct rclpy)
- Pi uses **Docker ROS2** (containerized)
- Both use same `ROS_DOMAIN_ID` (default: 0)
- Both use CycloneDDS with peer addresses configured
- Topics are visible across both devices

**Benefits of Docker on Pi**:
- ✅ Isolated environment
- ✅ Easy updates (rebuild container)
- ✅ Consistent ROS2 version
- ✅ No conflicts with Pi OS packages
- ✅ Easy cleanup (remove container)

See `docker/ros2_robot/docker-compose.multidevice.yml` for containerized deployment.

### CI/CD Integration

Use GitHub Actions or similar to:
- Build ROS packages
- Run tests
- Deploy to devices
- Monitor system health

## Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [CycloneDDS Configuration](https://github.com/eclipse-cyclonedds/cyclonedds)
- [Multi-Robot Systems](https://robotics.stackexchange.com/questions/tagged/ros2)
- [Omega-Code Repository](https://github.com/yourusername/Omega-Code)

## Support

For issues or questions:
1. Check troubleshooting section
2. Review ROS2 logs: `~/.ros2/log/`
3. Check device connectivity
4. Verify environment variables

---

**Last Updated**: 2024  
**ROS2 Version**: Humble Hawksbill  
**Tested On**: Ubuntu 22.04

