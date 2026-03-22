# Multi-Platform & Hardware Platforms Guide

## Ecosystem Overview

```
┌──────────────────────────────────────────────────────────────┐
│                  Omega-1 Device Ecosystem                    │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐     │
│  │ Dev Machine  │   │ Raspberry    │   │ Jetson Orin  │     │
│  │ (Mac/Lenovo/ │   │ Pi 4B        │   │ Nano         │     │
│  │  Alienware)  │   │              │   │              │     │
│  │              │   │ Hardware IO  │   │ AI Compute   │     │
│  │ Dev Cockpit  │◄──┤ • Motors     │◄──┤ • Vision     │     │
│  │ • RViz       │   │ • GPIO       │   │ • SLAM       │     │
│  │ • SSH mgmt   │   │ • Sensors    │   │ • ML/CUDA    │     │
│  │ • Docker     │   │ • Real-time  │   │ • YOLO       │     │
│  └──────────────┘   └──────────────┘   └──────────────┘     │
│                                                              │
│              ROS2 DDS (CycloneDDS) — Domain ID: 0            │
└──────────────────────────────────────────────────────────────┘
```

---

## Capability Profiles

Omega-1 auto-detects your hardware and runs only what it can handle.

| Profile | Hardware | Mode | ML | SLAM | YOLO | Face Rec | Max FPS |
|---------|----------|------|----|------|------|----------|---------|
| A | MacBook + Pi | light_mode | — | — | — | — | ~20 |
| B | Lenovo + Pi | dev_mode | — | CPU | — | slow | ~25 |
| C | Alienware + Pi | dev_mode | GPU | GPU | — | fast | ~30 |
| D | Jetson + Pi | omega_mode | CUDA | GPU | CUDA | CUDA | 30-60 |

All profiles always have: motion detection, object tracking (KCF/MOSSE), ArUco detection.

### Apply a profile

```bash
# Auto-detect and apply
./scripts/apply_profile.sh

# Force a specific profile
ros2 launch omega_robot omega_camera.launch.py profile:=jetson

# Check current profile
cat /tmp/omega_capabilities.json
```

### Use capabilities in ROS2 nodes

```python
from omega_robot.capability_utils import is_ml_capable, get_max_resolution, get_max_fps

if is_ml_capable():
    # GPU/CUDA path
    pass

width, height = get_max_resolution()
```

---

## Raspberry Pi 4B (Hardware IO)

**Role:** Low-level hardware — motors, GPIO, sensors, real-time control

The Pi runs ROS2 inside Docker. See [SETUP.md](SETUP.md) for the full Docker workflow.

**Capabilities:**
- Motor drivers (PCA9685)
- GPIO sensors (HC-SR04 ultrasonic, line tracking)
- I2C, SPI, PWM
- Pi camera (Picamera2/libcamera)
- Lightweight ROS2 nodes via Docker

---

## Jetson Orin Nano (AI Compute)

**Role:** Heavy computation — AI vision, SLAM, ML inference
**SSH:** `ssh omega1@100.107.112.110` (Tailscale IP)
**OS:** Ubuntu 22.04 + JetPack 5.x

### Initial setup

```bash
# SSH to Jetson
ssh omega1@100.107.112.110

# Run setup script
cd ~/Omega-Code
chmod +x scripts/setup_jetson_orin.sh
./scripts/setup_jetson_orin.sh
```

The setup script installs ROS2 Humble, configures CUDA, sets up CycloneDDS, and installs ML dependencies.

### Run AI nodes on Jetson

```bash
# Vision processor
ros2 run omega_robot vision_processor

# Object detection (YOLO)
ros2 run omega_robot object_detector

# SLAM
ros2 run omega_robot slam_processor
```

### Monitor Jetson performance

```bash
sudo tegrastats    # system stats
nvidia-smi         # GPU stats
htop               # CPU/RAM
```

### CycloneDDS config

Verify Jetson is listed in the DDS config:
```bash
cat docker/ros2_robot/config/cyclonedds.xml
# Should contain: <Peer address="100.107.112.110"/>
```

Copy config to Jetson:
```bash
scp docker/ros2_robot/config/cyclonedds.xml \
    omega1@100.107.112.110:~/omega_ws/config/
```

---

## Alienware Aurora (High-Performance Dev)

**Role:** GPU-accelerated development workstation, ML training, heavy simulation
**OS:** Ubuntu 22.04 (or WSL2 on Windows)

**Capabilities beyond laptop:**
- GPU ML training (full batch training vs laptop inference only)
- High-end RViz/Gazebo simulation
- Fast Docker image builds
- Parallel colcon builds

### Setup

```bash
# Clone repo
git clone git@github.com:abelxmendoza/Omega-Code.git ~/Omega-Code
cd ~/Omega-Code

# Run multi-device setup
./scripts/setup_multidevice_ros2.sh
```

The script auto-detects the device type, installs ROS2 Humble, configures CycloneDDS, and optimizes the system.

---

## Multi-Device ROS2 Setup

### Step 1: Configure device IPs

```bash
cd ~/Omega-Code
cp .env.ros2.multidevice.example .env.ros2.multidevice
```

Edit `.env.ros2.multidevice`:
```bash
LAPTOP_IP=192.168.1.100
PI_IP=192.168.1.107
ORIN_IP=100.107.112.110
ROS_DOMAIN_ID=0
```

### Step 2: Run setup on each device

```bash
# On laptop, Pi, and Orin — each device auto-detects its role
./scripts/setup_multidevice_ros2.sh
```

### Step 3: Generate and distribute CycloneDDS config

```bash
# Generate config with all device IPs
source .env.ros2.multidevice
envsubst < docker/ros2_robot/config/cyclonedds.xml.template > \
  docker/ros2_robot/config/cyclonedds.xml

# Distribute to all devices
scp docker/ros2_robot/config/cyclonedds.xml pi@${PI_IP}:~/omega_ws/config/
scp docker/ros2_robot/config/cyclonedds.xml omega1@${ORIN_IP}:~/omega_ws/config/
```

### Step 4: Set environment on each device

Add to `~/.bashrc` on every machine:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/Omega-Code/docker/ros2_robot/config/cyclonedds.xml

source /opt/ros/humble/setup.bash
if [ -f "$HOME/omega_ws/install/setup.bash" ]; then
    source "$HOME/omega_ws/install/setup.bash"
fi
```

### Step 5: Test cross-device communication

```bash
# Terminal 1: Laptop — publish
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run demo_nodes_cpp talker

# Terminal 2: Jetson via SSH — subscribe
ssh omega1@100.107.112.110
source ~/.ros2_jetson_setup.bash
ros2 run demo_nodes_cpp listener
```

### Build workspace on Jetson

```bash
ssh omega1@100.107.112.110
cd ~/omega_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Launch Configurations

| Launch File | Best For |
|-------------|----------|
| `pi_only.launch.py` | Pi standalone (sensors + motors) |
| `pi_orin_hybrid.launch.py` | Pi hardware + Orin AI |
| `omega_camera.launch.py` | Camera + vision (auto-selects based on profile) |
| `omega_brain.launch.py` | Autonomy stack (skips if light mode) |
| `omega_full.launch.py` | Everything: camera + brain + sensors |

---

## Troubleshooting

### Topics not visible across devices

```bash
# Verify ROS_DOMAIN_ID matches on all machines
echo $ROS_DOMAIN_ID

# Verify CycloneDDS config is identical on all machines
cat ~/omega_ws/config/cyclonedds.xml

# Test connectivity
ping <other-device-ip>
```

### Wrong capability profile detected

```bash
# Check what hardware is detected
cat /proc/device-tree/model  # Jetson check
nvcc --version               # CUDA check
python3 -c "import torch; print(torch.cuda.is_available())"

# Re-run detection
./scripts/apply_profile.sh
cat /tmp/omega_capabilities.json
```

### Capabilities not updating

The capability detector publishes every 5 seconds. Force a refresh:
```bash
./scripts/apply_profile.sh
```

---

## Capability API & Frontend Integration

### Backend API endpoints

```bash
# Full capability profile
curl http://localhost:8000/api/capabilities

# Check a specific feature
curl "http://localhost:8000/api/capabilities/check?feature=ml_capable"

# Get max resolution / FPS
curl http://localhost:8000/api/capabilities/resolution
curl http://localhost:8000/api/capabilities/fps

# Get profile mode
curl http://localhost:8000/api/capabilities/profile
```

Example response:
```json
{
  "ok": true,
  "capabilities": {
    "profile_mode": "jetson",
    "ml_capable": true,
    "slam_capable": true,
    "max_resolution": "1920x1080",
    "max_fps": 60,
    "gpu_available": true
  }
}
```

### Backend usage (Python)

```python
from api.capability_service import get_capability_service

service = get_capability_service()
if service.is_ml_capable():
    # GPU processing path
    pass
width, height = service.get_max_resolution()
fps = service.get_max_fps()
```

### Frontend usage (React/TypeScript)

```tsx
import { useCapabilities } from '@/hooks/useCapabilities';
import { CapabilityGate, ProfileGate } from '@/components/capability';

// Hook
const { isMLCapable, isSLAMCapable, profileMode } = useCapabilities();

// Conditional rendering
<CapabilityGate feature="ml_capable">
  <YOLODetection />
</CapabilityGate>

<CapabilityGate feature="face_recognition" fallback={<p>Requires Jetson</p>}>
  <FaceRecognition />
</CapabilityGate>

<ProfileGate mode="jetson">
  <GPUAcceleratedFeatures />
</ProfileGate>
```

Key files:
- `api/capability_service.py` — detects capabilities, singleton service
- `api/capability_routes.py` — REST endpoints
- `src/hooks/useCapabilities.ts` — React hook, auto-refreshes every 30s
- `src/context/CapabilityContext.tsx` — app-wide provider
- `src/components/capability/CapabilityGate.tsx` — conditional rendering
- `src/components/capability/CapabilityStatus.tsx` — header status badge
