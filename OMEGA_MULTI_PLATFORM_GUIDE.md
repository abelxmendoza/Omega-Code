# Omega-1 Multi-Platform Capability System

## Overview

The Omega-1 robot automatically adapts to your development environment, running only features it can handle. No overloading, no wasted cycles, no broken pipelines.

## Supported Platforms

### Profile A: MacBook + Raspberry Pi (Light Mode)
- **Capabilities**: Basic ROS2, camera streaming, light CV
- **Enabled**: Motion detection, object tracking (MOSSE/KCF), ArUco detection
- **Disabled**: ML inference, SLAM, GPU processing, YOLO, face recognition
- **Max Resolution**: 640x480 @ 20 FPS

### Profile B: Lenovo Linux + Raspberry Pi (Dev Mode)
- **Capabilities**: Full ROS2, RViz, navigation stack
- **Enabled**: Everything from Profile A + SLAM, navigation, path planning, ArUco pose estimation
- **Disabled**: GPU ML, high-FPS inference, real-time detection
- **Max Resolution**: 1280x720 @ 25 FPS

### Profile C: Jetson Orin Nano + Raspberry Pi (Omega Mode)
- **Capabilities**: CUDA, TensorRT, GPU ML inference, real-time SLAM
- **Enabled**: Everything + YOLO, SAM segmentation, face recognition, visual SLAM, semantic navigation
- **Disabled**: Nothing - full autonomy mode
- **Max Resolution**: 1920x1080 @ 60 FPS

## Quick Start

### 1. Apply Profile

```bash
# Run capability detection
./scripts/apply_profile.sh

# This creates /tmp/omega_capabilities.json
```

### 2. Launch Camera System

```bash
# Automatically adapts to detected capabilities
ros2 launch omega_robot omega_camera.launch.py
```

### 3. Launch Full System

```bash
# Launches camera + autonomy stack (if capable)
ros2 launch omega_robot omega_full.launch.py
```

## Architecture

### Capability Detection

The `system_capabilities` node automatically detects:
- Hardware type (Jetson vs. regular Linux vs. macOS)
- CUDA availability
- ROS2 development tools
- GPU capabilities
- Maximum supported resolution/FPS

### Capability Profile

Published to `/omega/capabilities` topic and saved to `/tmp/omega_capabilities.json`:

```json
{
  "profile_mode": "jetson",
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
  "gpu_name": "NVIDIA Tegra Orin"
}
```

### Launch Files

#### `omega_camera.launch.py`
- Always runs: Capability detector, camera publisher
- Light mode: Basic tracking, ArUco
- Dev mode: + SLAM, navigation
- Omega mode: + GPU vision, YOLO, face recognition

#### `omega_brain.launch.py`
- Launches autonomy stack (only if not light mode)
- Navigation, path planning, action servers
- GPU-accelerated nodes on Jetson

#### `omega_full.launch.py`
- Launches everything: camera + brain
- One command to start complete system

## Using Capabilities in Your Nodes

### Method 1: Load Profile Directly

```python
from omega_robot.capability_utils import (
    get_capability_profile,
    is_ml_capable,
    is_slam_capable,
    get_max_resolution,
    get_max_fps
)

# Check capabilities
profile = get_capability_profile()
if is_ml_capable():
    # Run ML inference
    pass

width, height = get_max_resolution()
max_fps = get_max_fps()
```

### Method 2: Subscribe to Updates

```python
from omega_robot.capability_utils import CapabilitySubscriber

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.capabilities = CapabilitySubscriber(self)
        
    def process(self):
        if self.capabilities.is_ml_capable():
            # Use GPU
            pass
```

## Operating Modes Summary

| Combo | Mode | ML | SLAM | Tracking | ArUco | YOLO | Face Rec | FPS |
|-------|------|----|------|-----------|-------|------|-----------|-----|
| MacBook + Pi | light_mode | ❌ | ❌ | ✔ | ✔ | ❌ | ❌ | ~20 |
| Lenovo + Pi | dev_mode | ❌ | ✔ | ✔ | ✔ | ❌ | ✔(slow) | ~25 |
| Jetson + Pi | omega_mode | ✔ CUDA | ✔ GPU | ✔ | ✔ | ✔ | ✔ | 30-60 |

## Manual Override

You can override the detected profile:

```bash
ros2 launch omega_robot omega_camera.launch.py profile:=jetson
```

## Integration with Existing Nodes

### Camera Publisher
- Automatically uses max resolution from profile
- Adjusts FPS based on capabilities
- Publishes compressed images for web

### Vision Processor
- Checks `ml_capable` before enabling GPU
- Falls back to CPU processing if GPU unavailable
- Respects profile mode

### Video Server
- Motion detection: Always enabled
- Object tracking: Enabled (CPU trackers)
- Face recognition: Only if `face_recognition: true`
- ArUco detection: Always enabled

## Troubleshooting

### Profile Not Detected

```bash
# Manually run detection
./scripts/apply_profile.sh

# Check profile file
cat /tmp/omega_capabilities.json
```

### Wrong Profile Detected

```bash
# Check Jetson detection
cat /proc/device-tree/model

# Check CUDA
nvcc --version
python3 -c "import torch; print(torch.cuda.is_available())"
```

### Capabilities Not Updating

The capability detector publishes every 5 seconds. If you need immediate update:

```bash
# Restart capability detector
ros2 run omega_robot system_capabilities
```

## File Structure

```
ros/
├── launch/
│   ├── omega_camera.launch.py      # Camera + vision
│   ├── omega_brain.launch.py        # Autonomy stack
│   └── omega_full.launch.py         # Complete system
├── src/omega_robot/
│   ├── omega_robot/
│   │   ├── system_capabilities.py   # Detection node
│   │   ├── capability_utils.py      # Helper utilities
│   │   ├── camera_publisher.py      # Camera node
│   │   └── vision_processor.py      # Vision processing
scripts/
└── apply_profile.sh                 # Profile detection script
```

## Next Steps

1. **Add YOLO Node**: Create GPU-accelerated YOLO detection node for Jetson
2. **Visual SLAM**: Integrate ORB-SLAM3 or similar for Jetson
3. **Semantic Navigation**: Add semantic segmentation for navigation
4. **Multi-Camera**: Support multiple camera streams on Jetson
5. **Profile Persistence**: Save profile preferences across reboots

## Examples

### Example 1: Capability-Aware Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from omega_robot.capability_utils import CapabilitySubscriber

class SmartVisionNode(Node):
    def __init__(self):
        super().__init__('smart_vision')
        self.capabilities = CapabilitySubscriber(self)
        
        if self.capabilities.is_ml_capable():
            self.get_logger().info("Using GPU-accelerated processing")
            # Initialize GPU models
        else:
            self.get_logger().info("Using CPU processing")
            # Initialize CPU algorithms
            
    def process_frame(self, frame):
        if self.capabilities.is_ml_capable():
            return self.process_gpu(frame)
        else:
            return self.process_cpu(frame)
```

### Example 2: Adaptive Resolution

```python
from omega_robot.capability_utils import get_max_resolution

width, height = get_max_resolution()
camera.set_resolution(width, height)
```

---

**Status**: ✅ Implemented  
**Last Updated**: 2024  
**Compatible With**: ROS2 Humble/Rolling

