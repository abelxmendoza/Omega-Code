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

## Implementation Details

### Core Detection System

**`ros/src/omega_robot/omega_robot/system_capabilities.py`**
- ROS2 node that auto-detects system capabilities
- Detects Jetson hardware, CUDA availability, ROS2 tools
- Publishes capability profile to `/omega/capabilities` topic
- Saves profile to `/tmp/omega_capabilities.json` for launch files
- Supports three profiles: `mac`, `lenovo`, `jetson`

**`ros/src/omega_robot/omega_robot/capability_utils.py`**
- Utility module for nodes to check capabilities
- Functions: `is_ml_capable()`, `is_slam_capable()`, `get_max_resolution()`, etc.
- `CapabilitySubscriber` class for subscribing to capability updates
- Loads profile from temp file or provides defaults

### Adaptive Launch Files

**`ros/launch/omega_camera.launch.py`**
- Automatically adapts camera and vision nodes based on profile
- Light mode: Basic tracking, ArUco
- Dev mode: + SLAM, navigation
- Omega mode: + GPU vision, YOLO, face recognition

**`ros/launch/omega_brain.launch.py`**
- Launches autonomy stack (only if not light mode)
- Navigation, path planning, action servers
- GPU-accelerated nodes on Jetson

**`ros/launch/omega_full.launch.py`**
- Launches complete system: camera + brain
- One command to start everything

### Profile Detection Script

**`scripts/apply_profile.sh`**
- Standalone script to detect capabilities
- Creates `/tmp/omega_capabilities.json`
- Can be run before launching ROS2 nodes
- Provides colored output for easy debugging

## App-Wide Integration

### Backend (Python)

**`servers/robot-controller-backend/api/capability_service.py`**
- Detects system capabilities (Jetson, CUDA, ROS2)
- Provides singleton service for capability queries
- Integrates with ROS2 capability profile if available
- Falls back to local detection

**`servers/robot-controller-backend/api/capability_routes.py`**
- REST API endpoints:
  - `GET /api/capabilities` - Get full capability profile
  - `GET /api/capabilities/check?feature=X` - Check specific feature
  - `GET /api/capabilities/resolution` - Get max resolution
  - `GET /api/capabilities/fps` - Get max FPS
  - `GET /api/capabilities/profile` - Get profile mode

**Video Server Integration**
- `video/video_server.py` now uses capability service for default resolution/FPS
- Automatically adapts camera settings based on profile

### Frontend (React/TypeScript)

**`src/hooks/useCapabilities.ts`**
- React hook to fetch and use capabilities
- Auto-refreshes every 30 seconds
- Provides convenience getters: `isMLCapable`, `isSLAMCapable`, etc.

**`src/context/CapabilityContext.tsx`**
- React context provider for app-wide capability access
- Wraps entire app in `_app.tsx`
- Provides hooks: `useCapabilityContext()`, `useIsMLCapable()`, etc.

**`src/components/capability/CapabilityStatus.tsx`**
- Displays current profile mode and capabilities
- Shows badges for enabled features
- Color-coded by profile (green=Jetson, blue=Lenovo, gray=Mac)

**`src/components/capability/CapabilityGate.tsx`**
- Conditional rendering component
- `<CapabilityGate feature="ml_capable">` - Only renders if ML available
- `<ProfileGate mode="jetson">` - Only renders in Jetson mode

### Usage Examples

**Backend Usage**:
```python
from api.capability_service import get_capability_service

service = get_capability_service()

# Check capabilities
if service.is_ml_capable():
    # Use GPU processing
    pass

# Get max resolution
width, height = service.get_max_resolution()
fps = service.get_max_fps()
```

**Frontend Usage**:
```tsx
import { useCapabilities } from '@/hooks/useCapabilities';

function MyComponent() {
  const { isMLCapable, isSLAMCapable, profileMode } = useCapabilities();
  
  return (
    <div>
      {isMLCapable && <GPUFeatures />}
      {isSLAMCapable && <SLAMFeatures />}
    </div>
  );
}
```

**Using Capability Gate**:
```tsx
import { CapabilityGate, ProfileGate } from '@/components/capability';

function Dashboard() {
  return (
    <div>
      {/* Only show YOLO if ML capable */}
      <CapabilityGate feature="ml_capable">
        <YOLODetection />
      </CapabilityGate>
      
      {/* Only show SLAM in Jetson mode */}
      <ProfileGate mode="jetson">
        <SLAMVisualization />
      </ProfileGate>
    </div>
  );
}
```

### API Endpoints

**Get Full Profile**:
```bash
curl http://localhost:8000/api/capabilities
```

**Check Specific Feature**:
```bash
curl http://localhost:8000/api/capabilities/check?feature=ml_capable
```

## Next Steps

1. **Add YOLO Node**: Create GPU-accelerated YOLO detection node for Jetson
2. **Visual SLAM**: Integrate ORB-SLAM3 or similar for Jetson
3. **Semantic Navigation**: Add semantic segmentation for navigation
4. **Multi-Camera**: Support multiple camera streams on Jetson
5. **Profile Persistence**: Save profile preferences across reboots
6. **Capability-aware UI themes**: Show capability warnings in UI
7. **Capability comparison view**: Add capability-based feature recommendations

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

