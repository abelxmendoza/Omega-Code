# Omega Capability System - Quick Reference

## 🚀 Quick Start

```bash
# 1. Detect capabilities
./scripts/apply_profile.sh

# 2. Launch camera (auto-adapts)
ros2 launch omega_robot omega_camera.launch.py

# 3. Launch full system
ros2 launch omega_robot omega_full.launch.py
```

## 📊 Profile Comparison

| Feature | MacBook + Pi | Lenovo + Pi | Jetson + Pi |
|---------|--------------|-------------|-------------|
| **Mode** | Light | Dev | Omega |
| **ML/GPU** | ❌ | ❌ | ✅ |
| **SLAM** | ❌ | ✅ | ✅ GPU |
| **YOLO** | ❌ | ❌ | ✅ |
| **Face Rec** | ❌ | ✅ (slow) | ✅ (fast) |
| **Tracking** | ✅ | ✅ | ✅ |
| **ArUco** | ✅ | ✅ | ✅ |
| **Max Res** | 640x480 | 1280x720 | 1920x1080 |
| **Max FPS** | 20 | 25 | 60 |

## 🔍 Capability Detection

### Check Current Profile

```bash
# View profile file
cat /tmp/omega_capabilities.json

# Or subscribe to ROS2 topic
ros2 topic echo /omega/capabilities
```

### Manual Detection

```bash
# Run detection script
./scripts/apply_profile.sh

# Output shows detected profile
```

## 🎯 Using in Code

### Simple Check

```python
from omega_robot.capability_utils import (
    is_ml_capable,
    is_slam_capable,
    get_max_resolution,
    get_max_fps
)

if is_ml_capable():
    # Use GPU
    pass

width, height = get_max_resolution()
```

### Subscribe to Updates

```python
from omega_robot.capability_utils import CapabilitySubscriber

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.caps = CapabilitySubscriber(self)
        
    def process(self):
        if self.caps.is_ml_capable():
            # GPU processing
            pass
```

## 📁 Key Files

- `ros/src/omega_robot/omega_robot/system_capabilities.py` - Detection node
- `ros/src/omega_robot/omega_robot/capability_utils.py` - Helper utilities
- `ros/launch/omega_camera.launch.py` - Camera launch (adaptive)
- `ros/launch/omega_brain.launch.py` - Autonomy launch (adaptive)
- `scripts/apply_profile.sh` - Detection script
- `/tmp/omega_capabilities.json` - Profile cache

## 🛠️ Troubleshooting

### Wrong Profile?

```bash
# Check Jetson
cat /proc/device-tree/model

# Check CUDA
nvcc --version
python3 -c "import torch; print(torch.cuda.is_available())"

# Force profile
ros2 launch omega_robot omega_camera.launch.py profile:=jetson
```

### Profile Not Updating?

```bash
# Restart detector
ros2 run omega_robot system_capabilities

# Or manually apply
./scripts/apply_profile.sh
```

## 📝 Profile JSON Structure

```json
{
  "profile_mode": "jetson",
  "ml_capable": true,
  "slam_capable": true,
  "max_resolution": "1920x1080",
  "max_fps": 60,
  "gpu_available": true
}
```

## 🎨 Launch File Overrides

```bash
# Override resolution
ros2 launch omega_robot omega_camera.launch.py width:=1280 height:=720

# Override profile
ros2 launch omega_robot omega_camera.launch.py profile:=jetson

# Override FPS
ros2 launch omega_robot omega_camera.launch.py fps:=30
```

---

**See**: `OMEGA_MULTI_PLATFORM_GUIDE.md` for full documentation

