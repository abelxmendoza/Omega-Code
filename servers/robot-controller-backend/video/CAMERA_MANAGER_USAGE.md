# CameraManager Usage Guide

## Overview

CameraManager implements intelligent camera backend detection and selection, automatically choosing the best backend based on hardware and available devices.

## Features

✅ **Automatic Detection**:
- Detects Raspberry Pi ribbon cameras (libcamera/Picamera2)
- Detects USB webcams (UVC/V4L2)
- Detects hardware platform (Pi, Jetson, Linux, Mac)

✅ **Deterministic Selection**:
- Picamera2 for ribbon cameras (unicam driver)
- V4L2 for USB cameras (uvcvideo driver)
- Mock camera as final fallback

✅ **Clean Logging**:
- Color-coded detection results
- Hardware and device information
- Backend selection reasoning

## Usage

### Basic Usage

CameraManager is enabled by default. Just use the Camera class as normal:

```python
from video.camera import Camera

camera = Camera(width=640, height=480, fps=30)
frame = camera.get_frame()
```

### Environment Variables

**Enable/Disable CameraManager**:
```bash
export OMEGA_USE_CAMERA_MANAGER=1  # Enable (default)
export OMEGA_USE_CAMERA_MANAGER=0  # Disable (use legacy mode)
```

**Force Specific Backend**:
```bash
# Force Picamera2
export OMEGA_FORCE_PICAMERA2=1

# Force V4L2
export OMEGA_FORCE_V4L2=1
```

**Debug Mode**:
```bash
# Enable verbose logging
export OMEGA_DEBUG_CAMERA=1
```

### Detection Strategy

CameraManager follows this detection order:

1. **Hardware Detection**:
   - Checks `/proc/device-tree/model` for Pi/Jetson
   - Checks `/proc/cpuinfo` for Pi CPU
   - Checks platform.system() for OS type

2. **Device Scanning**:
   - Lists `/dev/video*` devices
   - Checks driver type using `v4l2-ctl` or sysfs
   - Identifies unicam (ribbon) vs uvcvideo (USB)

3. **Backend Selection**:
   - **Priority 1**: Picamera2 for unicam devices
   - **Priority 2**: Picamera2 on Raspberry Pi
   - **Priority 3**: V4L2 for uvcvideo devices
   - **Priority 4**: V4L2 fallback if devices exist
   - **Priority 5**: Mock camera

### Example Output

When CameraManager runs, you'll see:

```
============================================================
📷 Camera Detection Results
============================================================
Hardware: raspberry_pi
Backend: picamera2
Devices found: 1
Ribbon cameras: 1
  • /dev/video0 (Raspberry Pi Camera)
============================================================
```

Or for USB camera:

```
============================================================
📷 Camera Detection Results
============================================================
Hardware: linux
Backend: v4l2
Devices found: 1
USB cameras: 1
  • /dev/video0 (Logitech HD Pro Webcam C920)
============================================================
```

## Integration with video_server.py

CameraManager is automatically used when creating a Camera instance:

```python
# In video_server.py
from video.camera import Camera

camera = Camera(device="/dev/video0", width=640, height=480)
# CameraManager automatically detects and selects backend
```

## Troubleshooting

### CameraManager Not Working

If CameraManager fails, it automatically falls back to legacy mode. Check logs for:
```
CameraManager failed, falling back to legacy mode: <error>
```

### Force Legacy Mode

To disable CameraManager and use legacy detection:

```bash
export OMEGA_USE_CAMERA_MANAGER=0
```

### Debug Detection

Enable debug mode to see detailed device information:

```bash
export OMEGA_DEBUG_CAMERA=1
python3 video_server.py
```

### Manual Backend Selection

If automatic detection doesn't work, force a specific backend:

```bash
# Force Picamera2
export OMEGA_FORCE_PICAMERA2=1

# Or force V4L2
export OMEGA_FORCE_V4L2=1
```

## Requirements

- **v4l2-utils** (optional, for better device detection):
  ```bash
  sudo apt install v4l-utils
  ```

- **Picamera2** (for ribbon cameras):
  ```bash
  pip3 install picamera2
  ```

- **OpenCV** (for V4L2 backend):
  ```bash
  sudo apt install python3-opencv
  # or
  pip3 install opencv-python
  ```

## Backward Compatibility

CameraManager is fully backward compatible:
- Existing code using `Camera()` works unchanged
- Legacy mode available via `OMEGA_USE_CAMERA_MANAGER=0`
- All Camera API methods work the same

## API Reference

### CameraManager Class

```python
from video.camera_manager import CameraManager

manager = CameraManager(width=640, height=480, fps=30)

# Access detection results
print(manager.hardware_type)  # HardwareType enum
print(manager.backend_type)   # BackendType enum
print(manager.device_info)    # Dict with device info

# Initialize backend
camera = manager.initialize_backend()
frame = camera.get_frame()
```

### BackendType Enum

- `BackendType.PICAMERA2` - Picamera2 backend
- `BackendType.V4L2` - V4L2/OpenCV backend
- `BackendType.MOCK` - Mock camera backend

### HardwareType Enum

- `HardwareType.RASPBERRY_PI` - Raspberry Pi
- `HardwareType.JETSON` - NVIDIA Jetson
- `HardwareType.LINUX` - Generic Linux
- `HardwareType.MACOS` - macOS
- `HardwareType.UNKNOWN` - Unknown platform

---

**Last Updated**: 2024

