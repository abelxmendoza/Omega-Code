# ROS2 and OpenCV Integration Guide

This document describes how ROS2 and OpenCV are integrated throughout the Omega-Code robot controller application.

## Overview

The application supports **two modes** for ROS2 integration:

1. **Docker Mode** (default): ROS2 nodes run in Docker containers
2. **Native Mode**: Direct ROS2 integration using rclpy Python bindings

OpenCV is used throughout for:
- Camera capture and video streaming
- Computer vision (object detection, face recognition, ArUco markers)
- Image processing and motion detection
- Frame encoding for MJPEG streams

## ROS2 Integration

### Docker Mode (Default)

ROS2 nodes run in Docker containers, managed via the API:

```bash
# Start ROS2 containers
curl -X POST http://localhost:8000/api/ros/control \
  -H "Content-Type: application/json" \
  -d '{"action": "start"}'

# Check status
curl http://localhost:8000/api/ros/status

# List topics
curl http://localhost:8000/api/ros/topics
```

**Configuration:**
- `ROS_DOCKER_COMPOSE_PATH`: Path to docker-compose.yml
- `ROS_WORKSPACE_PATH`: ROS2 workspace path
- Auto-detects project root (works from laptop or Pi)

### Native Mode

Direct ROS2 integration using rclpy:

```bash
# Enable native mode
export ROS_NATIVE_MODE=true

# Ensure ROS2 is sourced
source /opt/ros/rolling/setup.bash
source ~/omega_ws/install/setup.bash

# Start backend
cd servers/robot-controller-backend
python main_api.py
```

**Features:**
- Direct topic subscription/publishing
- No Docker overhead
- Real-time message bridging
- WebSocket integration for UI

**API Endpoints:**
- `/api/ros/status` - Bridge status and topics
- `/api/ros/topics` - List available topics
- `/api/ros/telemetry` - WebSocket for telemetry stream

### Native Bridge API

The `ros_native_bridge.py` module provides:

```python
from api.ros_native_bridge import get_bridge, init_ros2_bridge

# Initialize bridge
init_ros2_bridge()

# Get bridge instance
bridge = get_bridge()

# Subscribe to topic
bridge.subscribe_topic("/omega/telemetry", String)

# Publish to topic
bridge.publish_string("/omega/commands", "move_forward")

# Get latest message
msg = bridge.get_latest_message("/omega/telemetry")
```

## OpenCV Integration

### Camera Backend

OpenCV is used as a fallback camera backend:

```python
from video.camera import Camera

# Auto-detects backend (Picamera2 or OpenCV/V4L2)
camera = Camera(width=640, height=480, fps=30)
frame = camera.get_frame()  # Returns BGR numpy array
```

**Backend Priority:**
1. Picamera2 (for Raspberry Pi CSI cameras)
2. OpenCV/V4L2 (for USB webcams)
3. Mock camera (for development)

### Computer Vision Features

All CV features use OpenCV:

- **Motion Detection** (`video/motion_detection.py`)
- **Object Tracking** (`video/object_tracking.py`)
- **Face Recognition** (`video/face_recognition.py`)
- **ArUco Detection** (`video/aruco_detection.py`)

### Video Server

The MJPEG video server uses OpenCV for:

- Frame capture
- JPEG encoding
- Overlay rendering (motion, tracking, etc.)

```python
# In video_server.py
import cv2

# Encode frame to JPEG
ok, buf = cv2.imencode(".jpg", frame)
jpg = buf.tobytes()
```

## Configuration

### Environment Variables

**ROS2:**
```bash
ROS_DOMAIN_ID=0                    # ROS2 domain ID
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Middleware
CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
ROS_NATIVE_MODE=true               # Use native ROS2 (not Docker)
ROS_DOCKER_COMPOSE_PATH=/path/to/docker-compose.yml
ROS_WORKSPACE_PATH=~/omega_ws
```

**OpenCV/Camera:**
```bash
CAMERA_BACKEND=auto                # auto | picamera2 | v4l2
CAMERA_DEVICE=/dev/video0          # For V4L2
CAMERA_WIDTH=640
CAMERA_HEIGHT=480
CAMERA_FPS=30
```

### Project Paths

The application auto-detects paths:

- **Project Root**: Auto-detected from script location
- **Docker Compose**: `{project_root}/docker/ros2_robot/docker-compose.yml`
- **Workspace**: `~/omega_ws` (configurable)

Works from:
- ✅ Laptop (Ubuntu)
- ✅ Raspberry Pi
- ✅ Jetson Orin Nano

## Installation

### ROS2 (Native Mode)

```bash
# Install ROS2 Rolling (if not already installed)
sudo apt install ros-rolling-desktop ros-rolling-rclpy

# Source ROS2
source /opt/ros/rolling/setup.bash
```

### OpenCV

```bash
# System-wide (if needed)
sudo apt install python3-opencv

# In virtual environment (recommended)
cd servers/robot-controller-backend
python3 -m venv venv
source venv/bin/activate
pip install opencv-python opencv-contrib-python
```

### Dependencies

```bash
cd servers/robot-controller-backend
pip install -r requirements.txt
```

## Verification

Run the verification script:

```bash
./scripts/verify_ros2_opencv_setup.sh
```

This checks:
- ✅ ROS2 installation
- ✅ rclpy availability
- ✅ OpenCV installation
- ✅ Workspace setup
- ✅ Project configuration

## Usage Examples

### Start Robot Controller (Laptop)

```bash
# 1. Setup environment
source ~/.ros2_rolling_setup.bash
source ~/omega_ws/install/setup.bash

# 2. Enable native ROS2 (optional)
export ROS_NATIVE_MODE=true

# 3. Start backend
cd servers/robot-controller-backend
source venv/bin/activate
python main_api.py

# 4. In another terminal, start UI
cd ui/robot-controller-ui
npm run dev
```

### Publish ROS2 Commands

```python
from api.ros_native_bridge import get_bridge

bridge = get_bridge()
bridge.publish_twist("/cmd_vel", linear_x=0.5, angular_z=0.0)
```

### Subscribe to ROS2 Topics

```python
from api.ros_native_bridge import get_bridge
from std_msgs.msg import String

bridge = get_bridge()

def on_telemetry(msg):
    print(f"Received: {msg.data}")

bridge.subscribe_topic("/omega/telemetry", String, on_telemetry)
```

## Troubleshooting

### ROS2 Not Found

```bash
# Check ROS2 installation
ls /opt/ros/rolling/setup.bash

# Source ROS2
source /opt/ros/rolling/setup.bash

# Verify
ros2 --help
```

### rclpy Import Error

```bash
# Install rclpy
sudo apt install ros-rolling-rclpy

# Verify Python version matches ROS2
python3 --version  # Should be 3.12 for Rolling
```

### OpenCV Not Found

```bash
# Install in venv
pip install opencv-python opencv-contrib-python

# Verify
python3 -c "import cv2; print(cv2.__version__)"
```

### Path Issues

The application auto-detects paths, but you can override:

```bash
export ROS_DOCKER_COMPOSE_PATH=/custom/path/docker-compose.yml
export ROS_WORKSPACE_PATH=/custom/workspace
```

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│         Robot Controller App (Multi-Device)              │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────┐              ┌──────────────┐         │
│  │   Laptop     │              │  Raspberry   │         │
│  │  (Ubuntu)    │              │     Pi 4B    │         │
│  │              │              │              │         │
│  │  ┌────────┐  │              │  ┌────────┐  │         │
│  │  │FastAPI │  │              │  │FastAPI │  │         │
│  │  │Backend │  │              │  │Backend │  │         │
│  │  └───┬────┘  │              │  └───┬────┘  │         │
│  │      │       │              │      │       │         │
│  │  ┌───▼───┐   │              │  ┌───▼───┐   │         │
│  │  │OpenCV │   │              │  │OpenCV │   │         │
│  │  │Camera │   │              │  │Camera │   │         │
│  │  └───────┘   │              │  └───────┘   │         │
│  │      │       │              │      │       │         │
│  │  ┌───▼───┐   │              │  ┌───▼───┐   │         │
│  │  │Native │   │◄───DDS──────►│  │Docker │   │         │
│  │  │ROS2   │   │  Network     │  │ROS2   │   │         │
│  │  │Rolling│   │              │  │Humble │   │         │
│  │  └───────┘   │              │  └───────┘   │         │
│  └──────────────┘              └──────────────┘         │
│                                                         │
│              ROS2 DDS (CycloneDDS)                      │
│              Domain ID: 0                               │
└─────────────────────────────────────────────────────────┘
```

**Note**: 
- **Laptop**: Uses native ROS2 Rolling (rclpy)
- **Pi**: Uses Docker containers with ROS2 Humble (Raspberry Pi OS)
- **Communication**: Via ROS2 DDS (CycloneDDS) over network

## Best Practices

1. **Use Native Mode** for development (faster iteration)
2. **Use Docker Mode** for production (isolation)
3. **Always source ROS2** before running backend
4. **Use virtual environment** for Python dependencies
5. **Verify setup** with verification script before deployment

## Support

For issues:
1. Run verification script: `./scripts/verify_ros2_opencv_setup.sh`
2. Check logs: `servers/robot-controller-backend/logs/`
3. Verify environment variables
4. Check ROS2 daemon: `ros2 daemon status`

---

**Last Updated**: 2024  
**ROS2 Version**: Rolling  
**OpenCV Version**: 4.8.0+

