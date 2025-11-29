# Omega Hybrid Vision System

## Overview

The Omega Hybrid Vision System implements a **distributed compute architecture** combining:
- **Raspberry Pi 4B** (onboard sensor hub) - Current hardware
- **Jetson Orin Nano** (offboard AI brain) - Future hardware

This system enables the Pi to handle real-time sensor data and low-level vision tasks, while offloading deep learning and high-level perception to the more powerful Orin.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    RASPBERRY PI 4B                          │
│              (Onboard Sensor Hub)                            │
│                                                              │
│  • Picamera2 capture (640x480@30fps)                        │
│  • Motion detection                                          │
│  • KCF tracking                                             │
│  • ArUco detection                                          │
│  • Light face detection (Haar)                              │
│  • Frame overlays                                            │
│  • Video recording (XVID)                                    │
│  • ROS2 micro nodes                                         │
│                                                              │
│  Publishes:                                                  │
│    - /omega/camera/compressed                               │
│    - /omega/events/aruco                                    │
│    - /omega/events/tracking                                  │
│    - /omega/events/motion                                    │
│    - /omega/telemetry                                       │
└─────────────────────────────────────────────────────────────┘
                            │
                            │ ROS2 (FastDDS)
                            │
┌─────────────────────────────────────────────────────────────┐
│              JETSON ORIN NANO                               │
│              (Offboard AI Brain)                            │
│                                                              │
│  • YOLOv8 detection                                         │
│  • DeepFace recognition                                     │
│  • ByteTrack multi-tracking                                 │
│  • Object classification                                    │
│  • Navigation planning                                      │
│  • TensorRT acceleration                                     │
│                                                              │
│  Subscribes:                                                 │
│    - /omega/camera/compressed                               │
│    - /omega/events/*                                        │
│    - /omega/telemetry                                       │
│                                                              │
│  Publishes:                                                  │
│    - /omega/brain/detections                                │
│    - /omega/brain/tracking                                  │
│    - /omega/brain/navigation                                │
│    - /omega/brain/commands                                  │
└─────────────────────────────────────────────────────────────┘
```

## System Modes

### Mode 0: Pi-Only (Current)
- **Activation**: Default mode when Orin is not available
- **Pi Role**: Full sensor hub + light CV
- **Capabilities**: All Pi capabilities enabled
- **Limitations**: No deep learning, limited to 640x480

### Mode 1: Pi+Orin Hybrid (Future)
- **Activation**: When Orin is detected (NVMe installed)
- **Pi Role**: Sensor hub + low-level vision
- **Orin Role**: AI brain + high-level perception
- **Capabilities**: Full hybrid system with deep learning

## Files

### Core Files

1. **`hybrid_messages.py`**
   - Custom message definitions for Pi↔Orin communication
   - ArUco markers, tracking bboxes, motion events, detections, navigation commands

2. **`hybrid_system.py`**
   - System mode detection (Pi-only vs Pi+Orin)
   - Thermal and CPU monitoring with throttling
   - Capability management

3. **`pi_sensor_hub.py`**
   - ROS2 node for Pi sensor hub
   - Publishes compressed frames, events, telemetry
   - Subscribes to Orin commands

4. **`orin_ai_brain.py`** (in `ros/src/omega_robot/omega_robot/`)
   - ROS2 node for Orin AI brain
   - Subscribes to Pi data
   - Processes with YOLOv8, DeepFace, ByteTrack
   - Publishes detections and commands

### Integration

- **`video_server.py`**
   - Integrated hybrid system support
   - Publishes events to Orin when in hybrid mode
   - Thermal/CPU throttling

## Usage

### Pi-Only Mode (Current)

```bash
# On Raspberry Pi
cd servers/robot_controller_backend
python video_server.py

# Or via ROS2 launch
ros2 launch omega_robot pi_only.launch.py
```

The system automatically detects Pi-only mode and operates accordingly.

### Pi+Orin Hybrid Mode (Future)

**On Raspberry Pi:**
```bash
# Start video server (automatically detects Orin)
cd servers/robot_controller_backend
export ENABLE_HYBRID_SYSTEM=1
python video_server.py
```

**On Jetson Orin Nano:**
```bash
# Start Orin AI brain node
ros2 run omega_robot orin_ai_brain
```

**Or launch both:**
```bash
# On Pi
ros2 launch omega_robot pi_orin_hybrid.launch.py
```

## Environment Variables

### Hybrid System Control

- `ENABLE_HYBRID_SYSTEM` - Enable hybrid system (default: 1)
- `OMEGA_ORIN_AVAILABLE` - Force Orin availability (default: auto-detect)

### Pi Configuration

- `CAMERA_BACKEND` - Camera backend (picamera2/v4l2)
- `CAMERA_WIDTH` - Frame width (default: 640)
- `CAMERA_HEIGHT` - Frame height (default: 480)
- `CAMERA_FPS` - Frame rate (default: 30)

### ROS2 Configuration

- `ROS_DOMAIN_ID` - ROS2 domain ID (default: 0)
- `PI_IP` - Raspberry Pi IP address
- `ORIN_IP` - Jetson Orin Nano IP address

## Thermal & CPU Monitoring

The system includes automatic throttling:

- **Thermal Throttling**: Activates when CPU temp > 70°C
- **CPU Load Throttling**: Activates when CPU load > 75%
- **Priority Order**: motion → tracking → aruco → face_detection

When throttling is active, modules are disabled in priority order to reduce load.

## ROS2 Topics

### Pi → Orin

- `/omega/camera/compressed` - Compressed JPEG frames
- `/omega/events/aruco` - ArUco marker detections
- `/omega/events/tracking` - Tracking bounding boxes
- `/omega/events/motion` - Motion detection events
- `/omega/telemetry` - System telemetry

### Orin → Pi

- `/omega/brain/detections` - YOLOv8 detection results
- `/omega/brain/tracking` - ByteTrack tracking results
- `/omega/brain/navigation` - Navigation commands
- `/omega/brain/commands` - Control commands

## Upgrade Path

When Orin arrives:

1. Install NVMe SSD on Orin
2. Flash Jetson OS
3. Install ROS2 Humble
4. Enable GPU + TensorRT
5. Add YOLOv8 server
6. Subscribe to Pi compressed images
7. Send detections back to Pi
8. Connect navigation loop

The system automatically detects Orin availability and switches to hybrid mode.

## Troubleshooting

### Orin Not Detected

- Check NVMe installation: `ls /dev/nvme0n1`
- Check Jetson hardware: `cat /proc/device-tree/model`
- Force enable: `export OMEGA_ORIN_AVAILABLE=1`

### ROS2 Communication Issues

- Check domain ID: `export ROS_DOMAIN_ID=0`
- Verify network connectivity: `ping <orin_ip>`
- Check ROS2 topics: `ros2 topic list`

### Performance Issues

- Check thermal throttling: Monitor CPU temp
- Check CPU load: Monitor CPU usage
- Reduce resolution: Set `CAMERA_WIDTH=320 CAMERA_HEIGHT=240`

## Child-Friendly Explanation

- **Pi**: The robot's little eyes and ears. It sees and hears things up close.
- **Orin**: The robot's big smart brain that stays in the room and thinks very fast.
- **Teamwork**: The little robot sends pictures to the big brain, the big brain figures things out, and tells the robot what to do next.

