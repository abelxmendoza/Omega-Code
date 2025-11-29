# Omega-Code Complete Documentation

This is the consolidated documentation for the Omega-Code robotics control stack. All documentation has been organized into logical sections for easy navigation.

## Table of Contents

1. [Quick Start](#quick-start)
2. [System Overview](#system-overview)
3. [Features & Capabilities](#features--capabilities)
4. [Setup & Installation](#setup--installation)
5. [ROS2 Integration](#ros2-integration)
6. [Hardware Integration](#hardware-integration)
7. [Video & Computer Vision](#video--computer-vision)
8. [Autonomous Behaviors](#autonomous-behaviors)
9. [Network Configuration](#network-configuration)
10. [Troubleshooting](#troubleshooting)
11. [Development Guides](#development-guides)
12. [API Reference](#api-reference)

---

## Quick Start

### 5-Minute Setup

```bash
# 1. Clone repository
git clone <repo-url> Omega-Code
cd Omega-Code

# 2. Configure environment
cp .env.example .env
cp servers/robot_controller_backend/.env.example servers/robot_controller_backend/.env
cp ui/robot-controller-ui/env.example ui/robot-controller-ui/.env.local

# 3. Install dependencies
make backend-install
cd ui/robot-controller-ui && npm install

# 4. Start services
make run-movement  # Terminal 1
make run-video     # Terminal 2
make ui-dev        # Terminal 3
```

### Verify Installation

```bash
# Check endpoints
make check PROFILE=local

# Test hardware (on Pi)
./scripts/test_hardware_on_pi.sh
```

---

## System Overview

### Architecture

Omega-Code is a **high-performance, production-ready** robotics control stack for Raspberry Pi powered rovers with the following components:

- **Backend**: Multi-language (Go + Python) services with WebSocket servers, REST APIs, and ROS2 integration
- **Frontend**: Next.js 14 + React 18 dashboard with real-time controls and monitoring
- **ROS2**: Distributed robotics framework for autonomous behaviors
- **Hardware**: GPIO, motors, sensors, camera, lighting control

### Key Design Principles

1. **Modularity**: Services can run independently or together
2. **Performance**: Optimized for real-time control (<10ms motor response)
3. **Flexibility**: Works on Pi, laptop, or multi-device setups
4. **Reliability**: Watchdogs, retries, graceful degradation

### Performance Targets

| Component | Target | Status |
|-----------|--------|--------|
| GPIO Operations | <1ms | ✅ Achieved |
| Motor Control | <10ms | ✅ Achieved |
| Camera Capture | 30+ FPS | ✅ Achieved |
| Sensor Reading | <5ms | ✅ Achieved |
| WebSocket Latency | 50% reduction | ✅ Achieved |
| Backend Load | 80% reduction | ✅ Achieved |

---

## Features & Capabilities

### Core Robot Control

#### Movement Control
- **Motor Control**: Forward, backward, left, right, stop with speed control
- **Servo Control**: Horizontal/vertical camera servos with angle control
- **Speed Management**: Adjustable speed (0-4095) with increment/decrement
- **Timed Moves**: Execute movements for specific durations
- **Straight-Drive Assist**: Automatic drift correction for straight driving

#### Sensor Systems
- **Ultrasonic Sensor**: HC-SR04 distance measurement (2-400cm range)
- **Line Tracking**: IR line sensor array for line following
- **Battery Monitoring**: Voltage and percentage tracking
- **System Telemetry**: CPU, memory, temperature monitoring

#### Lighting System
- **LED Strip Control**: WS2812/WS2811 RGB LED control
- **Patterns**: Rainbow, fade, blink, chase, music visualizer
- **Color Control**: RGB color selection with brightness control
- **Music Reactive**: Real-time audio visualization (with fallback)

#### Camera & Vision
- **MJPEG Streaming**: Real-time video feed (multi-resolution support)
- **Motion Detection**: Background subtraction for motion tracking
- **Face Recognition**: Known face detection and identification
- **ArUco Detection**: Marker detection and pose estimation
- **Object Tracking**: MOSSE/KCF tracking algorithms
- **Video Recording**: Capture video streams to disk
- **Frame Overlays**: Timestamp, FPS, telemetry overlays

### Autonomous Behaviors

#### ROS2 Actions
- **Navigate to Goal**: Path planning and navigation to target coordinates
- **Follow Line**: Line following with sensor feedback
- **Obstacle Avoidance**: Reactive obstacle avoidance using ultrasonic sensors
- **Docking**: Autonomous docking behavior

#### Autonomy Modes
- **Patrol Mode**: Autonomous patrolling with waypoints
- **Line Following**: Continuous line following mode
- **Exploration**: Autonomous exploration with mapping

### Multi-Platform Support

#### Capability Profiles
- **Light Mode** (MacBook + Pi): Basic features, 640x480 @ 20 FPS
- **Dev Mode** (Lenovo + Pi): Full ROS2, SLAM, 1280x720 @ 25 FPS
- **Omega Mode** (Jetson + Pi): GPU acceleration, ML, 1920x1080 @ 60 FPS

#### Auto-Detection
- Hardware capability detection
- Automatic feature gating
- Profile-based optimization

### Network Features

#### Multi-Profile Support
- **Local**: Development on same machine
- **LAN**: Local network connection
- **Tailscale**: VPN-based remote access

#### Connection Management
- Automatic reconnection with exponential backoff
- WebSocket keep-alive
- Health monitoring and status indicators

### Performance Features

#### Backend Optimizations
- **WebSocket Message Batching**: 50% latency reduction
- **Redis Caching**: 80% reduction in backend load
- **Async Processing**: Non-blocking operations
- **Connection Pooling**: Efficient WebSocket management

#### Frontend Optimizations
- **React Memoization**: 60% fewer re-renders
- **Debounced Input**: 70% fewer WebSocket messages
- **Lazy Loading**: Faster initial load
- **Code Splitting**: Reduced bundle size

---

## Setup & Installation

### Prerequisites

#### Required
- Go 1.22+
- Python 3.11+
- Node.js 18.17+ (20 LTS recommended)
- Raspberry Pi 5 (or Pi 4 with compatibility layer)

#### Optional
- Redis (for advanced caching)
- ROS2 Humble/Rolling (for autonomy)
- Jetson Orin Nano (for AI workloads)

### Backend Setup

```bash
cd servers/robot_controller_backend

# Go modules
go mod download

# Python environment
python -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

# Optional: Redis
sudo apt install redis-server
sudo systemctl start redis-server
```

### Frontend Setup

```bash
cd ui/robot-controller-ui
npm install
```

### ROS2 Setup

See [ROS2 Integration](#ros2-integration) section for detailed ROS2 setup instructions.

### Hardware Setup

#### GPIO Permissions (Pi 5)
```bash
sudo chown root:gpio /dev/gpiochip0
sudo chmod g+rw /dev/gpiochip0
sudo usermod -aG gpio $USER
```

#### Camera Setup
```bash
# Install camera libraries
sudo apt-get install python3-picamera2 python3-libcamera rpicam-apps v4l-utils
```

---

## ROS2 Integration

### Quick Start

```bash
# 1. Setup ROS2 (on Ubuntu/Lenovo)
./scripts/setup_ros2_laptop.sh

# 2. Build workspace
cd ~/omega_ws
colcon build --symlink-install
source install/setup.bash

# 3. Test nodes
ros2 run omega_robot telemetry_publisher
ros2 run omega_robot telemetry_listener
```

### Multi-Device Setup

#### Architecture
- **Laptop**: Development cockpit (colcon, RViz, SSH management)
- **Pi 4B**: Hardware IO controller (motors, GPIO, sensors)
- **Jetson Orin Nano**: AI compute engine (vision, SLAM, ML)

#### Setup Steps

1. **Configure Environment**
```bash
cp .env.ros2.multidevice.example .env.ros2.multidevice
# Edit with device IPs
```

2. **Run Setup on Each Device**
```bash
./scripts/setup_multidevice_ros2.sh
```

3. **Test Communication**
```bash
# Terminal 1 (Laptop)
ros2 run demo_nodes_cpp talker

# Terminal 2 (Pi/Orin)
ros2 run demo_nodes_cpp listener
```

### ROS2 Topics

#### Sensor Topics
- `/omega/sensors/ultrasonic` (Float32) - Distance in cm
- `/omega/sensors/line_tracking` (Int32MultiArray) - Line sensor readings
- `/omega/sensors/battery` (BatteryState) - Battery status

#### Control Topics
- `/cmd_vel` (Twist) - Velocity commands
- `/omega/motors/left` (Float32) - Left motor speed
- `/omega/motors/right` (Float32) - Right motor speed

#### Telemetry Topics
- `/omega/telemetry` (String) - JSON telemetry data
- `/omega/capabilities` (String) - System capabilities

### ROS2 Actions

- `/navigate_to_goal` - Navigate to target position
- `/follow_line` - Follow line on ground
- `/obstacle_avoidance` - Avoid obstacles autonomously

### Docker ROS2 (Pi)

```bash
cd docker/ros2_robot
sudo docker build -t omega_robot:latest -f Dockerfile ../..
sudo docker-compose up -d
```

---

## Hardware Integration

### Motor Control

#### PCA9685 PWM Controller
- 16-channel PWM controller
- Motor speed control (0-4095)
- Servo angle control (0-180°)

#### Motor Commands
```json
{
  "command": "forward",
  "speed": 1200,
  "durationMs": 1000
}
```

### Sensors

#### Ultrasonic (HC-SR04)
- **Pins**: Trigger=GPIO27 (Pin 13), Echo=GPIO22 (Pin 15)
- **Range**: 2-400cm
- **Update Rate**: 1Hz (configurable)

#### Line Tracking
- **Sensors**: IR sensor array
- **Output**: Array of sensor states (0/1)
- **Update Rate**: Configurable

### Camera

#### Supported Backends
- **Picamera2**: CSI ribbon camera (preferred on Pi)
- **V4L2**: USB webcams (/dev/video*)

#### Configuration
```bash
export CAMERA_BACKEND=auto  # or picamera2, v4l2
export CAMERA_WIDTH=640
export CAMERA_HEIGHT=480
export CAMERA_FPS=30
```

### Lighting

#### LED Strip (WS2812/WS2811)
- **Control**: SPI or GPIO
- **Patterns**: Rainbow, fade, blink, chase, music
- **Colors**: RGB hex or integer values

---

## Video & Computer Vision

### Video Streaming

#### MJPEG Server
- **Endpoint**: `/video_feed`
- **Multi-Resolution**: `/video_feed_low`, `/video_feed_medium`, `/video_feed_high`
- **Health Check**: `/health`
- **Snapshot**: `/snapshot?save=1&quality=85`

#### Features
- Hardware-aware optimization (Pi 4B vs Jetson)
- Adaptive JPEG quality based on CPU load
- Frame skipping under heavy load
- Watchdog for camera recovery

### Computer Vision Features

#### Motion Detection
- Background subtraction algorithm
- Motion bounding boxes
- Configurable sensitivity

#### Face Recognition
- Known face database
- Real-time detection and identification
- Configurable recognition threshold

#### ArUco Detection
- Marker detection and ID reading
- Pose estimation (6DOF)
- Multiple dictionary support

#### Object Tracking
- MOSSE/KCF tracking algorithms
- Manual object selection
- Real-time tracking updates

---

## Autonomous Behaviors

### Autonomy Controller

#### Mode System
- Pluggable mode handlers
- Async mode execution
- State management
- Error handling

#### Available Modes
- **Patrol**: Waypoint-based patrolling
- **Line Follow**: Continuous line following
- **Explore**: Autonomous exploration
- **Dock**: Autonomous docking

### ROS2 Actions

#### Navigate to Goal
```python
# Goal message
{
  "x": 1.0,
  "y": 2.0,
  "theta": 0.0
}

# Feedback
{
  "distance_remaining": 0.5,
  "status": "navigating"
}
```

#### Follow Line
```python
# Goal message
{
  "duration": 0  # 0 = infinite
}

# Feedback
{
  "sensor_readings": [1, 0, 1, 0, 1],
  "line_lost": false
}
```

---

## Network Configuration

### Environment Variables

#### Backend (.env)
```bash
# Ports
PORT_MOVEMENT=8081
PORT_ULTRASONIC=8080
VIDEO_PORT=5000

# Network
PI_IP=192.168.1.107
TAILSCALE_IP_PI=100.93.225.61

# CORS
ORIGIN_ALLOW=http://localhost:3000
```

#### Frontend (.env.local)
```bash
# Profile
NEXT_PUBLIC_NETWORK_PROFILE=tailscale

# Gateway
NEXT_PUBLIC_GATEWAY_HOST=100.93.225.61
NEXT_PUBLIC_GATEWAY_PORT=7070

# WebSocket URLs (per profile)
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE=ws://100.93.225.61:8081
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN=ws://192.168.1.107:8081
```

### Network Profiles

#### Local
- Development on same machine
- `localhost` URLs
- No network dependency

#### LAN
- Local network connection
- Direct IP addresses
- Low latency

#### Tailscale
- VPN-based remote access
- Secure connection
- Works across networks

---

## Troubleshooting

### Common Issues

#### Ultrasonic Sensor Timeout
**Symptoms**: "timeout waiting for echo" errors

**Solutions**:
1. Check power: VCC → 5V, GND → GND
2. Verify wiring: Trigger → GPIO27, Echo → GPIO22
3. Check GPIO permissions: `sudo usermod -aG gpio $USER`
4. Run diagnostic: `go run test_ultrasonic_hardware.go`

#### Camera Not Starting
**Symptoms**: No video feed, camera unavailable

**Solutions**:
1. Check camera connection
2. Verify permissions: `sudo usermod -aG video $USER`
3. Test camera: `rpicam-hello` or `v4l2-ctl --list-devices`
4. Check logs: `journalctl -u video-server`

#### ROS2 Topics Not Visible
**Symptoms**: Topics not appearing across devices

**Solutions**:
1. Verify `ROS_DOMAIN_ID` matches (default: 0)
2. Check CycloneDDS config has all device IPs
3. Verify firewall: `sudo ufw allow 7400:7500/udp`
4. Restart daemon: `ros2 daemon stop && ros2 daemon start`

#### WebSocket Connection Failed
**Symptoms**: UI can't connect to backend

**Solutions**:
1. Check backend is running: `make check PROFILE=local`
2. Verify CORS settings: `ORIGIN_ALLOW` environment variable
3. Check firewall/port forwarding
4. Verify WebSocket URL matches backend port

### Diagnostic Tools

#### Hardware Diagnostics
```bash
cd servers/robot_controller_backend
python diagnostics.py --log
```

#### Endpoint Checker
```bash
make check PROFILE=tailscale
```

#### ROS2 Diagnostics
```bash
ros2 topic list
ros2 node list
ros2 topic echo /omega/telemetry
```

---

## Development Guides

### Adding New Features

#### Backend Service
1. Create service module in appropriate directory
2. Add WebSocket/REST endpoint
3. Update gateway API if needed
4. Add tests

#### Frontend Component
1. Create component in `src/components/`
2. Add to appropriate page
3. Connect to backend via WebSocket/REST
4. Add tests

#### ROS2 Node
1. Create node in `ros/src/omega_robot/omega_robot/`
2. Add to `setup.py`
3. Create launch file entry
4. Update documentation

### Testing

#### Backend Tests
```bash
# Go tests
go test ./...

# Python tests
pytest tests/unit
pytest tests/integration
pytest tests/e2e
```

#### Frontend Tests
```bash
npm test              # Jest unit tests
npx cypress run       # E2E tests
npm run lint          # Linting
```

### Performance Optimization

#### Backend
- Use Redis caching for frequently accessed data
- Implement async processing for long operations
- Batch WebSocket messages
- Monitor with performance API

#### Frontend
- Memoize expensive components
- Debounce user input
- Lazy load heavy components
- Use code splitting

---

## API Reference

### WebSocket APIs

#### Movement WebSocket (`ws://host:8081`)
```json
// Commands
{"command": "forward", "speed": 1200}
{"command": "stop"}
{"command": "servo-horizontal", "angle": 10}
{"command": "buzz-for", "durationMs": 500}

// Status
{"command": "status"}
```

#### Ultrasonic WebSocket (`ws://host:8080/ultrasonic`)
```json
// Response
{
  "status": "success",
  "distance_cm": 25,
  "distance_m": 0.25
}
```

### REST APIs

#### FastAPI (`http://host:8000`)

**Lighting**
- `POST /lighting/set` - Set LED color
- `POST /lighting/pattern` - Set pattern
- `GET /lighting/status` - Get status

**Autonomy**
- `POST /autonomy/start` - Start autonomy mode
- `POST /autonomy/stop` - Stop autonomy
- `GET /autonomy/status` - Get status

**ROS2 Bridge**
- `WebSocket /api/ros/bridge` - ROS2-Web bridge

#### Gateway API (`http://host:7070`)

**Proxies**
- `/ws/movement` - Movement WebSocket
- `/ws/ultrasonic` - Ultrasonic WebSocket
- `/video_feed` - Video stream
- `/api/net/*` - Network utilities

### Video API

#### Endpoints
- `GET /video_feed` - MJPEG stream
- `GET /health` - Camera health
- `GET /snapshot` - Single frame
- `GET /metrics` - Performance metrics
- `POST /recording/start` - Start recording
- `POST /recording/stop` - Stop recording

---

## Additional Resources

### Documentation Files
- `FILE_SUMMARY.md` - Complete file reference
- `ENVIRONMENT_VARIABLES.md` - Environment variable reference
- `QUICK_START_INTEGRATION.md` - Integration quick start

### Phase Documentation
- `PHASE2_CAMERA_INTEGRATION.md` - Camera integration details
- `PHASE3_AUTONOMOUS_BEHAVIORS.md` - Autonomy implementation
- `PHASE4_NAVIGATION_SLAM.md` - Navigation and SLAM

### Platform-Specific
- `ROS2_ARCHITECTURE.md` - ROS2 architecture
- `ROS2_MULTIDEVICE_SETUP.md` - Multi-device setup
- `JETSON_ORIN_INTEGRATION.md` - Jetson integration

---

**Last Updated**: 2024  
**Version**: 1.0  
**License**: MIT

