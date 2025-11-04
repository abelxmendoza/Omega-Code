# Environment Variables

This document describes the environment variables needed to run the robot controller application.

## Frontend (Next.js UI)

### Required Variables
Create a `.env.local` file in the `ui/robot-controller-ui/` directory:

```bash
# Active network profile for all endpoints (used by resolvers & proxies)
# Options: lan | tailscale | local
NEXT_PUBLIC_NETWORK_PROFILE=tailscale

# Gateway host (unified FastAPI gateway on port 7070)
# Replace with your Pi's Tailscale IP
NEXT_PUBLIC_GATEWAY_HOST=100.93.225.61
NEXT_PUBLIC_GATEWAY_PORT=7070

# Per-profile robot host hints used by some resolvers
NEXT_PUBLIC_ROBOT_HOST_TAILSCALE=100.93.225.61
NEXT_PUBLIC_ROBOT_HOST_LAN=192.168.6.164
NEXT_PUBLIC_ROBOT_HOST_LOCAL=100.93.225.61

# Video stream (MJPEG) upstreams used by /api/video-proxy and /api/video-health
NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE=http://100.93.225.61:5000/video_feed
NEXT_PUBLIC_VIDEO_STREAM_URL_LAN=http://192.168.6.164:5000/video_feed
NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL=http://100.93.225.61:5000/video_feed

# WebSocket URLs (gateway proxies to subsystems under /ws/*)
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE=ws://100.93.225.61:5000/ws/movement
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN=ws://192.168.6.164:5000/ws/movement
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL=ws://100.93.225.61:5000/ws/movement

NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE=ws://100.93.225.61:5000/ws/ultrasonic
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN=ws://192.168.6.164:5000/ws/ultrasonic
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL=ws://100.93.225.61:5000/ws/ultrasonic

NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE=ws://100.93.225.61:5000/ws/line
NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN=ws://192.168.6.164:5000/ws/line
NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL=ws://100.93.225.61:5000/ws/line

NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE=ws://100.93.225.61:5000/ws/lighting
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN=ws://192.168.6.164:5000/ws/lighting
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL=ws://100.93.225.61:5000/ws/lighting
```

### Optional Variables
```bash
# WebSocket behavior & debug
NEXT_PUBLIC_WS_FORCE_INSECURE=0  # Keep ws:// even on HTTPS pages
NEXT_PUBLIC_WS_DEBUG=1  # Browser console debug for URL selection & socket lifecycle
```

## Backend (Python Services)

### Required Variables
Create a `.env` file in the `servers/robot-controller-backend/` directory:

```bash
# Server configuration
HOST=0.0.0.0
PORT=8000
DEBUG=true

# Hardware configuration (Raspberry Pi specific)
GPIO_PIN_LED=18
GPIO_PIN_MOTOR_LEFT=20
GPIO_PIN_MOTOR_RIGHT=21
```

### Optional Variables
```bash
# Logging
LOG_LEVEL=INFO
LOG_FILE=robot.log

# Camera settings
CAMERA_WIDTH=640
CAMERA_HEIGHT=480
CAMERA_FPS=30
```

## Quick Setup

1. **Frontend**: Copy the example variables above to `ui/robot-controller-ui/.env.local`
2. **Backend**: Copy the example variables above to `servers/robot-controller-backend/.env`
3. **Development**: Set `NEXT_PUBLIC_MOCK_WS=true` to use mock connections

## ROS 2 Docker Environment

### Required Variables

For ROS 2 Humble Docker containers (`docker/ros2_robot/`):

```bash
# ROS 2 domain ID for topic isolation (default: 0)
ROS_DOMAIN_ID=0

# ROS 2 middleware implementation (CycloneDDS for inter-container communication)
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# CycloneDDS configuration file path
CYCLONEDDS_URI=file:///root/omega_ws/config/cyclonedds.xml
```

### Configuration

The CycloneDDS configuration file (`docker/ros2_robot/config/cyclonedds.xml`) should be updated with your network IPs:

```xml
<Peers>
  <Peer address="192.168.1.107"/> <!-- Raspberry Pi -->
  <Peer address="192.168.1.202"/> <!-- MacBook -->
</Peers>
```

### Usage

Set these variables when running Docker containers:

```bash
sudo docker run -it --rm --network host \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///root/omega_ws/config/cyclonedds.xml \
  omega_robot:latest \
  ros2 run omega_robot telemetry_publisher
```

Or use `docker-compose.yml` which sets these automatically.

## Troubleshooting

- **"WS URL missing"**: Make sure all `NEXT_PUBLIC_BACKEND_WS_URL_*` variables are set
- **Connection refused**: Ensure backend services are running on the specified ports
- **Hardware errors**: Check GPIO pin assignments match your robot's wiring
- **ROS 2 topics not visible**: Verify `ROS_DOMAIN_ID` matches across containers and CycloneDDS configuration is correct
