# Integration Verification Guide

This guide helps you verify that your frontend controller works correctly with your backend and hardware on omega1.

## Quick Start

### 1. SSH into omega1

```bash
# Via Tailscale (recommended for remote access)
./scripts/ssh_omega1.sh tailscale

# Via LAN
./scripts/ssh_omega1.sh lan

# Local (if running on same machine)
./scripts/ssh_omega1.sh local
```

### 2. Verify Integration

From your MacBook, run the verification script:

```bash
# Test via Tailscale
./scripts/verify_integration.sh tailscale

# Test via LAN
./scripts/verify_integration.sh lan

# Test locally
./scripts/verify_integration.sh local
```

## Architecture Overview

```
┌─────────────────┐
│   MacBook       │
│   (Frontend)    │
│   Port 3000     │
└────────┬────────┘
         │ HTTP/WebSocket
         │ (via Tailscale/LAN)
         │
┌────────▼─────────────────────────┐
│   omega1 (Raspberry Pi)          │
│                                  │
│  ┌──────────────────────────┐   │
│  │  Gateway API (Port 7070) │   │
│  │  - /api/net/summary      │   │
│  │  - /api/performance/*    │   │
│  │  - /ws/movement          │   │
│  │  - /ws/ultrasonic        │   │
│  │  - /ws/line             │   │
│  │  - /ws/lighting         │   │
│  └──────────┬───────────────┘   │
│             │                    │
│  ┌──────────▼───────────────┐   │
│  │  Backend Services        │   │
│  │  - Movement WS (8081)     │   │
│  │  - Ultrasonic WS (8080)  │   │
│  │  - Video Server (5000)    │   │
│  └──────────┬───────────────┘   │
│             │                    │
│  ┌──────────▼───────────────┐   │
│  │  Hardware                │   │
│  │  - GPIO (Motors)         │   │
│  │  - Sensors              │   │
│  │  - Camera               │   │
│  └──────────────────────────┘   │
└──────────────────────────────────┘
```

## Environment Configuration

### Frontend (MacBook)

Create/edit `ui/robot-controller-ui/.env.local`:

```bash
# Network Profile
NEXT_PUBLIC_NETWORK_PROFILE=tailscale

# Gateway Configuration
NEXT_PUBLIC_GATEWAY_HOST=100.93.225.61  # omega1 Tailscale IP
NEXT_PUBLIC_GATEWAY_PORT=7070

# Per-profile robot hosts
NEXT_PUBLIC_ROBOT_HOST_TAILSCALE=100.93.225.61
NEXT_PUBLIC_ROBOT_HOST_LAN=192.168.6.164
NEXT_PUBLIC_ROBOT_HOST_LOCAL=localhost

# WebSocket URLs (gateway proxies these)
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE=ws://100.93.225.61:7070/ws/movement
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE=ws://100.93.225.61:7070/ws/ultrasonic
NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE=ws://100.93.225.61:7070/ws/line
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE=ws://100.93.225.61:7070/ws/lighting

# Video Stream
NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE=http://100.93.225.61:5000/video_feed
```

### Backend (omega1)

Create/edit `servers/robot_controller_backend/.env`:

```bash
# Gateway Configuration
PORT=7070

# Downstream WebSocket Services
DS_MOVE_WS=ws://127.0.0.1:8081
DS_ULTRA_WS=ws://127.0.0.1:8080/ultrasonic
DS_LINE_WS=ws://127.0.0.1:8090/line-tracker
DS_LIGHT_WS=ws://127.0.0.1:8082/lighting

# Video Server
VIDEO_UPSTREAM=http://127.0.0.1:5000/video_feed

# Movement Server
PORT_MOVEMENT=8081
MOVEMENT_PATH=/
ROBOT_SIM=0  # Set to 1 for simulation mode (no hardware)

# Ultrasonic Sensor
PORT_ULTRASONIC=8080
ULTRA_PATH=/ultrasonic

# Video Server
VIDEO_PORT=5000
CAMERA_BACKEND=libcamera  # or picamera2
```

## Starting Services on omega1

### Option 1: Individual Services

```bash
# SSH into omega1
./scripts/ssh_omega1.sh tailscale

# Start Gateway API
cd ~/Omega-Code/servers/robot_controller_backend
uvicorn servers.gateway_api:app --host 0.0.0.0 --port 7070

# In another terminal, start Movement WebSocket
cd ~/Omega-Code/servers/robot_controller_backend/movement
python3 movement_ws_server.py

# In another terminal, start Video Server
cd ~/Omega-Code/servers/robot_controller_backend
python3 video/video_server.py
```

### Option 2: Using Makefile

```bash
# On omega1
cd ~/Omega-Code
make run-movement
make run-video
# Gateway should be started separately or via systemd
```

## Testing Frontend-Backend Connection

### 1. Check Gateway Health

```bash
curl http://100.93.225.61:7070/health
# Should return: ok
```

### 2. Test WebSocket Connection

```bash
# Install wscat if needed
npm install -g wscat

# Test movement WebSocket
wscat -c ws://100.93.225.61:7070/ws/movement

# Send a test command
{"command": "move-up", "speed": 50}
```

### 3. Test from Frontend

1. Start frontend on MacBook:
   ```bash
   cd ui/robot-controller-ui
   npm run dev
   ```

2. Open browser: `http://localhost:3000`

3. Check Network Wizard in header:
   - Should show active profile (tailscale/lan/local)
   - Should show connection status
   - Should display omega1 IP address

4. Test robot controls:
   - Movement controls should send commands via WebSocket
   - Video feed should display camera stream
   - Sensor data should update in real-time

## Troubleshooting

### Frontend can't connect to backend

1. **Check network profile:**
   ```bash
   # On MacBook
   ./scripts/verify_integration.sh tailscale
   ```

2. **Verify gateway is running:**
   ```bash
   # SSH into omega1
   ./scripts/ssh_omega1.sh tailscale
   curl http://localhost:7070/health
   ```

3. **Check firewall:**
   ```bash
   # On omega1
   sudo ufw status
   # Allow ports if needed
   sudo ufw allow 7070/tcp
   sudo ufw allow 5000/tcp
   ```

### WebSocket connection fails

1. **Check gateway logs:**
   ```bash
   # On omega1
   tail -f ~/Omega-Code/servers/robot_controller_backend/logs/gateway.log
   ```

2. **Verify downstream services:**
   ```bash
   # Check if movement server is running
   ps aux | grep movement_ws_server
   
   # Check if port is listening
   netstat -tlnp | grep 8081
   ```

### Hardware not responding

1. **Check GPIO permissions:**
   ```bash
   # On omega1
   groups  # Should include gpio
   ls -l /dev/gpiomem
   ```

2. **Test hardware directly:**
   ```bash
   # Run diagnostics
   cd ~/Omega-Code/servers/robot_controller_backend
   python3 diagnostics.py
   ```

3. **Check simulation mode:**
   ```bash
   # In backend .env
   ROBOT_SIM=0  # Should be 0 for real hardware
   ```

## Development Workflow

### 1. Develop on MacBook

```bash
# Make changes to frontend
cd ui/robot-controller-ui
npm run dev

# Make changes to backend
cd servers/robot_controller_backend
# Edit files, test locally with mock hardware
```

### 2. Push to Repository

```bash
git add .
git commit -m "Your changes"
git push origin master
```

### 3. Pull on omega1

```bash
# SSH into omega1
./scripts/ssh_omega1.sh tailscale

# Pull latest changes
cd ~/Omega-Code
git pull origin master

# Restart services if needed
# (Use systemd, screen, or tmux to manage services)
```

## Quick Reference

### SSH Commands

```bash
# Connect via Tailscale
./scripts/ssh_omega1.sh tailscale

# Connect via LAN
./scripts/ssh_omega1.sh lan

# Execute command remotely
./scripts/ssh_omega1.sh tailscale "cd ~/Omega-Code && git pull"
```

### Verification Commands

```bash
# Full integration test
./scripts/verify_integration.sh tailscale

# Check endpoints only
./scripts/check_endpoints.sh --profile tailscale
```

### Service Ports

| Service | Port | Protocol |
|---------|------|----------|
| Gateway API | 7070 | HTTP/WebSocket |
| Movement WS | 8081 | WebSocket |
| Ultrasonic WS | 8080 | WebSocket |
| Line Tracker WS | 8090 | WebSocket |
| Lighting WS | 8082 | WebSocket |
| Video Server | 5000 | HTTP (MJPEG) |

## Next Steps

1. ✅ Verify SSH access to omega1
2. ✅ Run integration verification script
3. ✅ Test frontend controls
4. ✅ Verify hardware responses
5. ✅ Set up automated deployment (optional)

For more details, see:
- [ENVIRONMENT_VARIABLES.md](ENVIRONMENT_VARIABLES.md)
- [README.md](README.md)
- [ROS2_ARCHITECTURE.md](ROS2_ARCHITECTURE.md)

