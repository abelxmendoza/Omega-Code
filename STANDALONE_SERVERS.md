# Standalone Server Guide

Each server in the robot controller can run **independently** without the main backend API. This allows you to:

- ✅ Test individual components in isolation
- ✅ Run only the services you need
- ✅ Develop and debug without starting everything
- ✅ Use the UI with just one service running

## Overview

The robot controller consists of multiple independent WebSocket servers:

| Server | Port | Language | File | Purpose |
|--------|------|----------|------|---------|
| **Ultrasonic** | 8080 | Go | `sensors/main_ultrasonic.go` | Distance sensing (HC-SR04) |
| **Movement** | 8081 | Python/Go | `movement/movement_ws_server.py` or `movement/movement.go` | Robot movement control |
| **Lighting** | 8082 | Go | `controllers/lighting/main_lighting.go` | LED strip control |
| **Line Tracker** | 8090 | Python | `sensors/line_tracking_ws_server.py` | Line following sensors |
| **Video** | 5000 | Python | `video/video_server.py` | Camera streaming |
| **Main API** | 8000 | Python | `main_api.py` | REST API & gateway (optional) |

## Running Individual Servers

### 1. Ultrasonic Sensor Server

**Standalone - No dependencies**

```bash
cd servers/robot-controller-backend/sensors
go run main_ultrasonic.go
```

**Environment Variables:**
```bash
PORT_ULTRASONIC=8080                    # WebSocket port (default: 8080)
ULTRA_PATH=/ultrasonic                  # WebSocket path (default: /ultrasonic)
ULTRA_MEASURE_INTERVAL=1s               # Measurement interval (default: 1s)
ULTRA_LOG_EVERY=1s                      # Log frequency (default: 1s)
ULTRA_LOG_DELTA_CM=5                    # Log when change >= 5cm (default: 5)
ORIGIN_ALLOW=http://localhost:3000     # CORS origins (comma-separated)
```

**UI Connection:**
The UI connects directly to `ws://<host>:8080/ultrasonic`. No main backend required.

**Test:**
```bash
# In another terminal
wscat -c ws://localhost:8080/ultrasonic
```

### 2. Movement Server

**Standalone - No dependencies**

**Python version:**
```bash
cd servers/robot-controller-backend/movement
python3 movement_ws_server.py
```

**Go version:**
```bash
cd servers/robot-controller-backend/movement
go run movement.go
```

**Environment Variables:**
```bash
PORT_MOVEMENT=8081                      # WebSocket port (default: 8081)
MOVEMENT_PATH=/                         # WebSocket path (default: /)
ORIGIN_ALLOW=http://localhost:3000     # CORS origins
ROBOT_SIM=0                             # 1 = simulation mode (no hardware)
```

**UI Connection:**
The UI connects directly to `ws://<host>:8081/`. No main backend required.

**Test:**
```bash
# Send movement command
echo '{"type":"move","direction":"forward","speed":0.5}' | wscat -c ws://localhost:8081/
```

### 3. Lighting Server

**Standalone - Requires LED hardware or simulation**

```bash
cd servers/robot-controller-backend/controllers/lighting
go run main_lighting.go
```

**Environment Variables:**
```bash
# Port is hardcoded to 8082 in main_lighting.go
# Modify the code or use a reverse proxy to change port
```

**UI Connection:**
The UI connects directly to `ws://<host>:8082/lighting`. No main backend required.

**Test:**
```bash
# Send lighting command
echo '{"color":"#ff0000","mode":"single","pattern":"static","interval":0,"brightness":1.0}' | wscat -c ws://localhost:8082/lighting
```

### 4. Line Tracker Server

**Standalone - No dependencies**

```bash
cd servers/robot-controller-backend/sensors
python3 line_tracking_ws_server.py
```

**Environment Variables:**
```bash
LINE_TRACKER_HOST=0.0.0.0               # Bind address
LINE_TRACKER_PORT=8090                  # WebSocket port
LINE_TRACKER_PATH=/line-tracker         # WebSocket path
RATE_HZ=10                               # Update rate
FORCE_SIM=0                              # 1 = simulation mode
```

**UI Connection:**
The UI connects directly to `ws://<host>:8090/line-tracker`. No main backend required.

### 5. Video Server

**Standalone - Requires camera or simulation**

```bash
cd servers/robot-controller-backend/video
python3 video_server.py
```

**Environment Variables:**
```bash
VIDEO_PORT=5000                          # HTTP port for MJPEG stream
BIND_HOST=0.0.0.0                        # Bind address
PLACEHOLDER_WHEN_NO_CAMERA=1             # Show placeholder if no camera
```

**UI Connection:**
The UI connects directly to `http://<host>:5000/video_feed`. No main backend required.

## UI Configuration

The UI connects directly to each server. Configure in `.env.local`:

```bash
# Ultrasonic - Direct connection (bypasses gateway)
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE=ws://100.93.225.61:8080/ultrasonic
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN=ws://192.168.6.164:8080/ultrasonic
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL=ws://localhost:8080/ultrasonic

# Movement - Direct connection
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE=ws://100.93.225.61:8081/
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN=ws://192.168.6.164:8081/
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL=ws://localhost:8081/

# Lighting - Direct connection
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE=ws://100.93.225.61:8082/lighting
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN=ws://192.168.6.164:8082/lighting
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL=ws://localhost:8082/lighting

# Line Tracker - Direct connection
NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE=ws://100.93.225.61:8090/line-tracker
NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN=ws://192.168.6.164:8090/line-tracker
NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL=ws://localhost:8090/line-tracker

# Video Stream - Direct connection
NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE=http://100.93.225.61:5000/video_feed
NEXT_PUBLIC_VIDEO_STREAM_URL_LAN=http://192.168.6.164:5000/video_feed
NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL=http://localhost:5000/video_feed
```

## Running Multiple Servers

You can run multiple servers simultaneously:

```bash
# Terminal 1: Ultrasonic
cd servers/robot-controller-backend/sensors
go run main_ultrasonic.go

# Terminal 2: Movement
cd servers/robot-controller-backend/movement
python3 movement_ws_server.py

# Terminal 3: Lighting
cd servers/robot-controller-backend/controllers/lighting
go run main_lighting.go

# Terminal 4: Video
cd servers/robot-controller-backend/video
python3 video_server.py
```

Or use a process manager like `tmux` or `screen`:

```bash
# Create a tmux session
tmux new-session -d -s robot

# Split into panes and run servers
tmux split-window -h
tmux select-pane -t 0
tmux send-keys 'cd sensors && go run main_ultrasonic.go' C-m
tmux select-pane -t 1
tmux send-keys 'cd movement && python3 movement_ws_server.py' C-m
# ... etc
```

## Testing Standalone Servers

### Test Ultrasonic

```bash
# Start server
go run sensors/main_ultrasonic.go

# Connect with wscat
wscat -c ws://localhost:8080/ultrasonic

# You should see:
# {"status":"connected","service":"ultrasonic","message":"...","ts":...}
# {"status":"success","distance_cm":25,"distance_m":0.25,...}
```

### Test Movement

```bash
# Start server
python3 movement/movement_ws_server.py

# Connect and send command
wscat -c ws://localhost:8081/
# Then type: {"type":"move","direction":"forward","speed":0.5}
```

### Test Lighting

```bash
# Start server
go run controllers/lighting/main_lighting.go

# Connect and send command
wscat -c ws://localhost:8082/lighting
# Then type: {"color":"#00ff00","mode":"single","pattern":"static","interval":0,"brightness":1.0}
```

## Main Backend (Optional)

The main backend (`main_api.py`) provides:

- REST API endpoints
- Gateway/proxy functionality
- ROS integration
- Autonomy modes
- Performance monitoring

**You don't need it for basic sensor/movement/lighting control!**

If you want to use the gateway (port 7070) instead of direct connections:

```bash
# Start gateway
cd servers/robot-controller-backend
python3 servers/gateway_api.py

# Then configure UI to use gateway URLs:
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE=ws://omega1-1.hartley-ghost.ts.net:7070/ws/ultrasonic
```

## Troubleshooting

### Port Already in Use

```bash
# Find process using port
lsof -i :8080

# Kill process
kill <PID>
```

### CORS Errors

Make sure `ORIGIN_ALLOW` includes your UI origin:

```bash
export ORIGIN_ALLOW=http://localhost:3000,http://192.168.1.100:3000
```

### UI Can't Connect

1. Check server is running: `curl http://localhost:8080/` (should fail, but confirms server is up)
2. Check WebSocket path matches (e.g., `/ultrasonic` vs `/`)
3. Check firewall allows the port
4. Verify UI `.env.local` has correct URLs

### Hardware Not Working

- Use simulation mode: `ROBOT_SIM=1` or `FORCE_SIM=1`
- Check GPIO permissions: `sudo usermod -a -G gpio $USER`
- Verify hardware wiring matches code expectations

## Summary

✅ **Each server is independent** - Run only what you need  
✅ **UI connects directly** - No main backend required  
✅ **Easy testing** - Test components in isolation  
✅ **Flexible deployment** - Run servers on different machines if needed  

The ultrasonic sensor, movement control, lighting, and other services all work standalone with the UI!

