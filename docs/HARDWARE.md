# Hardware Guide — Omega Robot

## Standalone Servers Overview

Each server runs independently — no main backend required. The UI connects directly to each service.

| Server | Port | Language | Entry Point | Purpose |
|--------|------|----------|-------------|---------|
| Ultrasonic | 8080 | Go | `sensors/main_ultrasonic.go` | HC-SR04 distance sensor |
| Movement | 8081 | Python/Go | `movement/movement_ws_server.py` | Robot movement control |
| Lighting | 8082 | Go | `controllers/lighting/main_lighting.go` | LED strip control |
| Line Tracker | 8090 | Python | `sensors/line_tracking_ws_server.py` | Line following sensors |
| Video | 5000 | Python | `video/video_server.py` | MJPEG camera stream |
| Main API | 8000 | Python | `main_api.py` | REST API gateway (optional) |

All commands run from `servers/robot_controller_backend/`.

---

## Ultrasonic Sensor (HC-SR04)

**Server:** `sensors/main_ultrasonic.go`
**Protocol:** WebSocket at `ws://<host>:8080/ultrasonic`
**GPIO:** Trigger = GPIO27 (Pin 13), Echo = GPIO22 (Pin 15)

### Run

```bash
cd servers/robot_controller_backend/sensors
go run main_ultrasonic.go
```

### Environment variables

```bash
PORT_ULTRASONIC=8080          # WebSocket port (default: 8080)
ULTRA_PATH=/ultrasonic        # WebSocket path
ORIGIN_ALLOW=http://localhost:3000,http://192.168.1.107:3000
ULTRA_MEASURE_INTERVAL=1s     # Measurement interval
ULTRA_LOG_EVERY=5s            # Log frequency
ULTRA_LOG_DELTA_CM=10         # Log when change >= N cm
```

### Data format

**Server sends:**
```json
{ "status": "success", "distance_cm": 25, "distance_m": 0.25, "distance_inch": 9.84, "distance_feet": 0.82 }
```

On timeout:
```json
{ "status": "error", "error": "timeout" }
```

On connect:
```json
{ "status": "connected", "service": "ultrasonic", "message": "...", "ts": 1234567890 }
```

**UI components that use this:**
- `SensorDashboard.tsx` — shows distance in all units
- `UltrasonicSensorStatus.tsx` — standalone status component
- `Header.tsx` / `ServiceStatusBar.tsx` — service health indicator

### Troubleshooting

**"timeout waiting for echo"**

Run the hardware diagnostic:
```bash
cd servers/robot_controller_backend/sensors
go run test_ultrasonic_hardware.go
```

**Power issues** — Verify 5V at VCC pin (HC-SR04 needs ~15mA)

**Wiring checklist:**
```
HC-SR04  →  Raspberry Pi
VCC      →  5V (Pin 2 or 4)
GND      →  GND (Pin 6, 9, 14...)
Trigger  →  GPIO27 (Pin 13)
Echo     →  GPIO22 (Pin 15)
```

**GPIO permissions:**
```bash
sudo usermod -a -G gpio $USER
newgrp gpio
```

**Wrong pins** — edit `main_ultrasonic.go` around line 321-322:
```go
trigger := rpi.P1_13 // GPIO27
echo := rpi.P1_15    // GPIO22
```

**Test without hardware:**
```bash
cd sensors
FORCE_SIM=1 python3 ultrasonic_ws_server.py
```

**Step-by-step GPIO debugging:**
```bash
# Check GPIO state
gpio readall

# Manually test trigger pin
echo 27 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio27/direction
echo 1 > /sys/class/gpio/gpio27/value
sleep 0.00002
echo 0 > /sys/class/gpio/gpio27/value

# Monitor echo pin
echo 22 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio22/direction
watch -n 0.1 cat /sys/class/gpio/gpio22/value
```

**Expected behavior:**
- Echo goes HIGH within ~100µs of trigger
- Readings consistent within ±2cm
- Range: 2cm to 400cm

---

## Movement Server

**Server:** `movement/movement_ws_server.py` or `movement/movement.go`
**Protocol:** WebSocket at `ws://<host>:8081/`

### Run

```bash
# Python
cd servers/robot_controller_backend/movement
python3 movement_ws_server.py

# Go
go run movement.go
```

### Environment variables

```bash
PORT_MOVEMENT=8081
ORIGIN_ALLOW=http://localhost:3000
ROBOT_SIM=0    # 1 = simulation mode (no hardware)
```

### Test

```bash
wscat -c ws://localhost:8081/
# Send: {"type":"move","direction":"forward","speed":0.5}
```

---

## Lighting Server

**Server:** `controllers/lighting/main_lighting.go`
**Protocol:** WebSocket at `ws://<host>:8082/lighting`
**Port:** Hardcoded to 8082

### Run

```bash
cd servers/robot_controller_backend/controllers/lighting
go run main_lighting.go
```

### Test

```bash
wscat -c ws://localhost:8082/lighting
# Send: {"color":"#ff0000","mode":"single","pattern":"static","interval":0,"brightness":1.0}
```

### Gateway proxy (optional)

By default the UI connects directly to port 8082 (lower latency). To route through the gateway instead:

```bash
# Set on the Pi when starting gateway
DS_LIGHT_WS=ws://127.0.0.1:8082/lighting python3 -m uvicorn servers.gateway_api:app --port 7070
```

Then update `.env.local` to use gateway URLs (port 7070) instead of direct (port 8082).

---

## Line Tracker Server

**Server:** `sensors/line_tracking_ws_server.py`
**Protocol:** WebSocket at `ws://<host>:8090/line-tracker`

### Run

```bash
cd servers/robot_controller_backend/sensors
python3 line_tracking_ws_server.py
```

### Environment variables

```bash
LINE_TRACKER_HOST=0.0.0.0
LINE_TRACKER_PORT=8090
LINE_TRACKER_PATH=/line-tracker
RATE_HZ=10
FORCE_SIM=0    # 1 = simulation mode
```

---

## Video Server

**Server:** `video/video_server.py`
**Protocol:** HTTP MJPEG at `http://<host>:5000/video_feed`

### Run

```bash
cd servers/robot_controller_backend/video
python3 video_server.py
```

### Environment variables

```bash
VIDEO_PORT=5000
BIND_HOST=0.0.0.0
PLACEHOLDER_WHEN_NO_CAMERA=1    # Show placeholder if no camera
```

---

## UI Configuration (`.env.local`)

```bash
# Ultrasonic
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE=ws://100.93.225.61:8080/ultrasonic
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN=ws://192.168.6.164:8080/ultrasonic
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL=ws://localhost:8080/ultrasonic

# Movement
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE=ws://100.93.225.61:8081/
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN=ws://192.168.6.164:8081/
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL=ws://localhost:8081/

# Lighting (direct — port 8082)
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE=ws://100.93.225.61:8082/lighting
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN=ws://192.168.6.164:8082/lighting
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL=ws://localhost:8082/lighting

# Line Tracker
NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE=ws://100.93.225.61:8090/line-tracker
NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN=ws://192.168.6.164:8090/line-tracker
NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL=ws://localhost:8090/line-tracker

# Video
NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE=http://100.93.225.61:5000/video_feed
NEXT_PUBLIC_VIDEO_STREAM_URL_LAN=http://192.168.6.164:5000/video_feed
NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL=http://localhost:5000/video_feed
```

Set `NEXT_PUBLIC_NETWORK_PROFILE` to `tailscale`, `lan`, or `local` to select the connection profile.

---

## Running All Servers

```bash
# Using tmux (recommended)
tmux new-session -d -s robot
tmux split-window -h

# Pane 0: Ultrasonic
cd servers/robot_controller_backend/sensors && go run main_ultrasonic.go

# Pane 1: Movement
cd servers/robot_controller_backend/movement && python3 movement_ws_server.py

# Pane 2: Lighting
cd servers/robot_controller_backend/controllers/lighting && go run main_lighting.go

# Pane 3: Video
cd servers/robot_controller_backend/video && python3 video_server.py
```

---

## General Troubleshooting

**Port already in use:**
```bash
lsof -i :8080
kill <PID>
```

**CORS errors:**
```bash
export ORIGIN_ALLOW=http://localhost:3000,http://192.168.1.100:3000
```

**UI can't connect:**
1. Check server is running: `curl http://localhost:8080/` (will fail, but confirms port is up)
2. Check WebSocket path matches (e.g., `/ultrasonic` vs `/`)
3. Check firewall allows the port
4. Verify `.env.local` URLs

**Hardware not working:**
```bash
ROBOT_SIM=1 python3 movement_ws_server.py   # simulation mode
FORCE_SIM=1 python3 line_tracking_ws_server.py
```
