# ROS2 Integration Guide — Omega Robot

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                 Omega Robot ROS2 Ecosystem                   │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  Dev Machine (MacBook/Laptop)    Raspberry Pi 4B             │
│  ┌──────────────────────┐        ┌─────────────────────┐     │
│  │ Web UI (Next.js)     │        │ Docker: ROS2 Humble  │     │
│  │ Backend (FastAPI)    │        │ sensor_data_publisher│     │
│  │ ROS2-Web Bridge      │        │ robot_controller     │     │
│  └──────────┬───────────┘        │ enhanced_telemetry   │     │
│             │                    └──────────┬────────────┘    │
│             └────────────────────────────── ┘                │
│                    ROS2 DDS (CycloneDDS)                     │
│                    Domain ID: 0                              │
└──────────────────────────────────────────────────────────────┘
```

### Node Graph

| Node | Publishes | Subscribes |
|------|-----------|------------|
| `sensor_data_publisher` | `/omega/sensors/ultrasonic`, `/omega/sensors/line_tracking`, `/omega/sensors/battery` | — |
| `robot_controller` | `/omega/motors/left`, `/omega/motors/right` | `/cmd_vel` |
| `enhanced_telemetry` | `/omega/telemetry` | `/omega/sensors/*` |
| `camera_publisher` | `/camera/image_raw`, `/camera/image_raw/compressed` | — |
| Web Bridge (backend) | `/cmd_vel` | `/omega/sensors/*`, `/omega/telemetry` |

### Topics Reference

| Topic | Type | Description |
|-------|------|-------------|
| `/omega/sensors/ultrasonic` | Float32 | Distance in cm |
| `/omega/sensors/line_tracking` | Int32MultiArray | `[left, center, right]` sensor states |
| `/omega/sensors/battery` | BatteryState | Voltage and percentage |
| `/omega/motors/left` | Float32 | Left motor speed (-1.0 to 1.0) |
| `/omega/motors/right` | Float32 | Right motor speed (-1.0 to 1.0) |
| `/omega/telemetry` | String | JSON telemetry blob |
| `/cmd_vel` | Twist | Velocity commands from web app |
| `/camera/image_raw` | Image | Raw camera frames |
| `/camera/image_raw/compressed` | CompressedImage | Compressed camera frames |

---

## Quick Start

### Build the ROS2 workspace

```bash
# On Lenovo/Ubuntu with ROS2 installed natively
cd ~/omega_ws
colcon build --packages-select omega_robot
source install/setup.bash
```

### Run nodes

```bash
# Terminal 1: Sensor data
ros2 run omega_robot sensor_data_publisher

# Terminal 2: Robot controller
ros2 run omega_robot robot_controller

# Terminal 3: Monitor topics
ros2 topic list
ros2 topic echo /omega/sensors/ultrasonic

# Or launch everything at once
ros2 launch omega_robot robot_full.launch.py
```

### Launch files

| File | Purpose |
|------|---------|
| `pi_only.launch.py` | Pi-only sensor + motor nodes |
| `pi_orin_hybrid.launch.py` | Pi hardware + Orin AI brain |
| `omega_full.launch.py` | Full stack including camera + autonomy |
| `omega_camera.launch.py` | Camera nodes only |

---

## ROS2 + Docker Integration

ROS2 nodes run in Docker containers on the Pi and are managed via the backend API.

### API Endpoints

```bash
# Start ROS2 containers
curl -X POST http://localhost:8000/api/ros/control \
  -H "Content-Type: application/json" \
  -d '{"action": "start"}'

# Stop containers
curl -X POST http://localhost:8000/api/ros/control \
  -d '{"action": "stop"}'

# Check status
curl http://localhost:8000/api/ros/status

# List active topics
curl http://localhost:8000/api/ros/topics
```

### Architecture flow

```
UI (ros.tsx) → /api/ros/* → gateway_api.py (7070) → ros_routes.py (8000)
  → subprocess docker commands → ROS2 containers
```

### Docker containers

Defined in `docker/ros2_robot/docker-compose.yml`:
- `telemetry_publisher` — publishes sensor data
- `telemetry_listener` — receives and logs telemetry

### ROS2 modes

The backend supports two modes:

**Docker Mode (default):** Nodes run in containers, managed via API.

**Native Mode:** Direct rclpy integration — set `ROS_NATIVE_MODE=true` and source ROS2 before starting the backend.

```bash
export ROS_NATIVE_MODE=true
source /opt/ros/rolling/setup.bash
source ~/omega_ws/install/setup.bash
cd servers/robot_controller_backend
python main_api.py
```

---

## Web Integration (ROS2-Web Bridge)

The backend bridges ROS2 to the web UI via WebSocket.

### Connect from JavaScript

```javascript
const ws = new WebSocket('ws://localhost:8000/api/ros/bridge');

// Move forward
ws.send(JSON.stringify({
  type: "publish",
  topic: "/cmd_vel",
  msg_type: "Twist",
  command: { linear: { x: 0.5, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }
}));

// Subscribe to ultrasonic sensor
ws.send(JSON.stringify({
  type: "subscribe",
  topic: "/omega/sensors/ultrasonic",
  msg_type: "Float32"
}));

// Handle incoming data
ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  if (msg.type === "ros2_message") {
    console.log("Distance:", msg.data.data, "cm");
  }
};
```

### UI Components

- `CameraViewer.tsx` — displays ROS2 camera topic as a live feed
- ROS dashboard page (`ros.tsx`) — start/stop containers, view telemetry, monitor topics

---

## Autonomous Behaviors (Action Servers)

Autonomous modes use ROS2 action servers and are triggered from the **AutonomyModal** in the UI.

### Enable in UI

1. Open AutonomyModal (click "Autonomy")
2. Toggle "Enable ROS2 Integration" ON
3. Check badge turns green (ROS2 connected)
4. Select a mode marked with a rocket icon (uses ROS2)

### Action Servers

#### Navigate to Goal
**File:** `ros/src/omega_robot/omega_robot/navigate_to_goal_action.py`
```bash
ros2 run omega_robot navigate_to_goal_action_server
```
- Navigate to (x, y, theta)
- Obstacle detection via ultrasonic
- Auto-cancels on obstacle

#### Follow Line
**File:** `ros/src/omega_robot/omega_robot/follow_line_action.py`
```bash
ros2 run omega_robot follow_line_action_server
```
- Line tracking sensors
- Configurable duration (0 = infinite)
- Line-lost detection

#### Obstacle Avoidance
**File:** `ros/src/omega_robot/omega_robot/obstacle_avoidance_action.py`
```bash
ros2 run omega_robot obstacle_avoidance_action_server
```

### ROS2 Mode Handlers (Backend)

**File:** `servers/robot_controller_backend/autonomy/modes/ros2_modes.py`

- `ROS2LineFollowMode` — triggers `/follow_line` action
- `ROS2ObstacleAvoidanceMode` — triggers `/obstacle_avoidance` action
- `ROS2WaypointMode` — triggers `/navigate_to_goal` action

Falls back to simple logging modes if ROS2 is unavailable.

---

## Camera Integration

**Camera publisher:** `ros/src/omega_robot/omega_robot/camera_publisher.py`

```bash
ros2 run omega_robot camera_publisher
```

**Parameters:**
- `width` (default: 640)
- `height` (default: 480)
- `fps` (default: 30)
- `publish_compressed` (default: true)

**Supports:** Picamera2 (Pi), OpenCV/V4L2, placeholder frames when no camera is available.

**Web bridge** (`api/ros_web_bridge.py`) handles `Image` and `CompressedImage` types, encoding frames to base64 JPEG for WebSocket transmission.

### OpenCV Integration

OpenCV is used for:
- Camera capture and MJPEG streaming
- Motion detection, object tracking (KCF/CSRT/MOSSE)
- Face detection (Haar cascades)
- ArUco marker detection and pose estimation
- Frame encoding

---

## Multi-Device Setup

See [PLATFORMS.md](PLATFORMS.md) for per-device setup details.

**All devices must share:**
```bash
ROS_DOMAIN_ID=0                         # same on all machines
CYCLONEDDS_URI=/path/to/cyclonedds.xml  # CycloneDDS config
```

**Check topic visibility across devices:**
```bash
# On each machine
ros2 topic list
# All machines should see the same topics if DDS is configured correctly
```

---

## Troubleshooting

### Nodes not found
```bash
cd ~/omega_ws
colcon build --packages-select omega_robot
source install/setup.bash
```

### Topics not visible
```bash
ros2 daemon stop
ros2 daemon start
echo $ROS_DOMAIN_ID  # verify matches across devices
```

### Web bridge not working
```bash
# Verify ROS2 is available in native mode
export ROS_NATIVE_MODE=true
source ~/.ros2_rolling_setup.bash
# Check backend logs for "ROS2-Web bridge initialized"
```

### Container issues
```bash
docker logs omega_ros2
docker exec -it omega_ros2 bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

---

## ROS Environment Variables

ROS is fully optional — the backend runs without it. Use these variables to control behavior:

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_ENABLED` | `true` | Set `false` to disable all ROS features completely |
| `ROS_NATIVE_MODE` | `false` | Set `true` to use rclpy directly instead of Docker containers |
| `ROS_DOCKER_COMPOSE_PATH` | auto | Path to the docker-compose.yml for ROS containers |

### Operating modes

**No ROS** (`ROS_ENABLED=false`):
- All core features work (movement, sensors, lighting)
- ROS endpoints return `503 disabled` status
- Use this for macOS dev or when ROS is not needed

**Docker ROS** (`ROS_ENABLED=true`, `ROS_NATIVE_MODE=false`) — default:
- ROS2 runs in Docker containers
- Dashboard can start/stop/restart containers

**Native ROS** (`ROS_ENABLED=true`, `ROS_NATIVE_MODE=true`):
- Direct rclpy integration, lower latency, no Docker overhead
- Requires ROS2 sourced before starting backend

### Config examples

```bash
# macOS dev — no ROS
export ROS_ENABLED=false
python main_api.py

# Pi with Docker ROS (default)
export ROS_ENABLED=true
export ROS_NATIVE_MODE=false
export ROS_DOCKER_COMPOSE_PATH=/home/pi/Omega-Code/docker/ros2_robot/docker-compose.yml
python main_api.py

# Pi or Lenovo with native ROS
source /opt/ros/rolling/setup.bash
source ~/omega_ws/install/setup.bash
export ROS_ENABLED=true
export ROS_NATIVE_MODE=true
python main_api.py
```

### API behavior when ROS is disabled

`GET /api/ros/status` → `{ "mode": "disabled", "containers": [], "topics": [] }`
`POST /api/ros/control` → HTTP 503
`GET /api/ros/topics` → `{ "topics": [], "message": "ROS features are disabled" }`

---

## ROS Dashboard (`/ros`)

Access via the "ROS" badge in the header or navigate directly to `/ros`.

**Features:**
- Start/stop/restart Docker containers
- Real-time `/omega/telemetry` visualization
- Configurable auto-refresh (1s, 2s, 5s, 10s)
- WebSocket connection status

**Enable verbose debug logging:**
```bash
NEXT_PUBLIC_ROS_DEBUG=1 npm run dev
# Or in browser console:
localStorage.setItem('ros_debug', 'true')  # then refresh
```

**Browser console log prefixes:**
- `[ROS Debug]` — info
- `[ROS Error]` — errors
- `[ROS Network]` — API calls
- `[ROS WebSocket]` — WebSocket events

**DevTools tips:**
- Network tab → filter `/api/ros` to see all ROS API calls
- Network tab → filter `WS` to inspect WebSocket frames
- Console → filter `[ROS` to isolate ROS logs

**Copy any request as curl for manual testing:**
```bash
curl -X POST http://localhost:7070/api/ros/control \
  -H "Content-Type: application/json" \
  -d '{"action":"start"}'
```
