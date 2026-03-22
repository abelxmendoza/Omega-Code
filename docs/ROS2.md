# ROS2 Integration Guide — Omega Robot

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                     Omega Robot ROS2 Ecosystem                       │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  Dev Machine (Mac/Lenovo/Jetson)    Raspberry Pi 4B                  │
│  ┌───────────────────────────┐      ┌──────────────────────────┐     │
│  │ Web UI (Next.js)          │      │ Docker: ROS2 Humble       │     │
│  │ Backend (FastAPI)         │      │  motor_controller_node    │     │
│  │ ROS2-Web Bridge           │      │  sensor_node              │     │
│  │ Action servers            │      │  camera_publisher_node    │     │
│  └──────────────┬────────────┘      └──────────────┬───────────┘     │
│                 └──────────────────────────────────┘                 │
│                        ROS2 DDS (CycloneDDS)                         │
│                        Domain ID: 0                                  │
└──────────────────────────────────────────────────────────────────────┘
```

### Node Graph

| Node | Device | Publishes | Subscribes |
|------|--------|-----------|------------|
| `motor_controller_node` | Pi | `/odom` | `/cmd_vel` |
| `sensor_node` | Pi | `/omega/ultrasonic`, `/omega/line_tracking/state` | — |
| `camera_publisher_node` | Pi | `/camera/image_raw`, `/camera/image_raw/compressed` | — |
| `xbox_teleop_node` | Pi/Lenovo | `/cmd_vel` | — |
| `navigate_to_goal_action_server` | Laptop/Jetson | — | `/odom`, `/omega/ultrasonic` |
| `follow_line_action_server` | Laptop/Jetson | `/cmd_vel` | `/omega/line_tracking/state` |
| `obstacle_avoidance_action_server` | Laptop/Jetson | `/cmd_vel` | `/omega/ultrasonic` |
| `path_planner` | Laptop/Jetson | `/path` | `/odom`, `/map` |
| `vision_processor` | Jetson | `/omega/detections` | `/camera/image_raw` |
| Web Bridge (backend) | Laptop | `/cmd_vel` | `/omega/ultrasonic`, `/omega/line_tracking/state` |

### Topics Reference

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/omega/ultrasonic` | `sensor_msgs/Range` | `sensor_node` | Distance in metres |
| `/omega/line_tracking/state` | `std_msgs/String` (JSON) | `sensor_node` | `{left, center, right}` |
| `/odom` | `nav_msgs/Odometry` | `motor_controller_node` | Wheel odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | UI / teleop | Velocity commands |
| `/camera/image_raw` | `sensor_msgs/Image` | `camera_publisher_node` | Raw frames |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | `camera_publisher_node` | Compressed frames |
| `/omega/detections` | `std_msgs/String` (JSON) | `vision_processor` | YOLO/ML detections (Jetson) |
| `/path` | `nav_msgs/Path` | `path_planner` | Planned path |

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
# On the Pi — hardware IO
ros2 run omega_robot motor_controller
ros2 run omega_robot sensor_node
ros2 run omega_robot camera_publisher_node

# On Laptop/Jetson — planning and action servers
ros2 run omega_robot path_planner
ros2 run omega_robot navigate_to_goal_action_server
ros2 run omega_robot follow_line_action_server
ros2 run omega_robot obstacle_avoidance_action_server

# Monitor topics
ros2 topic list
ros2 topic echo /omega/ultrasonic

# Or launch everything at once
ros2 launch omega_robot robot_full.launch.py
```

### Launch files

| File | Purpose |
|------|---------|
| `pi_only.launch.py` | Pi hardware IO — motor_controller + sensor_node |
| `pi_orin_hybrid.launch.py` | Pi hardware + Jetson Orin AI nodes |
| `robot_full.launch.py` | Full stack — all nodes |
| `omega_camera.launch.py` | Camera + capability detection |
| `omega_brain.launch.py` | Autonomy stack — action servers + path planner |
| `multidevice_setup.launch.py` | Reference — laptop action servers + Pi/Orin node comments |

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
- `motor_controller` — drives PCA9685, publishes `/odom`, subscribes `/cmd_vel`
- `sensor_node` — reads HC-SR04 + line sensors, publishes `/omega/ultrasonic` + `/omega/line_tracking/state`

### ROS2 modes

The backend supports two modes:

**Docker Mode (default):** Nodes run in containers, managed via API.

**Native Mode:** Direct rclpy integration — set `ROS_NATIVE_MODE=true` and source ROS2 before starting the backend.

```bash
export ROS_NATIVE_MODE=true
source /opt/ros/humble/setup.bash
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

### Sensor WebSocket Bridge

`api/sensor_bridge.py` subscribes to ROS2 sensor topics in a daemon thread and fans
data into asyncio queues that WebSocket handlers drain — no blocking of the FastAPI event loop.

**Topics subscribed:**

| Topic | Type | Rate |
|-------|------|------|
| `/omega/ultrasonic` | `sensor_msgs/Range` | ~10 Hz |
| `/omega/line_tracking/state` | `std_msgs/String` (JSON) | ~20 Hz |

**WebSocket endpoints** (same FastAPI server, port 8000):

| Endpoint | Purpose |
|----------|---------|
| `/ws/ultrasonic` | Streams ultrasonic range data; any message = server alive |
| `/ws/line` | Streams line tracking state; ping/pong latency check |
| `/ws/lighting` | Accepts lighting commands, publishes via ROS bridge |

**Protocol:**
```json
// Server → Client on connect
{ "type": "welcome", "service": "<name>", "status": "connected" }

// Client → Server heartbeat
{ "type": "ping", "ts": 1234567890 }

// Server → Client heartbeat
{ "type": "pong", "ts": 1234567890 }

// Server → Client sensor data
{ "type": "ultrasonic", "range_m": 0.42, "stamp": 1234567890.1 }
{ "type": "line_tracking", "left": 1, "center": 0, "right": 1 }
```

Falls back gracefully — if ROS2 is unavailable, bridge is a no-op and endpoints stay open.

### UI Components

- `CameraFrame.tsx` — MJPEG stream via plain `<img>` (Next.js `Image` breaks multipart)
- `XboxControllerStatus.tsx` — shows connected/paused state, axis/button indicators, pause toggle
- `useGamepad.ts` — browser Gamepad API hook, GTA-style trigger/stick mapping, pause support
- ROS dashboard page (`ros.tsx`) — start/stop containers, view telemetry, monitor topics

### Xbox Teleop

The UI's `XboxControllerStatus` component polls the browser Gamepad API via `useGamepad`.
On the Pi, `xbox_teleop_node.py` reads evdev and publishes to `/cmd_vel`:

```bash
ros2 run omega_robot xbox_teleop
```

Wire both together: UI sends `/cmd_vel` over the ROS web bridge; the Pi node also publishes
directly — whichever is active wins at the `motor_controller_node`.

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

**Camera publisher:** `ros/src/omega_robot/omega_robot/camera_publisher_node.py`

```bash
ros2 run omega_robot camera_publisher_node
```

**Parameters:**
- `width` (default: 640)
- `height` (default: 480)
- `fps` (default: 30)
- `publish_compressed` (default: true)

**Camera backend:** GStreamer/OpenCV via `libcamerasrc` (see [HARDWARE.md](HARDWARE.md#video-server)).
The standalone `video/video_server.py` (port 5000) serves MJPEG directly;
the UI consumes it via `/api/video-proxy`. Use `setup_pi_camera.sh` for Pi setup.

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
source ~/.ros2_humble_setup.bash
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
source /opt/ros/humble/setup.bash
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
