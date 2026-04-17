# Omega-1 — Hybrid ROS2 Robotics Platform

<p align="center">
  <img src="image/README/1719168223539.png" alt="Omega-1 Robot" width="480"/>
  <br/>
  <img src="image/README/1719064424761.png" alt="Omega-1 Logo" width="300"/>
</p>

Omega-1 is a modular autonomous robotics platform built around a Raspberry Pi 4B and a
Freenove 4WD chassis. It runs a **hybrid architecture**: a Next.js web UI and FastAPI
backend handle operator interaction, while ROS2 (Humble) handles motor control, sensor
integration, servo pan-tilt, and autonomous operation.

---

## Quick Start

### New Pi or SD card — one script does everything

```bash
git clone https://github.com/abelxmendoza/Omega-Code.git ~/Desktop/Omega-Code
cd ~/Desktop/Omega-Code
chmod +x scripts/setup_fresh_pi.sh
./scripts/setup_fresh_pi.sh
sudo reboot
```

`setup_fresh_pi.sh` installs all system packages, creates the Python venv, runs
`pip install -r requirements.txt`, fetches Go dependencies, configures I2C/SPI/camera
in `/boot/firmware/config.txt`, installs the Xbox udev rule, and writes the full
`@reboot` crontab so all services start automatically after reboot.

After the reboot, these services start on their own:

| Service | Port | Log |
|---|---|---|
| FastAPI (main_api.py) | 8000 | `/tmp/main_api.log` |
| Movement WebSocket | 8081 | `/tmp/movement_ws.log` |
| Line-tracker WebSocket | — | `/tmp/line_tracker.log` |
| Ultrasonic WebSocket (Go) | — | `/tmp/ultrasonic.log` |

Check they started:
```bash
tail -f /tmp/main_api.log
curl http://localhost:8000/health   # → "ok"
```

### Dev machine — run the UI pointing at the Pi

```bash
cd ui/robot-controller-ui
cp .env.local.example .env.local
# Edit .env.local: set NEXT_PUBLIC_ROBOT_HOST_LAN=<pi-ip>
#                  set NEXT_PUBLIC_NETWORK_PROFILE=lan
npm install
npm run dev   # http://localhost:3000
```

---

## Engineering Overview

During development the control system exhibited critical instability: duplicate WebSocket
channels sent conflicting motor commands, frontend polling exceeded ~240 requests/min
(overwhelming the Pi's FastAPI backend), health checks silently failed by issuing
`fetch("ws://...")` calls (WS scheme is illegal for HTTP fetch), and the Performance
Dashboard displayed randomly-generated fake data that masked real system state.

The system was redesigned into a centralized, fault-tolerant architecture:

| Problem | Solution |
|---|---|
| Duplicate WebSocket control channels | Single-source-of-truth via `CommandContext`; all command sending delegates through one WS connection |
| ~240 HTTP requests/min to dead endpoints | Circuit breakers on every poller (3 failures → 60 s backoff); polling reduced to <10 req/min at rest |
| N independent health-check loops | `SystemHealthService` singleton + `SystemHealthContext` — one `/api/health` probe loop for the entire app |
| `fetch("ws://...")` silent failures | `toHealthUrl()` / `wsUrlToHttp()` normalize all WS URLs to HTTP before any fetch call |
| Fake simulated performance data | Removed; `PerformanceDashboard` shows real Pi metrics or a clear "backend offline" state |
| No automated test coverage | 156 tests added: unit (circuit breaker, URL helpers, env profile), integration (context, API), E2E (Cypress) |
| N duplicate system-mode polling loops | `SystemModeContext` shares one `/api/system/mode/status` loop across all consumers — polling stays O(1) regardless of how many components mount |
| PCA9685 shared PWM clock bug (500 Hz → invalid servo pulses) | Motor driver locked to 50 Hz; `Servo.__init__` reads the PRESCALE register on startup and auto-corrects if another process changed the frequency; all servo pulses hard-clamped to 1200–1800 µs on every command |

**Result:** HTTP requests <10/min at rest, stable WS control, accurate system state, no console noise from protocol mismatches.

---

## System Architecture

```
[Next.js UI]  http/ws
     |
     |--/api/video-proxy --> MJPEG stream (same-origin proxy, avoids CORS)
     |
[FastAPI]  port 8000    <-- OmegaRosBridge embeds rclpy node here
     |  publishes /cmd_vel_in (Twist)
     |  ROS2 DDS (CycloneDDS)
     |
+----+----------------------------------------------+
|               Raspberry Pi 4B                     |
|   ROS2 Node Graph (omega_hybrid.launch.py)        |
|                                                    |
|   xbox_teleop          --> /cmd_vel_in             |
|   (evdev, daemon thread)--> /omega/servo_increment |
|                                                    |
|   FastAPI ROS bridge   --> /cmd_vel_in             |
|                                                    |
|   obstacle_avoidance_node <-- /cmd_vel_in          |
|   (ultrasonic mux)        <-- /omega/ultrasonic    |
|        PASS_THROUGH / WARN / PIVOTING state machine|
|                           --> /cmd_vel             |
|                                                    |
|   motor_controller   <-- /cmd_vel                 |
|         |                --> /odom                 |
|         |                --> /omega/motor_state    |
|         v                --> /tf  (odom->base_link)|
|      PCA9685 (I2C 0x40)                           |
|      4WD motor HAL                                |
|                                                    |
|   sensor_node             --> /omega/ultrasonic    |
|   (HC-SR04, IR, ADS1115)  --> /omega/line_tracking/*|
|                            --> /omega/battery      |
|                                                    |
|   servo_controller   <-- /omega/servo_increment   |
|         |                --> /omega/servo_state    |
|         v                                          |
|      PCA9685 ch8/ch9  (pan-tilt gimbal)           |
|                                                    |
|   video/video_server.py   (Flask, port 5000)      |
|   GStreamer libcamerasrc pipeline                  |
|   CSI OV5647            --> MJPEG /video_feed     |
|                          --> /health               |
|                                                    |
|   omega_camera/camera_node  (ROS2)                |
|   (libcamerasrc GStreamer pipeline)               |
|   CSI OV5647                --> /omega/camera/image_raw       |
|                             --> /omega/camera/image_raw/compressed|
|                                                    |
|   system_capabilities     --> /omega/capabilities  |
|   static_tf               --> base_link->camera_link|
|                            --> base_link->ultrasonic_front|
+---------------------------------------------------+
         |
         | CycloneDDS (Tailscale)
         v
+---------------------------------------------------+
|             Jetson Orin (future)                  |
|   omega_detection/detection_node                  |
|   <-- /omega/camera/image_raw/compressed          |
|   --> /omega/detections  (Detection2DArray)       |
|   --> /omega/camera/annotated/compressed          |
+---------------------------------------------------+

Non-ROS (managed separately by OmegaOS / systemd):
  FastAPI REST/WS       http://omegaone:8000
  Go lighting server    ws://omegaone:8082
  MJPEG video stream    http://omegaone:5000   (proxied via /api/video-proxy)

Multi-machine DDS (CycloneDDS):
  Pi (omegaOne) <--> Laptop <--> Jetson Orin (Tailscale)
```

### TF Tree

```
odom
 └── base_link          (dynamic: motor_controller @ 50 Hz)
      ├── camera_link   (static: 8 cm fwd, 10 cm up)
      └── ultrasonic_front  (static: 12 cm fwd)
```

### ROS Topic Map

| Topic | Type | Hz | Producer |
|---|---|---|---|
| `/cmd_vel_in` | `geometry_msgs/Twist` | on-demand | FastAPI bridge, xbox_teleop |
| `/cmd_vel` | `geometry_msgs/Twist` | on-demand | obstacle_avoidance_node (mux) |
| `/odom` | `nav_msgs/Odometry` | 50 | motor_controller_node |
| `/omega/motor_state` | `omega_interfaces/MotorState` | 10 | motor_controller_node |
| `/omega/ultrasonic` | `sensor_msgs/Range` | 10 | sensor_node |
| `/omega/line_tracking/center` | `std_msgs/Bool` | 20 | sensor_node |
| `/omega/battery` | `sensor_msgs/BatteryState` | 1 | sensor_node |
| `/omega/obstacle_avoidance/state` | `std_msgs/String` (JSON) | 20 | obstacle_avoidance_node |
| `/omega/camera/image_raw` | `sensor_msgs/Image` (bgr8) | 10 | omega_camera/camera_node |
| `/omega/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 10 | omega_camera/camera_node |
| `/omega/servo_increment` | `geometry_msgs/Vector3` | on-demand | xbox_teleop / web UI |
| `/omega/servo_state` | `std_msgs/String` (JSON) | 1 | servo_controller_node |
| `/omega/capabilities` | `std_msgs/String` (JSON) | 0.2 | system_capabilities |
| `/omega/detections` | `vision_msgs/Detection2DArray` | ~10 | detection_node *(Jetson, future)* |

---

## Repository Structure

```
Omega-Code/
├── setup.py                        # pip install -e . makes servers/ importable
├── ros/
│   └── src/
│       ├── omega_robot/            # Core ROS nodes
│       │   └── omega_robot/
│       │       ├── motor_controller_node.py
│       │       ├── sensor_node.py
│       │       ├── servo_controller_node.py
│       │       ├── xbox_teleop_node.py      # publishes /cmd_vel_in
│       │       ├── ultrasonic_avoidance_node.py  # cmd_vel mux
│       │       ├── camera_debug_node.py
│       │       ├── system_capabilities.py
│       │       └── ... (action servers, path planner, orin_ai_brain)
│       ├── omega_camera/           # Camera pipeline (separate package)
│       │   └── omega_camera/
│       │       └── camera_node.py  # libcamerasrc → OpenCV → ROS2
│       ├── omega_detection/        # Perception / YOLO (in development)
│       │   └── omega_detection/
│       │       └── detection_node.py
│       ├── omega_bringup/          # Launch files + CycloneDDS config
│       │   ├── launch/
│       │   │   ├── omega_hybrid.launch.py   # full stack on Pi
│       │   │   ├── omega_minimal.launch.py  # motor + sensors only
│       │   │   └── omega_sim.launch.py      # sim mode (no hardware)
│       │   └── config/
│       │       ├── cyclonedds_pi.xml
│       │       └── cyclonedds_laptop.xml
│       └── omega_interfaces/       # Custom msg / srv / action types
│           ├── msg/MotorState.msg
│           └── ...
├── servers/
│   └── robot_controller_backend/
│       ├── movement/               # Motor HAL: PCA9685, ramp, PID, odometry, watchdog
│       │   ├── pose_ekf.py         # SE(2) Extended Kalman Filter (dead-reckoning + ArUco)
│       │   └── hardware/
│       │       └── servo_control.py  # PCA9685 servo driver (ch8 pan, ch9 tilt)
│       ├── sensors/                # HC-SR04, IR, ADC WebSocket servers
│       ├── simulation/             # Software-sim engine (no hardware required)
│       │   ├── sim_engine.py       # Differential-drive kinematics + ArUco rvec/tvec synthesis
│       │   └── __init__.py
│       ├── video/
│       │   ├── video_server.py     # Flask MJPEG server (port 5000)
│       │   └── camera.py           # GStreamer/libcamerasrc Camera class
│       ├── api/
│       │   ├── ros_bridge.py       # FastAPI <-> ROS2 bridge (publishes /cmd_vel_in)
│       │   ├── sensor_bridge.py    # ROS subscriber → WebSocket fan-out
│       │   ├── sensor_ws_routes.py # WS /ws/ultrasonic, /ws/line, /ws/battery, /ws/radar
│       │   ├── localization_routes.py  # SE(2) EKF pose, ArUco correction, marker map
│       │   ├── sim_routes.py       # Simulation control API (SIM_MODE=1 only)
│       │   ├── movement_routes.py  # REST movement and servo endpoints
│       │   ├── config_routes.py    # Read/write omega_config sections
│       │   ├── system_mode_routes.py
│       │   ├── service_routes.py
│       │   ├── capability_routes.py
│       │   ├── security_middleware.py
│       │   └── ...
│       ├── autonomy/               # Pluggable autonomy controller + mode handlers
│       ├── controllers/            # Servo, buzzer, lighting device drivers
│       ├── omega_config/           # YAML config + config_manager.py
│       ├── omega_services/         # Process supervisor, service manager, systemd unit
│       ├── network/                # Wi-Fi scan, AP mode, Tailscale, network watchdog
│       ├── hardware/               # Camera drivers, motor/LED helpers, hardware detection
│       ├── servers/                # Gateway proxy (gateway_api.py) + snapshot blueprint
│       ├── scripts/
│       │   └── start_sim_local.sh  # Launch the sim stack locally (SIM_MODE=1)
│       ├── tests/                  # 53 test files across 9 suites (unit → faults)
│       ├── requirements.txt        # All Python dependencies (pip install -r requirements.txt)
│       └── main_api.py             # FastAPI entry point + security middleware stack
└── ui/
    └── robot-controller-ui/        # Next.js 13 (Pages Router) operator dashboard
        ├── .env.local.example      # Copy to .env.local and set Pi IP / profile
        ├── package.json            # npm dependencies (npm install)
        ├── src/
        │   ├── pages/
        │   │   ├── index.tsx       # Main operator dashboard
        │   │   ├── mission.tsx     # Mission Control page (waypoints + sim launcher)
        │   │   ├── network.tsx     # Network wizard
        │   │   └── api/            # Next.js API routes (video-proxy, sim-launcher, etc.)
        │   ├── components/
        │   │   ├── LocalizationPanel.tsx   # SE(2) pose canvas + quality badge
        │   │   ├── mission/
        │   │   │   ├── MissionControlPanel.tsx  # Waypoint list, mission state
        │   │   │   ├── MissionMap.tsx           # 2D map canvas
        │   │   │   └── SimLauncherCard.tsx      # Launch sim mode from UI
        │   │   └── ... (control/, lighting/, sensors/, network/, ros/, etc.)
        │   ├── context/            # CommandContext, SystemHealthContext, SystemModeContext
        │   ├── services/           # systemHealth.ts — singleton poll + circuit breaker
        │   ├── hooks/
        │   │   ├── usePoseStream.ts       # Polls /localization/pose → LocalizationPanel
        │   │   ├── useMissionStream.ts    # WS hook for mission state
        │   │   └── ... (useWsStatus, useGamepad, useRobotOnline, etc.)
        │   ├── utils/              # robotFetch, urlHelpers, envProfile, autonomy client
        │   ├── config/             # gateway.ts — profile-aware URL resolution
        │   └── themes/             # omega-theme.ts (dark), cyber-theme.ts (neon)
        └── tests/                  # 156 Jest + MSW + Cypress tests

scripts/
├── setup_fresh_pi.sh   # Full bootstrap for new Pi/SD card (run once, then reboot)
└── setup_pi_camera.sh  # Camera-stack-only bootstrap (called by setup_fresh_pi.sh)
```

---

## Hardware

| Component | Details |
|---|---|
| Compute | Raspberry Pi 4B (4 GB) |
| Chassis | Freenove 4WD smart car |
| Motor driver | PCA9685 PWM hat (I2C 0x40) |
| Motors | 4× DC gear motors — ch0–7 |
| Pan-tilt | 2-axis servo gimbal — PCA9685 ch8 (yaw), ch9 (pitch) |
| Ultrasonic | HC-SR04 (GPIO 27 trig / GPIO 22 echo) |
| Line tracking | 3× IR sensors (GPIO 14/15/23) |
| Battery ADC | ADS1115 (I2C 0x48) |
| Camera | CSI OV5647 (via libcamera + GStreamer) |
| Controller | Xbox One S/X wireless (via xpad or Bluetooth) |

### Motor Channel Map (Freenove Ordinary_Car)

| Wheel | Forward ch | Backward ch |
|---|---|---|
| Left upper (front-left) | ch1 | ch0 |
| Left lower (rear-left) | ch2 | ch3 |
| Right upper (front-right) | ch7 | ch6 |
| Right lower (rear-right) | ch5 | ch4 |

STOP = set both channels of a wheel to 4095.

---

## Requirements

### All Machines
- ROS2 Humble Hawksbill
- Python 3.10+
- `sudo apt install ros-humble-rmw-cyclonedds-cpp`

### Raspberry Pi 4B
- Ubuntu 22.04 Server (64-bit)
- `sudo apt install python3-lgpio python3-smbus`
- `sudo apt install ros-humble-cv-bridge python3-opencv`
- Xbox controller: `sudo usermod -aG input $USER && pip install evdev`
- libcamera built from source (see [Camera Setup](#camera-setup) below)
- Hardware: PCA9685 (I2C 0x40), HC-SR04 (GPIO 27/22), IR sensors (GPIO 14/15/23), ADS1115 (I2C 0x48)

### Development Laptop
- Ubuntu 22.04 Desktop
- `sudo apt install ros-humble-desktop`
- Node.js 18+ (for UI development)

### Jetson Orin (detection node — future)
- JetPack 5.x + ROS2 Humble
- `sudo apt install ros-humble-vision-msgs`
- PyTorch + Ultralytics YOLOv8 + TensorRT
- Tailscale for VPN connectivity to Pi

---

## Camera Setup

The CSI OV5647 requires libcamera 0.7.0 with a GStreamer plugin. The Ubuntu apt package is
incomplete on ARM64 — libcamera must be built from source and installed to system paths.

### Automated setup (recommended)

```bash
# Run once on a fresh Pi — handles all system-level changes:
chmod +x scripts/setup_pi_camera.sh
./scripts/setup_pi_camera.sh
source ~/.bashrc
```

The script covers:
1. `apt install python3-opencv` (replaces pip opencv-python which lacks GStreamer support)
2. `pip3 install "numpy<2"` (cv_bridge compiled against NumPy 1.x)
3. Blacklists `bcm2835_v4l2` kernel module (conflicts with libcamera at boot)
4. Installs libcamera `.so` files, GStreamer plugin, IPA modules, tuning JSONs to `/usr/local/`
5. Creates shared-library symlinks in `/usr/local/lib/` (ldconfig requires these)
6. Writes `/etc/ld.so.conf.d/libcamera-local.conf` and runs `ldconfig`
7. Appends env vars to `~/.bashrc` (idempotent)

### Build libcamera from source (Pi only, one-time)

```bash
# Dependencies
sudo apt install meson ninja-build python3-jinja2 python3-ply \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libglib2.0-dev libyaml-dev libssl-dev

# Clone and build
git clone https://git.libcamera.org/libcamera/libcamera.git ~/libcamera
cd ~/libcamera && git checkout v0.7.0
meson setup build \
  -Dpipelines=rpi/vc4 \
  -Dipas=rpi/vc4 \
  -Dgstreamer=enabled \
  -Dtest=false \
  -Ddocumentation=disabled
ninja -C build

# Install to /usr/local/ (setup_pi_camera.sh automates this)
sudo cp build/src/libcamera/libcamera*.so* /usr/local/lib/
sudo cp build/src/libcamera/libcamera*.so* /usr/local/lib/
sudo cp build/src/gstreamer/libgstlibcamera.so /usr/local/lib/aarch64-linux-gnu/gstreamer-1.0/
sudo ldconfig
```

### Required environment variables (add to `~/.bashrc` on Pi)

```bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0:${GST_PLUGIN_PATH:-}
export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa
export PYTHONPATH=/usr/local/lib/python3/dist-packages:${PYTHONPATH:-}
```

These must be set in every shell that runs the camera node or video server.
The libcamera log line `"libcamera is not installed"` is expected and harmless; it means
the local build is in use rather than a system install.

### MJPEG video server (UI camera feed)

The web UI streams live video via a Flask server at port 5000. This is separate from the
ROS2 camera node and is the primary feed displayed in the dashboard.

```bash
# Start the video server on the Pi
cd ~/Desktop/Omega-Code/servers/robot_controller_backend/video
python3 video_server.py
# Stream available at http://omegaone:5000/video_feed
# Health endpoint:   http://omegaone:5000/health
```

The Next.js UI proxies the stream through `/api/video-proxy` to avoid CORS/mixed-content.
`CameraFrame.tsx` renders it as a plain `<img>` tag (Next.js `<Image>` breaks MJPEG streams).

Configure the Pi's video URL in `ui/robot-controller-ui/.env.local`:
```
NEXT_PUBLIC_VIDEO_STREAM_URL_LAN=http://omegaone:5000/video_feed
```

### Camera node (ROS2)

```bash
# Run standalone (outside the main launch)
ros2 run omega_camera camera_node

# Verify frames are streaming
ros2 topic hz /omega/camera/image_raw        # ~10 Hz
ros2 topic echo /omega/camera/image_raw --field header  # stamp.sec should be non-zero
```

The camera node uses this GStreamer pipeline:
```
libcamerasrc !
video/x-raw,width=640,height=480,framerate=10/1 !
queue leaky=downstream max-size-buffers=1 !
videoconvert ! video/x-raw,format=BGR !
appsink max-buffers=1 drop=true sync=false
```

`queue leaky=downstream` and `appsink drop=true sync=false` prioritise low latency
over completeness — stale frames are dropped rather than buffered.

### Headless debugging (no GUI)

```bash
# Save a frame to disk and SCP it to your laptop
python3 - <<'EOF'
import cv2
cap = cv2.VideoCapture(
    "libcamerasrc ! video/x-raw,width=640,height=480,framerate=10/1 ! videoconvert ! appsink drop=true",
    cv2.CAP_GSTREAMER
)
ret, frame = cap.read()
if ret:
    cv2.imwrite("/tmp/test_frame.jpg", frame)
    print("Saved /tmp/test_frame.jpg")
cap.release()
EOF

scp omega1@omegaOne:/tmp/test_frame.jpg ~/Desktop/
```

---

## Installation

### Option A — Fresh Pi (recommended for hardware setup)

See the [Quick Start](#quick-start) section at the top. `scripts/setup_fresh_pi.sh`
handles everything: system packages, Python venv, Go, Node.js, hardware config, crontab.

### Option B — Manual step-by-step

#### 1. Clone

```bash
git clone https://github.com/abelxmendoza/Omega-Code.git ~/Desktop/Omega-Code
```

#### 2. Python venv + dependencies

```bash
cd ~/Desktop/Omega-Code/servers/robot_controller_backend
python3 -m venv venv --system-site-packages
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

#### 3. Install backend as Python package

Makes `servers.robot_controller_backend.*` importable in ROS nodes:

```bash
pip install -e ~/Desktop/Omega-Code/
```

#### 4. UI dependencies (dev machine)

```bash
cd ~/Desktop/Omega-Code/ui/robot-controller-ui
npm install
cp .env.local.example .env.local
# Edit .env.local — set your Pi IP and NEXT_PUBLIC_NETWORK_PROFILE
```

#### 5. Create the Colcon workspace

```bash
mkdir -p ~/omega_ws/src
cd ~/omega_ws/src
ln -s ~/Desktop/Omega-Code/ros/src/omega_robot      omega_robot
ln -s ~/Desktop/Omega-Code/ros/src/omega_bringup    omega_bringup
ln -s ~/Desktop/Omega-Code/ros/src/omega_interfaces omega_interfaces
ln -s ~/Desktop/Omega-Code/ros/src/omega_camera     omega_camera
ln -s ~/Desktop/Omega-Code/ros/src/omega_detection  omega_detection
```

#### 6. Install ROS dependencies

```bash
source /opt/ros/humble/setup.bash
cd ~/omega_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### 7. Build

```bash
cd ~/omega_ws
colcon build --packages-select omega_interfaces
colcon build --packages-select omega_robot omega_bringup omega_camera omega_detection \
  --symlink-install
```

`--symlink-install` means Python edits to node files are live without rebuild.

#### 8. Environment (add to `~/.bashrc`)

```bash
source /opt/ros/humble/setup.bash
source ~/omega_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Pi:
export CYCLONEDDS_URI=file://$HOME/Desktop/Omega-Code/ros/src/omega_bringup/config/cyclonedds_pi.xml
# Laptop:
# export CYCLONEDDS_URI=file://$HOME/Desktop/Omega-Code/ros/src/omega_bringup/config/cyclonedds_laptop.xml

# Pi only — required for libcamerasrc:
export LD_LIBRARY_PATH=$HOME/libcamera/build/src/libcamera:$LD_LIBRARY_PATH
export GST_PLUGIN_PATH=$HOME/libcamera/build/src/gstreamer:$GST_PLUGIN_PATH
```

Update peer IPs in the CycloneDDS XML files to match your network.

---

## Obstacle Avoidance

The `obstacle_avoidance_node` acts as a `cmd_vel` mux — all velocity commands from the
web UI and Xbox controller flow through it before reaching the motor controller.

```
FastAPI bridge  ──┐
xbox_teleop     ──┤──> /cmd_vel_in ──> obstacle_avoidance_node ──> /cmd_vel ──> motor_controller
                   │                        ^
                   │              /omega/ultrasonic (HC-SR04)
                   │              camera snapshot (optional secondary)
```

**States:**

| State | Condition | Behaviour |
|---|---|---|
| `PASS_THROUGH` | dist > warn_m (0.50 m) | Commands pass through unchanged |
| `WARN` | stop_m < dist ≤ warn_m | Forward speed scaled **proportionally** to clearance (smooth deceleration, not binary stop) |
| `PIVOTING` | dist ≤ stop_m (0.25 m) | Pivots until the full warn zone is clear; direction alternates per obstacle |
| `RECOVERING` | brief post-pivot | Counter-pivot to partially restore original heading |
| `SENSOR_OFFLINE` | no valid reading > 0.5 s | Motors halted; resumes automatically when sensor recovers |
| `ESTOP` | ESTOP_EVENT set | Motors halted; waits for event clear (next `/autonomy/start`) |

**Optional camera secondary sensor:** set `camera_assist=true` in the start payload.
The Flask video server snapshot at `~2 FPS` is analysed for edge density; if an obstacle
is visually confirmed the effective stop threshold is raised for an earlier reaction.

**Heading recovery:** `recovery_ratio` (default 30%) — after each pivot the robot
counter-rotates for `recovery_ratio × pivot_time` seconds to partially restore the
original heading across multiple consecutive obstacles.

```bash
# Monitor avoidance state
ros2 topic echo /omega/obstacle_avoidance/state

# Enable / disable at runtime
ros2 topic pub --once /omega/obstacle_avoidance/enable std_msgs/Bool '{data: false}'
```

**Launch overrides:**
```bash
# Disable avoidance (pass-through only)
ros2 launch omega_bringup omega_hybrid.launch.py launch_avoidance:=false

# Tune distances and pivot behaviour
ros2 launch omega_bringup omega_hybrid.launch.py \
  warn_distance_m:=0.60 \
  stop_distance_m:=0.30 \
  pivot_speed:=0.80 \
  pivot_duration_s:=1.0
```

---

## Running the Robot

### Auto-start (recommended — after setup_fresh_pi.sh + reboot)

All services start automatically via `@reboot` crontab. Nothing to do manually.
Check status:

```bash
curl http://localhost:8000/health        # FastAPI → "ok"
tail -20 /tmp/main_api.log              # FastAPI log
tail -20 /tmp/movement_ws.log           # Movement WS log
tail -20 /tmp/ultrasonic.log            # Ultrasonic (Go) log
```

### Full hybrid stack (Pi, hardware mode — manual)

```bash
# Terminal 1 — ROS2 nodes (motor, sensors, obstacle avoidance, servo, TF)
ros2 launch omega_bringup omega_hybrid.launch.py

# Terminal 2 — MJPEG video server (camera feed for web UI)
cd ~/Desktop/Omega-Code/servers/robot_controller_backend/video
python3 video_server.py

# Terminal 3 — FastAPI backend (movement WS, sensor WS, localization EKF)
cd ~/Desktop/Omega-Code/servers/robot_controller_backend
venv/bin/uvicorn main_api:app --host 0.0.0.0 --port 8000

# Terminal 4 — ROS2 camera node (optional — for ROS topic subscribers)
ros2 run omega_camera camera_node
```

### Launch overrides

```bash
# Skip servo gimbal
ros2 launch omega_bringup omega_hybrid.launch.py launch_servo:=false

# Disable obstacle avoidance (raw pass-through)
ros2 launch omega_bringup omega_hybrid.launch.py launch_avoidance:=false

# Motor + sensors only
ros2 launch omega_bringup omega_minimal.launch.py
```

---

## Simulation Mode (no hardware required)

The software simulation engine lets you develop and test the full localization + mission
control stack on a laptop or Pi without any physical robot. A simulated differential-drive
robot runs in Python, synthesises geometrically correct ArUco observations, and posts them
to the real `/localization/aruco_update` endpoint — the SE(2) EKF receives them exactly
as it would from a real camera.

### Start sim locally

```bash
cd servers/robot_controller_backend
bash scripts/start_sim_local.sh
# or:
SIM_MODE=1 venv/bin/uvicorn main_api:app --host 0.0.0.0 --port 8000
```

### Sim API endpoints (only mounted when SIM_MODE=1)

| Method | Path | Description |
|--------|------|-------------|
| `POST` | `/sim/start` | Start the simulation loop |
| `POST` | `/sim/stop` | Stop the simulation loop |
| `POST` | `/sim/velocity` | Inject linear/angular velocity (m/s, rad/s) |
| `POST` | `/sim/teleport` | Jump robot to an arbitrary pose `{x, y, theta_rad}` |
| `POST` | `/sim/load_scenario` | Load a named scenario (circle, figure8, grid, etc.) |
| `GET`  | `/sim/state` | Current sim state (pose, velocity, running) |
| `GET`  | `/sim/scenarios` | List built-in scenarios |
| `WS`   | `/sim/ws` | Real-time sim state stream (100 ms frames) |

### Sim from the UI

Open the **Mission Control** page (`/mission`) in the UI. The **Sim Launcher** card lets
you start/stop the sim, choose a scenario, and inject velocity commands directly from the
browser. The `LocalizationPanel` on the main dashboard shows the EKF pose updating live.

---

## Mission Control

The Mission Control page (`/mission`) provides:

- **2D mission map** — top-down canvas showing robot position (from EKF), waypoints, and path
- **Waypoint management** — add, remove, and reorder waypoints; send the list to the robot
- **Sim Launcher** — start the software simulation engine without touching the terminal
- **Live EKF feed** — pose quality badge, uncertainty ellipse, ArUco correction count

Access: `http://localhost:3000/mission` when running the UI in dev mode, or navigate via the header.

---

## Xbox Controller (Teleop)

```bash
ros2 run omega_robot xbox_teleop
```

| Input | Action |
|---|---|
| Right Trigger | Drive forward |
| Left Trigger | Drive reverse / brake |
| Left Stick X | Steer (angular.z) |
| Right Stick X/Y | Pan-tilt camera (continuous) |
| D-Pad Left/Right | Pan camera (step) |
| D-Pad Up/Down | Tilt camera (step) |
| A Button (hold) | Emergency stop |

Pure left-stick with no triggers → in-place pivot.

### Bluetooth pairing (one-time setup)

The FastAPI-mode teleop (`movement/xbox_controller_teleop.py`) detects the controller
automatically over USB or Bluetooth via `evdev` — no configuration change needed.
After pairing once with `bluetoothctl` the controller reconnects automatically:

```bash
sudo bluetoothctl
power on
agent on
scan on          # press the Bluetooth button on the controller
pair <MAC>
trust <MAC>
connect <MAC>
```

If the controller drops mid-session the teleop loop retries reconnection every 2 s
for up to 30 s before giving up. A 20 Hz keep-alive loop ensures the movement
watchdog never fires while the controller is held steady.

---

## Servo Pan-Tilt

The servo controller listens to `/omega/servo_increment` (`geometry_msgs/Vector3`):
- `x` → yaw delta (positive = right)
- `y` → pitch delta (positive = up)

Values are normalised `[-1.0, 1.0]`. Scaled by `scale_us` (default 15 µs/unit),
clamped to hard limits (yaw 1000–2000 µs, pitch 1300–1700 µs).

```bash
# Pan right
ros2 topic pub --once /omega/servo_increment geometry_msgs/Vector3 '{x: 0.5, y: 0.0}'
# Tilt up
ros2 topic pub --once /omega/servo_increment geometry_msgs/Vector3 '{x: 0.0, y: 0.5}'
# Monitor position
ros2 topic echo /omega/servo_state
```

### FastAPI / WebSocket path

The FastAPI backend (`controllers/servo_control.py`) also drives the servos
directly via WebSocket commands (`servo-horizontal`, `servo-vertical`, `set-servo-position`,
`reset-servo`). Safe pulse range is **1200–1800 µs** on both channels; all commands are
hard-clamped to this range before any I2C write.

### PCA9685 shared clock — safety constraint

All 16 PCA9685 channels share one global PWM clock. **The clock must stay at 50 Hz.**
Motors use channels 0–7; servos use channels 8–9. If the frequency is raised (e.g. to
500 Hz for motor smoothness), servo channels output ~150 µs pulses — below the valid
servo range — causing immediate oscillation.

Guardrails in place:
- `motor_driver_pi.py` sets and verifies 50 Hz at startup; logs `[SERVO_SAFETY]` error if the I2C write fails.
- `Servo.__init__` reads the PRESCALE register after `setPWMFreq(50)` and auto-corrects + re-centers if another process changed it.
- Every `setServoPwm()` call clamps the computed pulse to 1200–1800 µs regardless of input angle.

### Power-on procedure

```
1. Flip main Pi power switch (boots Pi only — do NOT power servos yet)
2. Wait ≥ 10 seconds
   • t=8s:  crontab fires → PCA9685 set to 50 Hz, ch8/9 → 1500 µs (center)
   • t=12s: movement_ws_server starts → verifies 50 Hz
3. Flip hat power switch (S1/S2) → battery voltage to servos
   Servo receives 1500 µs → moves smoothly to center and holds still
```

Verify crontab is present: `ssh omegaone crontab -l | grep reboot`
Verify live frequency: `i2cget -y 1 0x40 0xFE` → should return `0x79` (121 = 50 Hz prescale).

---

## Radar Sweep

The `/ws/radar` WebSocket endpoint sweeps the pan servo (PCA9685 ch8) back and forth
while reading HC-SR04 distance at each angle, streaming polar scan frames to the UI.

```
Client                           FastAPI /ws/radar
  │── { command: "start-sweep",       │
  │     step_deg: 5,                  │── move servo to angle
  │     dwell_ms: 150,                │── wait for settle
  │     sweep_min: 30,                │── read HC-SR04
  │     sweep_max: 150 }              │
  │◄── { type: "radar_scan",          │
  │      angle_deg: 45,               │
  │      distance_cm: 38.2,           │
  │      ts: 1714000000000 }  ────────┘
  │── { command: "stop-sweep" }
```

The UI (`UltrasonicVisualization`) renders a phosphor-green radar display on a `<canvas>`:
- Sweep animation with fading blip trails
- Scan line showing current servo angle
- Distance scale rings at 0.5 m / 1.0 m / 1.5 m
- Accessible from the Sensor Dashboard via the **Radar** button (opens as a portal modal)

All servo commands through the radar sweep path are subject to the same 1200–1800 µs
clamp. `sweep_min` is capped to 30–89° and `sweep_max` to 91–150° server-side.

---

## Manual Motor Control

```bash
# Forward 50%
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Rotate left
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.6}}'

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'

# Continuous (10 Hz)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.2}}'
```

---

## Detection Architecture (Pi + Jetson)

The perception pipeline splits across two machines to keep the Pi's CPU free for
real-time control:

```
Pi                                   Jetson Orin
────────────────────                 ─────────────────────────────
camera_node                          detection_node
  /omega/camera/image_raw            <-- /omega/camera/image_raw/compressed
  /omega/camera/image_raw/compressed     (decompress → YOLOv8 TensorRT)
        │                            --> /omega/detections
        │                                (vision_msgs/Detection2DArray)
        │◄────────────────────────────── /omega/detections
        │
decision_node (Pi)
  <-- /omega/detections
  <-- /omega/ultrasonic
  --> /cmd_vel
  --> /omega/servo_increment  (track detected target)
```

Bandwidth: 640×480 JPEG @ 80% ≈ 40–60 KB/frame × 10fps ≈ 400–600 KB/s over Tailscale.

---

## SE(2) EKF Localization

Dead-reckoning position estimation fused with ArUco marker corrections via an Extended
Kalman Filter. The filter state is `[x, y, θ]` in metres/radians relative to the
power-on origin (or the last `/localization/reset` call).

```
movement_routes.py  ──POST /localization/command──►  prediction loop (20 Hz)
                                                         │  kinematic model
                                                         ▼
video_server.py     ──POST /localization/aruco_update──► EKF update step
(ArUco detection)                                        │  marker map lookup
                                                         ▼
GET /localization/pose  ◄────────────────────────────  SE2PoseFilter.get_pose()
LocalizationPanel (UI)                                   x, y, θ, covariance, quality
```

### Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET`  | `/localization/pose` | Current pose estimate (x, y, θ, covariance, quality 0–1) |
| `POST` | `/localization/aruco_update` | Receive ArUco rvec/tvec from video server; apply EKF correction |
| `POST` | `/localization/command` | Update velocity hint used by the prediction loop |
| `POST` | `/localization/reset` | Reset filter to a known pose |
| `POST` | `/localization/marker_map` | Replace the ArUco marker world-position map at runtime |
| `GET`  | `/localization/status` | Filter diagnostics (hz, quality, correction count) |

### ArUco marker map

Define known marker world positions in a JSON file and set `ARUCO_MARKER_MAP_FILE`:

```json
{
  "7":  {"x": 1.0, "y": 0.0, "alpha": 3.14159},
  "12": {"x": 0.0, "y": 2.0, "alpha": 1.5708}
}
```

`alpha` is the direction the marker normal faces (radians, 0 = faces +X world axis).
Without a map the prediction loop still runs (dead-reckoning only); quality is capped at 0.5.

### Quality score

`quality` (0–1) returned by `GET /localization/pose` combines two factors:
- **covariance trace** — how uncertain is the geometric estimate (decays as dead-reckoning drifts)
- **staleness** — time since last ArUco correction (decays to 0 after 60 s without a fix)

A score ≥ 0.7 (green) means a recent ArUco correction with low covariance.
Dead-reckoning only is capped at 0.5 (amber) regardless of covariance.

### UI panel

`LocalizationPanel` polls `GET /localization/pose` every 2 s and shows:
- x / y position (metres) and heading θ (degrees)
- Quality badge (green / amber / red)
- Mini 2D top-down canvas: robot dot + heading arrow + 1-σ uncertainty ellipse
- ArUco correction count and last marker ID seen

### Video server integration

`video/video_server.py` posts ArUco detections with full rvec/tvec to
`POST /localization/aruco_update` in a fire-and-forget daemon thread so the camera
frame loop is never blocked. Requires `ARUCO_MARKER_LENGTH` (metres) and
`ARUCO_CALIBRATION_FILE` (camera intrinsics JSON) to be set in the video server
environment for pose estimation to produce metric results.

---

## Troubleshooting

### Camera not publishing

```bash
# Check topic exists and is streaming
ros2 topic list | grep camera
ros2 topic hz /omega/camera/image_raw

# Verify libcamerasrc GStreamer plugin is available
gst-inspect-1.0 libcamerasrc

# If gst-inspect fails, env vars are not set:
export LD_LIBRARY_PATH=$HOME/libcamera/build/src/libcamera:$LD_LIBRARY_PATH
export GST_PLUGIN_PATH=$HOME/libcamera/build/src/gstreamer:$GST_PLUGIN_PATH

# Check another process isn't holding the camera
fuser /dev/video0 /dev/media0 2>/dev/null
```

Expected log lines (not errors):
- `"libcamera is not installed"` — harmless, means local build is in use
- `"Cannot query video position"` — harmless GStreamer warning

Do NOT use `/dev/videoX` directly — V4L2 does not work with this CSI camera.

### Nodes on Pi not visible from laptop

1. `echo $ROS_DOMAIN_ID` — must be 42 on both machines
2. `echo $RMW_IMPLEMENTATION` — must be `rmw_cyclonedds_cpp`
3. Peer IPs in `cyclonedds_*.xml` must match actual addresses
4. `<AllowMulticast>false</AllowMulticast>` in CycloneDDS config if on a hotspot

### Camera feed not appearing in UI

```bash
# 1. Verify video server is running on Pi
curl http://omegaone:5000/health   # should return 200

# 2. Verify the proxy URL resolves from your dev machine
curl -I http://localhost:3002/api/video-proxy   # should start streaming

# 3. Check .env.local has correct URL
cat ui/robot-controller-ui/.env.local | grep VIDEO_STREAM

# 4. Restart Next.js dev server after .env.local changes
# (env changes are NOT hot-reloaded)
```

### Motors not responding

```bash
ros2 topic echo /omega/motor_state
# watchdog: true → no /cmd_vel in 500 ms

# If avoidance node is running, check it's passing commands through:
ros2 topic echo /omega/obstacle_avoidance/state

ros2 topic pub --once /cmd_vel_in geometry_msgs/Twist '{}'
```

### I2C / PCA9685 not found

```bash
i2cdetect -y 1   # PCA9685 at 0x40, ADS1115 at 0x48
sudo modprobe i2c-dev
```

### Xbox controller not detected

```bash
ls /dev/input/event*
sudo usermod -aG input $USER   # then log out and back in
pip install evdev
```

### `Package not found` after build

```bash
source /opt/ros/humble/setup.bash
source ~/omega_ws/install/setup.bash
```

### `ImportError: No module named 'servers'`

```bash
pip install -e ~/Desktop/Omega-Code/
```

---

## Known Issues

| Issue | Status |
|---|---|
| Front-left motor (left_upper) sometimes silent | ch0/ch1 wiring swap applied — run `motor_channel_scan.py` to verify channel mapping if still failing |
| Servo oscillating on power-on | Root cause: `motor_driver_pi.py` set PCA9685 to 500 Hz (DC motor smoothing), corrupting servo pulse widths to ~147 µs. Fixed: motor driver locked to 50 Hz; `Servo.__init__` reads PRESCALE on startup and auto-corrects; all pulse outputs hard-clamped to 1200–1800 µs |
| Pitch servo jitter at extremes | Hardware slack in gimbal; limits tightened to 1300–1700 µs |
| No wheel encoders | Dead-reckoning odometry only; large covariance in `/odom` until encoders added; SE(2) EKF partially compensates via ArUco marker corrections |
| Camera node not in omega_hybrid.launch.py | Camera is launched separately due to libcamera env var requirement |

---

## Roadmap

### Near-term
- **Detection node** — `omega_detection/detection_node.py` with YOLOv8 on Jetson Orin
- **Wheel encoders** — AS5600 magnetic → accurate `/odom` → unblock nav2

### Medium-term
- **nav2** — plug in once encoders available; TF tree already correct
- **slam_toolbox** — add RPLIDAR A1 for 2D maps

### Long-term
- **Behavior trees** — use existing action server scaffolding (`navigate_to_goal_action.py`, `obstacle_avoidance_action.py`)
- **Multi-robot** — `ros2_domain_bridge` for Pi ↔ Jetson domain isolation

---

## Architecture Decisions

| Decision | Rationale |
|---|---|
| libcamerasrc over V4L2 | V4L2 fails on this Pi's CSI OV5647 — select() timeouts, format errors |
| libcamera built from source, installed to /usr/local | Ubuntu apt package is incomplete on ARM64; build artifacts installed to system paths for systemd service compatibility |
| bcm2835_v4l2 blacklisted | Kernel module grabs camera device at boot and conflicts with libcamera |
| numpy<2 pinned | System apt OpenCV 4.5.4 and cv_bridge (ROS Humble) compiled against NumPy 1.x; NumPy 2.x breaks the C API |
| Flask MJPEG server (video_server.py) for UI feed | ROS camera node targets topic subscribers; Flask directly serves the browser without a bridge; lower latency, simpler |
| plain `<img>` not `next/image` in CameraFrame | Next.js Image component intercepts src → /_next/image optimizer, breaking multipart/x-mixed-replace MJPEG streams |
| /cmd_vel_in → avoidance_node → /cmd_vel mux | All teleop sources (web UI + Xbox) publish /cmd_vel_in; the avoidance node is the single arbiter of /cmd_vel to motor_controller, no changes needed to motor node |
| Camera node separate from main launch | libcamera requires env vars set before launch; isolating avoids polluting the main launch environment |
| omega_camera as its own package | Keeps camera pipeline self-contained; easier to iterate without rebuilding all of omega_robot |
| Detection on Jetson, not Pi | Pi 4B is CPU-only; YOLOv8n = ~2 FPS on Pi vs 40+ FPS on Jetson with TensorRT |
| Compressed image to Jetson | Raw bgr8 = ~9 MB/s; JPEG compressed = ~500 KB/s — required for Tailscale link |
| Keep Next.js UI | Browser-native WebSocket; no ROS on client |
| FastAPI embeds rclpy | Single entry point; no extra bridge process |
| Go lighting server stays | LED timing is critical; goroutines > rclpy timers |
| CycloneDDS unicast-only | Hotspot APs drop multicast; static peers is reliable |
| pip install -e . for backend | Cleaner than sys.path hacks; works in all envs |
| SystemHealthService as singleton | One `/api/health` probe loop for the entire frontend lifetime; components subscribe rather than each running their own polling loop — eliminates the N×polling amplification problem |
| Circuit breaker on all pollers | 3 consecutive failures → 60 s backoff; prevents log spam and 429s when the Pi is unreachable; success resets to normal interval automatically |
| ws:// normalized to http:// before fetch | `fetch()` rejects WS-scheme URLs at the browser level (silent TypeError); `toHealthUrl()` / `wsUrlToHttp()` coerce the scheme before any health probe |
| Simulated performance data removed | Fake random metrics make the dashboard look healthy when the Pi is offline; replaced with an explicit "backend offline" state so degradation is always visible |
| SE(2) EKF separated from `sensor_fusion.py` | Localization state is tightly coupled to ArUco observations and differential-drive kinematics; keeping it in a standalone `pose_ekf.py` + `localization_routes.py` avoids polluting the sensor fusion pipeline and makes the filter independently testable |
| PCA9685 clock locked to 50 Hz | All 16 channels share one global PWM clock; servos on ch8/ch9 require exactly 50 Hz (20 ms period). Setting any higher frequency for DC motors corrupts servo pulse widths. Motor driver and servo init both read back the PRESCALE register and abort/correct if the frequency drifted. |
| `SystemModeContext` polling consolidation | Multiple components independently polled `/api/system/mode/status` — O(N) requests per component mount. Context wraps the loop in a React context; consumers call `useSystemMode()` instead of spawning their own intervals. |
| Fire-and-forget ArUco posting from `video_server.py` | Flask video server runs in its own process; spawning a daemon thread for each ArUco observation keeps the MJPEG frame loop non-blocking while feeding the EKF localization endpoint with marker corrections. |
