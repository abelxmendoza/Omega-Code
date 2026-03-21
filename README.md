# Omega-1 — Hybrid ROS2 Robotics Platform

<p align="center">
  <img src="image/README/1719168223539.png" alt="Omega-1 Robot" width="480"/>
  <br/>
  <img src="image/README/1719064424761.png" alt="Omega-1 Logo" width="300"/>
</p>

Omega-1 is a modular autonomous robotics platform built around a Raspberry Pi 4B.
It runs a **hybrid architecture**: a Next.js web UI and FastAPI backend handle
operator interaction, while ROS2 (Humble) handles robot communication, sensor
integration, and autonomous operation.

The design philosophy is pragmatic — keep what works (web UI, Go services, MJPEG
stream), add ROS2 where it creates real value (motor control, sensors, navigation).

---

## System Architecture

```
[Next.js UI]  http/ws (unchanged)
     |
[FastAPI]  port 8000    <-- OmegaRosBridge embeds rclpy node here
     |  /cmd_vel (Twist)
     |  ROS2 DDS (CycloneDDS, no fixed port)
     +--------+--------+
              |
   +----------v----------+
   |    Raspberry Pi      |
   |  ROS2 Node Graph     |
   |                      |
   |  motor_controller  <-- /cmd_vel
   |       |               --> /odom
   |       |               --> /omega/motor_state
   |       v               --> /tf (odom -> base_link)
   |    PCA9685 (I2C)      |
   |    Motor HAL          |
   |                      |
   |  sensor_node          --> /omega/ultrasonic
   |  (HC-SR04, IR, ADC)   --> /omega/line_tracking/*
   |                       --> /omega/battery
   |                      |
   |  camera_publisher     --> /omega/camera/image_raw/compressed
   |  (Picamera2/V4L2)     --> /omega/camera/camera_info
   |                      |
   |  static_tf            --> base_link -> camera_link
   |                       --> base_link -> ultrasonic_front
   +---------------------+

Non-ROS (separate processes, managed by OmegaOS):
  Go lighting server    ws://pi:8082
  MJPEG video stream    http://pi:5000   (Flask)
  FastAPI REST/WS       http://pi:8000

Multi-machine DDS (CycloneDDS):
  Pi (omega1)    <--> Laptop (scythe)  <--> Jetson Orin (Tailscale)
```

### TF Tree
```
odom
 └── base_link          (dynamic: motor_controller_node @ 50 Hz)
      ├── camera_link   (static: 8 cm forward, 10 cm up)
      └── ultrasonic_front  (static: 12 cm forward)
```

### ROS Topic Map

| Topic | Type | Hz | Producer |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | on-demand | FastAPI bridge |
| `/odom` | `nav_msgs/Odometry` | 50 | motor_controller_node |
| `/omega/motor_state` | `omega_interfaces/MotorState` | 10 | motor_controller_node |
| `/omega/ultrasonic` | `sensor_msgs/Range` | 10 | sensor_node |
| `/omega/line_tracking/center` | `std_msgs/Bool` | 20 | sensor_node |
| `/omega/battery` | `sensor_msgs/BatteryState` | 1 | sensor_node |
| `/omega/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 30 | camera_publisher_node |
| `/omega/capabilities` | `std_msgs/String` (JSON) | 0.2 | system_capabilities |

---

## Repository Structure

```
Omega-Code/
├── setup.py                        # pip install -e . makes backend importable
├── ros/
│   ├── src/
│   │   ├── omega_robot/            # ROS node implementations
│   │   │   ├── omega_robot/
│   │   │   │   ├── motor_controller_node.py
│   │   │   │   ├── sensor_node.py
│   │   │   │   ├── camera_publisher_node.py
│   │   │   │   ├── system_capabilities.py
│   │   │   │   └── ... (action servers, path planner)
│   │   │   ├── package.xml
│   │   │   └── setup.py
│   │   ├── omega_bringup/          # Launch files + config (no executable code)
│   │   │   ├── launch/
│   │   │   │   ├── omega_hybrid.launch.py   # full stack on Pi
│   │   │   │   ├── omega_sim.launch.py      # laptop simulation
│   │   │   │   └── omega_minimal.launch.py  # motor + sensors only
│   │   │   └── config/
│   │   │       ├── cyclonedds_pi.xml
│   │   │       └── cyclonedds_laptop.xml
│   │   └── omega_interfaces/       # Custom message / service / action types
│   │       ├── msg/MotorState.msg
│   │       ├── msg/SensorBundle.msg
│   │       ├── srv/SetVelocity.srv
│   │       ├── action/DriveToGoal.action
│   │       ├── CMakeLists.txt
│   │       └── package.xml
│   └── launch/                     # Legacy launch files (kept for reference)
├── servers/
│   └── robot_controller_backend/
│       ├── movement/               # Motor HAL: PCA9685, PID, ramp, odometry
│       ├── sensors/                # HC-SR04, line tracking, ADC
│       ├── video/                  # Camera pipeline, MJPEG, vision modes
│       ├── api/
│       │   └── ros_bridge.py       # FastAPI <-> ROS bridge
│       ├── controllers/lighting/   # Go LED server
│       ├── omega_services/         # OmegaOS process orchestrator
│       └── main_api.py             # FastAPI entry point
├── ui/robot-controller-ui/         # Next.js 14 frontend
├── docker/ros2_robot/              # Docker setup for Pi deployment
└── scripts/                        # Setup and utility scripts
```

---

## Requirements

### All Machines
- ROS2 Humble Hawksbill
- Python 3.10+
- `rmw-cyclonedds-cpp` (`sudo apt install ros-humble-rmw-cyclonedds-cpp`)

### Raspberry Pi 4B
- Ubuntu 22.04 Server (64-bit)
- `sudo apt install python3-lgpio python3-picamera2`
- `pip install smbus2`
- Hardware: PCA9685 (I2C 0x40), HC-SR04 (GPIO 27/22), IR sensors (GPIO 14/15/23), ADS1115 (I2C 0x48)

### Development Laptop
- Ubuntu 22.04 Desktop
- `sudo apt install ros-humble-desktop`
- Node.js 18+ (for UI development)

### Jetson Orin Nano (optional)
- JetPack 5.x with ROS2 Humble
- CUDA 11.4+, PyTorch, Ultralytics YOLOv8
- Tailscale for VPN connectivity to the Pi

---

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/Omega-Code.git ~/Desktop/Omega-Code
```

### 2. Install Backend as Python Package

Run once per machine. This makes `servers.robot_controller_backend.*` importable
in ROS nodes without `sys.path` hacks:

```bash
pip install -e ~/Desktop/Omega-Code/

# Verify
python3 -c "from servers.robot_controller_backend.movement.minimal_motor_control import Motor; print('OK')"
```

### 3. Create the Colcon Workspace

```bash
mkdir -p ~/omega_ws/src
cd ~/omega_ws/src

ln -s ~/Desktop/Omega-Code/ros/src/omega_robot      omega_robot
ln -s ~/Desktop/Omega-Code/ros/src/omega_bringup    omega_bringup
ln -s ~/Desktop/Omega-Code/ros/src/omega_interfaces omega_interfaces
```

### 4. Install ROS Dependencies

```bash
source /opt/ros/humble/setup.bash
cd ~/omega_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build

```bash
cd ~/omega_ws

# omega_interfaces must build first (other packages depend on its messages)
colcon build --packages-select omega_interfaces
colcon build --packages-select omega_robot omega_bringup --symlink-install
```

`--symlink-install` means Python edits to node files are live immediately.

### 6. Configure Environment

Add to `~/.bashrc`:

```bash
# ROS2 Omega workspace
source /opt/ros/humble/setup.bash
source ~/omega_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Pi:     export CYCLONEDDS_URI=file://$HOME/Desktop/Omega-Code/ros/src/omega_bringup/config/cyclonedds_pi.xml
# Laptop: export CYCLONEDDS_URI=file://$HOME/Desktop/Omega-Code/ros/src/omega_bringup/config/cyclonedds_laptop.xml
```

Update the IP addresses in the CycloneDDS config files to match your network
before launching.

---

## Running the Robot

### Full Hybrid Stack (Pi, hardware mode)

```bash
# Terminal 1: ROS2 layer (motor, sensors, camera, TF)
ros2 launch omega_bringup omega_hybrid.launch.py

# Terminal 2: FastAPI backend (web UI bridge)
cd ~/Desktop/Omega-Code/servers/robot_controller_backend
python3 main_api.py

# Terminal 3: (optional) Go LED server
cd ~/Desktop/Omega-Code/servers/robot_controller_backend/controllers/lighting
./start_lighting_server.sh
```

Or start everything via OmegaOS:

```bash
cd ~/Desktop/Omega-Code/servers/robot_controller_backend
python3 -m omega_services.service_manager
```

### Motor + Sensors Only (Pi, minimal)

```bash
ros2 launch omega_bringup omega_minimal.launch.py
```

### From the Laptop — Observe and Control

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /omega/ultrasonic
ros2 topic echo /omega/motor_state
```

---

## Simulation Mode

No hardware required. Runs on any Ubuntu machine with ROS2 Humble.

```bash
ros2 launch omega_bringup omega_sim.launch.py
```

All hardware writes are no-ops. Sensors return synthetic data.
The camera publishes a scrolling test pattern.

---

## Manual Control

All velocity values are normalised to `[-1.0, 1.0]`:

```bash
# Drive forward at 50% speed
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Rotate left in-place
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.6}}'

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'

# Continuous stream at 10 Hz (until Ctrl+C)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.2}}'
```

---

## Testing

```bash
# Check node graph
ros2 node list
ros2 node info /omega_motor_controller

# Topic rates
ros2 topic hz /odom                               # ~50 Hz
ros2 topic hz /omega/camera/image_raw/compressed  # ~30 Hz
ros2 topic hz /omega/ultrasonic                   # ~10 Hz

# TF tree (creates frames.pdf)
ros2 run tf2_tools view_frames

# Run unit tests
cd ~/omega_ws
colcon test --packages-select omega_robot
colcon test-result --verbose
```

---

## Multi-Machine Setup

### Requirements
- Same `ROS_DOMAIN_ID` on all machines
- Same `RMW_IMPLEMENTATION` on all machines
- Correct peer IPs in CycloneDDS XML config
- All machines reachable (same LAN or Tailscale)

### Verify Cross-Machine Communication

```bash
# Pi: start nodes
ros2 launch omega_bringup omega_minimal.launch.py

# Laptop: confirm Pi's topics are visible
ros2 topic list        # should show /omega/ultrasonic, /odom, etc.
ros2 topic echo /omega/ultrasonic
```

### Jetson Orin via Tailscale

The Jetson connects via Tailscale VPN (`100.107.112.110`). This IP is already
in the CycloneDDS config files. No additional setup needed beyond:

```bash
# On Jetson:
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds_jetson.xml
ros2 run omega_robot orin_ai_brain
```

---

## Troubleshooting

### Nodes on Pi not visible from laptop
1. `echo $ROS_DOMAIN_ID` — must match on both machines
2. `echo $RMW_IMPLEMENTATION` — must be `rmw_cyclonedds_cpp`
3. Peer IPs in `cyclonedds_*.xml` must match actual network addresses
4. Disable multicast: `<AllowMulticast>false</AllowMulticast>` in CycloneDDS config

### Motors not responding
```bash
ros2 topic echo /omega/motor_state | python3 -c "import sys,json; [print(json.loads(l)['watchdog']) for l in sys.stdin]"
# watchdog state "triggered" means no /cmd_vel received in 500 ms
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'  # resets watchdog
```

### I2C / PCA9685 not found
```bash
i2cdetect -y 1  # PCA9685 should appear at 0x40
sudo modprobe i2c-dev
```

### Camera not publishing
```bash
ros2 topic hz /omega/camera/image_raw/compressed  # 0 Hz = not publishing
python3 -c "from picamera2 import Picamera2; c = Picamera2(); print('OK')"
v4l2-ctl --list-devices  # fallback V4L2
```

### `Package 'omega_robot' not found`
```bash
source /opt/ros/humble/setup.bash
source ~/omega_ws/install/setup.bash
```

### `ImportError: No module named 'servers'`
```bash
pip install -e ~/Desktop/Omega-Code/
```

### Build error: omega_interfaces messages not found
```bash
# Build interfaces first
colcon build --packages-select omega_interfaces
source ~/omega_ws/install/setup.bash
colcon build --packages-select omega_robot
```

---

## Overengineering to Remove

| Item | Problem | Action |
|---|---|---|
| `sensors/ultrasonic_ws_server.py` | Python duplicate of `main_ultrasonic.go` | Delete it |
| OmegaOS PID management | Reinvents systemd | Delegate to `systemctl`; keep web UI layer |
| 7 open ports | Firewall complexity, UI coupling | Route all WS through FastAPI port 8000 |
| PYTHONPATH in launch files | Fragile, breaks on path changes | Replaced by `pip install -e .` |

---

## Roadmap

### Near-term
- **Wheel encoders** — AS5600 magnetic encoders → accurate `/odom` → enable nav2
- **Consolidate ports** — all WebSocket traffic through port 8000 via FastAPI path routing

### Medium-term
- **nav2** — plug in once encoders + lidar are available; TF tree is already correct
- **slam_toolbox** — add RPLIDAR A1 (~$100), get 2D maps immediately

### Long-term
- **Jetson AI perception** — complete `orin_ai_brain.py` with YOLOv8 + `/omega/detections`
- **nav2 behavior trees** — use existing action server scaffolding (`navigate_to_goal_action.py`, `obstacle_avoidance_action.py`)
- **Multi-robot coordination** — `ros2_domain_bridge` for Pi ↔ Jetson isolation

---

## Architecture Decision Log

| Decision | Rationale |
|---|---|
| Keep Next.js UI | Browser-native WebSocket; no ROS on client |
| FastAPI embeds rclpy | Single entry point; no additional bridge process |
| Go lighting server stays | LED timing is critical; goroutines > rclpy timers |
| MJPEG separate from ROS camera | Avoids two-consumer camera conflicts on Pi |
| Dead-reckoning odom with large covariance | Honest uncertainty until encoders added |
| CycloneDDS unicast-only | Hotspot APs drop multicast; static peers is reliable |
| omega_bringup is launch-only | Nodes must not depend on their own launch config |
| pip install -e . for backend | Cleaner than sys.path hacks; works in all envs |
