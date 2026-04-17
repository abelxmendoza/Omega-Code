# Robot Controller Backend

Python services that run on (or alongside) the robot: a FastAPI REST/WebSocket gateway, a ROS2 bridge, a GStreamer MJPEG video streamer, hardware controllers, and an autonomy engine. Services are modular — run only what you need.

## Directory structure

| Path | Purpose |
| --- | --- |
| `api/` | FastAPI routers composed into `main_api.py`: `movement_routes`, `sensor_ws_routes` (incl. `/ws/radar`), `lighting_routes`, `autonomy_routes`, `ros_routes`, `localization_routes` (SE(2) EKF), `sim_routes` (SIM_MODE=1 only), `capability_routes`, `system_mode_routes`, `service_routes`, `config_routes`, plus `sensor_bridge.py`, `ros_bridge.py`, `security_middleware.py`, `error_handlers.py`, `input_validators.py`. |
| `simulation/` | Software simulation engine (`sim_engine.py`): differential-drive kinematics + geometrically correct ArUco rvec/tvec synthesis. Feeds the real SE(2) EKF without any hardware. Activated via `SIM_MODE=1`. |
| `autonomy/` | Pluggable autonomy controller (`controller.py`, `base.py`) with async mode handlers under `modes/`. |
| `controllers/` | Python device drivers (servo, buzzer, lighting) invoked by WebSocket servers and FastAPI actions. |
| `gpio/` | Hardware abstraction with mock interfaces for local development. |
| `movement/` | Motor HAL: PCA9685 driver, ramp, PID, odometry, watchdog, and the movement WebSocket server (`movement_ws_server.py`). `pose_ekf.py` — SE(2) EKF. `hardware/servo_control.py` — PCA9685 servo driver. |
| `sensors/` | Ultrasonic and line-tracker WebSocket servers (`ultrasonic_ws_server.py`, `line_tracking_ws_server.py`), ADC helpers. |
| `servers/` | Gateway (`gateway_api.py`) that surfaces `/ws/*`, `/video_feed`, and `/api/net/*` endpoints expected by the UI; snapshot blueprint (`control_api.py`). |
| `video/` | Flask/OpenCV MJPEG server with startup retry, stall watchdog, face recognition, ArUco detection, motion detection, and snapshot blueprint. |
| `hardware/` | Camera drivers, GPIO optimisation, hardware detection, motor and LED control helpers. |
| `network/` | Network management: Wi-Fi scan, AP mode, Tailscale VPN, mDNS, network watchdog service. |
| `omega_config/` | YAML config (`config.yaml`, `hardware_map.yaml`, `robot_profile.yaml`) and `config_manager.py`. |
| `omega_services/` | Process supervisor, service manager, health checks, and `omega-orchestrator.service` for systemd. |
| `utils/` | Env helpers, LED utility, threading control, optimisation utilities. |
| `scripts/` | Shell helpers: `start_sim_local.sh` (SIM_MODE=1), `start_all.sh`, `stop_all.sh`, `check_endpoints.sh`. |
| `tests/` | Pytest suites (`unit/`, `integration/`, `e2e/`, `system/`, `security/`, `performance/`, `regression/`, `hybrid/`, `faults/`) plus shared fixtures in `tests/utils/`. |
| `requirements.txt` | All Python dependencies — `pip install -r requirements.txt`. |

## Services and entry points

### FastAPI gateway (`main_api.py`)

FastAPI application that mounts all routers with a layered security middleware stack (CORS, rate limiting, request size limits, optional API-key auth and CSRF protection). Configuration loaded from `omega_config/config.yaml` with env var fallback.

Start with Uvicorn:

```bash
uvicorn main_api:app --host 0.0.0.0 --port 8000 --reload
```

Routes registered:

| Router | Prefix / tags | What it exposes |
|--------|---------------|-----------------|
| `movement_routes` | `Movement` | `GET /status`, `POST /move`, `POST /servo`, `POST /buzzer` |
| `sensor_ws_routes` | `Sensors` | `WS /ws/ultrasonic`, `WS /ws/line`, `WS /ws/battery`, `WS /ws/radar` (pan-servo sweep) |
| `localization_routes` | `Localization` | `GET /localization/pose`, `POST /localization/aruco_update`, `/reset`, `/command`, `/marker_map`, `/status` |
| `sim_routes` | `Simulation` | `POST /sim/start`, `/stop`, `/velocity`, `/teleport`, `/load_scenario`; `GET /sim/state`, `/scenarios`; `WS /sim/ws` — **only mounted when `SIM_MODE=1`** |
| `lighting_routes` | `/lighting` | Colour, pattern, mode, and brightness control |
| `autonomy_routes` | `Autonomy` | Start/stop/status for pluggable autonomy modes |
| `ros_routes` | `ROS` | Topic list, publish, subscribe, and bridge status |
| `capability_routes` | `Capabilities` | Feature detection and capability matrix |
| `system_mode_routes` | `System` | System mode switching (`normal`, `performance`, `eco`, …) |
| `service_routes` | `Services` | List, start, stop, and restart managed services |
| `config_routes` | `Config` | Read and write `omega_config` sections over REST |
| `network_routes` | `Network` | Wi-Fi scan, AP mode, Tailscale, and network status |
| `performance_api` | `/api/performance` | CPU, memory, cache, and uptime metrics |

### ROS bridge (`api/ros_bridge.py`)

`OmegaRosBridge` embeds an `rclpy` node in the FastAPI process. When ROS2 is present it publishes `/cmd_vel_in` (Twist) for movement commands and `/omega/servo_increment` (Vector3) for pan-tilt. Gracefully degrades to a no-op stub when ROS2 is not available (`bridge.is_active = False`).

### Sensor WebSocket bridge (`api/sensor_bridge.py` + `api/sensor_ws_routes.py`)

`OmegaSensorBridge` subscribes to `/omega/ultrasonic`, `/omega/line_tracking/*`, and `/omega/battery` and fans out JSON frames to all connected WebSocket clients. The three WebSocket endpoints are:

| Endpoint | Data |
|----------|------|
| `WS /ws/ultrasonic` | `{ "distance_cm": float }` |
| `WS /ws/line` | `{ "left": bool, "center": bool, "right": bool }` |
| `WS /ws/battery` | `{ "voltage": float, "percentage": float }` |

### Movement WebSocket server (`movement/movement_ws_server.py`)

Standalone WebSocket server for motor commands. Integrates the autonomy controller so autonomous modes can take over motor control. Optional Redis caching and performance monitoring.

```bash
python movement/movement_ws_server.py
```

Key environment variables:

| Variable | Default | Purpose |
|----------|---------|---------|
| `PORT_MOVEMENT` | `8081` | Listen port |
| `MOVEMENT_PATH` | `/` | WebSocket path |
| `ORIGIN_ALLOW` | — | Comma-separated origin allow-list |
| `ORIGIN_ALLOW_NO_HEADER` | — | Allow CLI clients without `Origin` |
| `ROBOT_SIM` | `0` | `1` = NOOP motor/servo/buzzer drivers |

### Ultrasonic server (`sensors/ultrasonic_ws_server.py`)

Standalone Python WebSocket server for the HC-SR04. Emits `{ "distance_cm": float }` envelopes, responds to `{ "type": "ping" }` with `{ "type": "pong" }`.

```bash
python sensors/ultrasonic_ws_server.py
```

### Line tracker (`sensors/line_tracking_ws_server.py`)

Publishes IR line sensor states over WebSocket. Set `FORCE_SIM=1` to run without GPIO.

### Video streaming (`video/video_server.py`)

Flask + OpenCV MJPEG server. Startup retry/backoff, stall watchdog, placeholder frames when no camera is attached, optional face recognition and ArUco marker detection.

```bash
python video/video_server.py
# Stream:  http://omegaone:5000/video_feed
# Health:  http://omegaone:5000/health
```

Key environment variables:

| Variable | Default | Purpose |
|----------|---------|---------|
| `VIDEO_PORT` | `5000` | HTTP port |
| `CAMERA_BACKEND` | `auto` | `gstreamer`, `auto`, or `v4l2` |
| `PLACEHOLDER_WHEN_NO_CAMERA` | `0` | Serve a placeholder frame if no camera |
| `RESTART_ON_STALL` | `1` | Restart camera pipeline on stall |
| `STALE_MS` | `3000` | Frame age (ms) before stall is declared |
| `FACE_RECOGNITION` | `0` | Enable face recognition overlay |
| `ARUCO_DETECTION` | `0` | Enable ArUco marker detection |

### Gateway proxy (`servers/gateway_api.py`)

All-in-one FastAPI service that surfaces `/ws/movement`, `/ws/lighting`, `/ws/ultrasonic`, `/ws/line`, `/video_feed`, `/health`, and `/api/net/*` in a single process. Proxies to downstream standalone servers when `DS_MOVE`, `DS_LIGHT`, `DS_ULTRA`, or `DS_LINE` are set.

```bash
uvicorn servers.gateway_api:app --host 0.0.0.0 --port 7070
```

## Simulation mode

The `simulation/sim_engine.py` module implements a software-simulated robot. It is
activated by setting `SIM_MODE=1` before starting the API server. All simulation routes
are mounted automatically; no code changes are needed.

```bash
# Start the sim stack (no hardware required)
SIM_MODE=1 venv/bin/uvicorn main_api:app --host 0.0.0.0 --port 8000
# or use the convenience script:
bash scripts/start_sim_local.sh
```

The sim engine:
- Runs differential-drive kinematics at ~50 Hz, updating the robot's world pose
- Synthesises ArUco `rvec`/`tvec` from the geometric relationship between the robot and
  any markers defined in the ArUco marker map
- Posts those observations to `POST /localization/aruco_update` exactly as
  `video_server.py` does from a real camera
- The SE(2) EKF downstream receives and fuses them with no modification

All sim control routes are at `/sim/*` (only available when `SIM_MODE=1`). The real
`/localization/*`, `/ws/*`, and movement endpoints behave identically to hardware mode.

## Environment configuration

Copy `.env.example` to `.env` and adjust for your deployment. Key groups:

- **Addressing**: `PI_IP`, `TAILSCALE_IP_PI`, `BIND_HOST`, `PORT_MOVEMENT`, `PORT_ULTRASONIC`, `PORT_LIGHTING`, `PORT_LINE_TRACKER`, `VIDEO_PORT`
- **TLS / origins**: `CERT_PATH`, `KEY_PATH`, `ORIGIN_ALLOW`, `PUBLIC_BASE_URL`
- **Hardware**: `USE_RPI`, `CAMERA_BACKEND`, `PLACEHOLDER_WHEN_NO_CAMERA`, `ROBOT_SIM`
- **Security**: `API_AUTH_ENABLED`, `API_KEY`, `RATE_LIMIT_ENABLED`, `REQUESTS_PER_MINUTE`, `CSRF_ENABLED`, `AUDIT_LOGGING`
- **Gateway proxy**: `DS_MOVE`, `DS_LIGHT`, `DS_ULTRA`, `DS_LINE`, `VIDEO_UPSTREAM`

## Setup

```bash
python -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

System libraries required on the Pi before running the video server:

```bash
sudo apt-get install -y python3-libcamera rpicam-apps v4l-utils python3-lgpio \
    gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

The root `Makefile` exposes `make venv`, `make backend-install`, `make run-movement`, `make run-video`, and `make check` targets that wrap the above with profile-aware environment loading.

## Testing

Activate your virtual environment, then run any combination of suites:

```bash
pytest tests/unit            # Unit — hardware mocked, no network
pytest tests/integration     # Integration — API endpoints, WebSocket smoke tests
pytest tests/e2e             # End-to-end workflows
pytest tests/system          # OmegaOS system-level checks
pytest tests/security        # API input validation and auth
pytest tests/performance     # Latency and load benchmarks
pytest tests/regression      # Regression suite
pytest tests/hybrid          # Mode-switching and hybrid stack tests
pytest tests/faults          # Fault injection and recovery
```

| Suite | Files | What it covers |
|-------|-------|----------------|
| `unit/` | 24 | Autonomy routes, movement routes, sensor WS routes, system mode routes, autonomy controller, LED control, video server, servo, command processor, input validators, config manager, service manager, advanced PID, sensor fusion |
| `integration/` | 10 | API config, lighting, ROS routes, config endpoints, movement WebSocket, video server, service config, system mode, latency |
| `e2e/` | 5 | Full workflow: ROS2 navigation, line follow, system modes, complete stack |
| `system/` | 1 | OmegaOS system-level checks |
| `security/` | 2 | API auth, input validation |
| `performance/` | 3 | Latency, load, profiling |
| `regression/` | 1 | OmegaOS regression |
| `hybrid/` | 3 | Mode switching, WebSocket state, thermal transitions |
| `faults/` | 4 | Frame drop recovery, Pi/Orin failures, ROS bridge delay |

`tests/conftest.py` mocks all Pi hardware (`rpi_ws281x`, `PCA9685`, `lgpio`) so every suite runs on a dev machine without GPIO. `tests/utils/` holds shared fixtures.

## Diagnostics and field tools

- `tools/diagnostics.py` — rich CLI diagnostics for GPIO peripherals. Run with `--log` to save a report.
- `tools/websocket_test_client.py` — interactive WebSocket REPL for testing any endpoint.
- `scripts/check_endpoints.sh` — validates movement, lighting, ultrasonic, and video endpoints.
- `scripts/start_all.sh` / `scripts/stop_all.sh` — orchestrate all backend services.

## Hardware notes

- Grant GPIO access on the Pi before running the servers:
  ```bash
  sudo chown root:gpio /dev/gpiochip0
  sudo chmod g+rw /dev/gpiochip0
  ```
- The lighting controller expects `run_led.sh` to run with elevated privileges. Audit and adjust that script before deploying.
- `controllers/buzzer.py` and servo helpers initialise hardware on import; set `ROBOT_SIM=1` when developing on a laptop.

## License

This backend is covered by the repository's MIT license.
