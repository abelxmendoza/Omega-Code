# Robot Controller Backend

Python services that run on (or alongside) the robot: WebSocket servers, FastAPI/Flask gateways, a ROS2 bridge, a GStreamer MJPEG video streamer, and hardware controllers. Services are modular — run only what you need.

## What's in this directory?

| Path / file | Purpose |
| --- | --- |
| `api/` | FastAPI routers (`lighting_routes.py`, `autonomy_routes.py`, `ros_routes.py`, `sensor_bridge.py`) composed into `main_api.py`. |
| `autonomy/` | Pluggable autonomy controller (`controller.py`, `base.py`) with async mode handlers under `modes/`. |
| `controllers/` | Python device drivers (servo, buzzer, lighting) invoked by WebSocket servers and FastAPI actions. |
| `gpio/` | Hardware abstraction with mock interfaces for local development. |
| `movement/` | Movement WebSocket server (`movement_ws_server.py`). |
| `sensors/` | Ultrasonic and line-tracker WebSocket servers (`ultrasonic_ws_server.py`, `line_tracking_ws_server.py`), ADC helpers. |
| `servers/` | Python gateways (`gateway_api.py`, `control_api.py`) that provide unified `/ws/*`, `/video_feed`, and network APIs. |
| `video/` | Hardened Flask/OpenCV MJPEG server with watchdogs, GStreamer/libcamera backend, motion detection, and snapshot blueprint. |
| `hardware/` | Camera drivers (`camera_drivers.py`), GPIO optimisation, and hardware detection. |
| `network/` | Network management (Tailscale, AP mode, mDNS). |
| `omega_config/` | YAML config (`config.yaml`, `robot_profile.yaml`) and `config_manager.py`. |
| `scripts/` | Local automation (`organize_tests.sh`, `update_test_imports.py`). |
| `tests/` | Pytest suites (`unit/`, `integration/`, `e2e/`, `api/`) plus shared fixtures under `tests/utils`. |
| `requirements.txt` | Python dependency manifest. |

## Services and entry points

### Movement (`movement/movement_ws_server.py`)

- JSON WebSocket API for motor commands (`move-up`, `move-left`, `stop`, timed moves), servo adjustments, buzzer, and status queries.
- Integrates the autonomy controller so autonomous modes can take over motor control.
- Optional Redis caching (`REDIS_URL`, `ENABLE_CACHING`) and performance monitoring (`ENABLE_PERFORMANCE_MONITORING`).
- Environment variables:
  - `PORT_MOVEMENT` (default `8081`)
  - `MOVEMENT_PATH` (`/` by default)
  - `ORIGIN_ALLOW` (comma-separated allow-list)
  - `ORIGIN_ALLOW_NO_HEADER` (allow CLI clients without an `Origin` header)
  - `ROBOT_SIM=1` — NOOP motor/servo/buzzer drivers for dev machines

```bash
python movement/movement_ws_server.py
```

### Performance API (`api/performance_api.py`)

REST endpoints for live system metrics, cache stats, and uptime. Mounted into `main_api.py`.

| Endpoint | Purpose |
|----------|---------|
| `GET /api/performance/metrics` | CPU, memory, request timing |
| `GET /api/performance/cache` | Cache hit rates |
| `GET /api/performance/system` | System info and uptime |

### Ultrasonic distance (`sensors/ultrasonic_ws_server.py`)

- Python WebSocket server that interfaces with the HC-SR04.
- Emits JSON envelopes with distance in centimetres.
- Sends a welcome message and responds to `{ "type": "ping" }` with `{ "type": "pong" }`.
- Key environment variables: `PORT_ULTRASONIC`, `ULTRA_PATH`, `ORIGIN_ALLOW`.

Run:

```bash
python sensors/ultrasonic_ws_server.py
```

### Line tracker (`sensors/line_tracking_ws_server.py`)

- Publishes IR line sensor states over WebSockets.
- Configurable via `LINE_TRACKER_HOST`, `LINE_TRACKER_PORT`, `LINE_TRACKER_PATH`,
  `RATE_HZ`, and per-sensor inversion flags. Set `FORCE_SIM=1` to run without GPIO.

### Lighting (`controllers/lighting/led_control.py`)

- Python LED controller that supports colour hex strings, brightness, and pattern/mode selection.
- Includes a music-reactive pattern (`pattern: "music"`) that samples the default
  microphone when `sounddevice` is installed. Falls back to a synthetic beat automatically.
- Exposed through the FastAPI `/lighting` routes (no separate WebSocket server needed).
- Ensure `run_led.sh` wrapper has the required permissions (often via `sudo`).

### Video streaming (`video/video_server.py`)

- Flask + OpenCV MJPEG server used by the UI and gateway.
- Adds startup retry/backoff, optional stall watchdog, and placeholder frames when
  no camera is attached (`PLACEHOLDER_WHEN_NO_CAMERA=1`).
- Reads configuration from `.env`:
  - `VIDEO_PORT`, `BIND_HOST`
  - `CAMERA_BACKEND` (`gstreamer`, `auto`, `v4l2`), `CAMERA_WIDTH`, `CAMERA_HEIGHT`, `CAMERA_FPS`
  - `STARTUP_RETRY_SEC`, `RESTART_ON_STALL`, `STALE_MS`, `WATCHDOG_PERIOD_MS`
  - `ORIGIN_ALLOW`, `PUBLIC_BASE_URL(_ALT)` for logging, `CERT_PATH`, `KEY_PATH`
  - `FACE_RECOGNITION`, `KNOWN_FACES_DIR`, `FACE_RECOGNITION_THRESHOLD`
  - `ARUCO_DETECTION`, `ARUCO_DICTIONARY`, `ARUCO_MARKER_LENGTH`, `ARUCO_CALIBRATION_FILE`, `ARUCO_DRAW_AXES`
- Registers the control blueprint (`servers/control_api.py`) so `/snapshot` can
  capture or save still frames.

Start it with:

```bash
python video/video_server.py
```

### REST API (`main_api.py`)

- Minimal FastAPI application that includes routers from `api/`.
- `/lighting` endpoints currently shell out to the Python LED controller.
- `/autonomy/*` endpoints map onto the asynchronous controller in `autonomy/` and
  return structured status envelopes. See `api/autonomy_routes.py` for request bodies.

Run via Uvicorn:

```bash
uvicorn main_api:app --host 0.0.0.0 --port 8000 --reload
```

### Gateway proxy (`servers/gateway_api.py`)

- All-in-one FastAPI service that surfaces `/ws/movement`, `/ws/lighting`, `/ws/ultrasonic`,
  `/ws/line`, `/video_feed`, `/health`, and `/api/net/*` endpoints expected by the UI.
- Proxies WebSocket traffic to downstream services when `DS_MOVE`, `DS_LIGHT`,
  `DS_ULTRA`, or `DS_LINE` are set (falls back to echo/mock behaviour otherwise).
- `VIDEO_UPSTREAM` configures the upstream MJPEG stream that `/video_feed` should proxy.
- Additional env hooks let you customise status responses: `LINK_TYPE`, `SSID`, etc.

Launch with:

```bash
uvicorn servers.gateway_api:app --host 0.0.0.0 --port 7070
```

## Environment configuration

Copy `.env.example` to `.env` and adjust to match your deployment:

- **Addressing & ports**: `PI_IP`, `TAILSCALE_IP_PI`, `BIND_HOST`, `PORT_MOVEMENT`,
  `PORT_ULTRASONIC`, `PORT_LIGHTING`, `PORT_LINE_TRACKER`, `PORT_LOCATION`, `VIDEO_PORT`.
- **TLS / origins**: `CERT_PATH`, `KEY_PATH`, `ORIGIN_ALLOW`, `PUBLIC_BASE_URL`,
  `PUBLIC_BASE_URL_ALT`.
- **Hardware toggles**: `USE_RPI`, `USE_JETSON_NANO`, `CAMERA_BACKEND`,
  `PLACEHOLDER_WHEN_NO_CAMERA`, `AUTO_PLACEHOLDER_AFTER_MISSES`.
- **Movement tweaks**: `MOVEMENT_PATH`, `ROBOT_SIM`, `ORIGIN_ALLOW_NO_HEADER`.
- **Ultrasonic**: `ULTRA_PATH`, `ULTRA_MEASURE_INTERVAL`, `ULTRA_WRITE_TIMEOUT`,
  `ULTRA_LOG_EVERY`, `ULTRA_LOG_DELTA_CM`.
- **Video watchdog**: `STARTUP_RETRY_SEC`, `RESTART_ON_STALL`, `STALE_MS`,
  `WATCHDOG_PERIOD_MS`.
- **Gateway proxy**: `DS_MOVE`, `DS_LIGHT`, `DS_ULTRA`, `DS_LINE`, `VIDEO_UPSTREAM`,
  `LINK_TYPE`, `SSID`, `TAILSCALE_IP_PI` (for mock responses).

Environment variables are loaded via `python-dotenv` where applicable, so editing the
`.env` file and restarting the service is usually sufficient.

## Setup & dependencies

1. Create/activate a Python virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate
   pip install --upgrade pip
   pip install -r requirements.txt
   ```
2. Install system libraries on the Pi before running the video server:
   ```bash
   sudo apt-get update
   sudo apt-get install -y python3-libcamera rpicam-apps v4l-utils python3-lgpio \
       gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
   # picamera2 as fallback: sudo apt install python3-picamera2
   ```
3. **Install Redis** (optional, for advanced caching):
   ```bash
   sudo apt install redis-server
   sudo systemctl start redis-server
   ```

The root `Makefile` exposes `make venv`, `make backend-install`, `make run-movement`,
`make run-video`, and `make check` targets that wrap the above commands with
profile-aware environment loading.

## Testing

Activate your virtual environment before running tests.

```bash
pytest tests/unit          # Unit tests — hardware mocked, no network
pytest tests/integration   # Integration — requires running services
pytest tests/e2e           # End-to-end workflows
pytest tests/system        # Full system smoke tests
pytest tests/security      # API input validation and auth
pytest tests/performance   # Latency and load benchmarks
pytest tests/regression    # Regression suite
pytest tests/hybrid        # Mode-switching and hybrid stack tests
pytest tests/faults        # Fault injection and recovery
```

| Suite | Files | What it covers |
|-------|-------|----------------|
| `unit/` | 25 | Autonomy modes, LED control, video, servo, omega_config, movement |
| `integration/` | 8+ | API endpoints, video server, movement WebSocket smoke test |
| `e2e/` | 5 | Full workflow: ROS2 navigation, line follow, system modes |
| `system/` | 2 | OmegaOS system-level checks |
| `security/` | 2 | API auth, input validation |
| `performance/` | 3 | Latency, load, profiling |
| `regression/` | 2 | OmegaOS regression |
| `hybrid/` | 3 | Mode switching, WebSocket state, thermal transitions |
| `faults/` | 4 | Frame drop recovery, Pi/Orin failures, ROS bridge delay |

`tests/conftest.py` mocks all Pi hardware (`rpi_ws281x`, `PCA9685`, `lgpio`) so every
suite runs on a dev machine without GPIO. `tests/utils/` holds shared fixtures.

Use `scripts/organize_tests.sh` or `scripts/update_test_imports.py` if you restructure packages.

## Diagnostics & field tools

- `tools/diagnostics.py` — rich CLI diagnostics for GPIO peripherals (ultrasonic, IR tracker, buzzer, LEDs). Run with `python tools/diagnostics.py --log` to save a report.
- `tools/websocket_test_client.py` — interactive WebSocket REPL for testing any endpoint.
- `scripts/check_endpoints.sh` — reads this directory's `.env` and validates movement, lighting, ultrasonic, and video endpoints.
- `scripts/start_robot.sh` — orchestrates ROS2 launch + backend services from `.env`.

## Hardware notes

- Grant GPIO access on the Pi 5 before running the servers:
  ```bash
  sudo chown root:gpio /dev/gpiochip0
  sudo chmod g+rw /dev/gpiochip0
  ```
- The lighting controller expects `run_led.sh` to run with elevated privileges so it
  can interact with the LED strip. Audit and adjust that script before deploying.
- `controllers/buzzer.py` and related servo helpers initialise hardware on import;
  catch exceptions and fall back to simulation mode (`ROBOT_SIM=1`) when developing on
  a laptop.

## License

This backend is covered by the repository's MIT license.
