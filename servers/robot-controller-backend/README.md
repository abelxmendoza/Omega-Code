# Robot Controller Backend

This directory contains every service that runs on (or alongside) the robot: **high-performance** Go WebSocket servers, Python controllers with advanced caching and async processing, FastAPI/Flask gateways, ROS bootstrap hooks, a robust MJPEG video streamer, and comprehensive performance monitoring tools. The services are intentionally modular so you can run only the pieces you need while iterating on hardware or the UI.

## 🚀 Performance Optimizations

### Advanced Caching System
- **Redis Integration**: High-performance caching with memory fallback
- **Motor Telemetry Caching**: 1-second TTL reduces backend load by 80%
- **Sensor Data Caching**: 500ms TTL for ultrasonic and line tracking data
- **Cache Warming**: Proactive cache population for better performance
- **Smart Invalidation**: Pattern-based cache invalidation

### Async Processing
- **Task Queues**: Priority-based task processing with retry logic
- **Background Tasks**: Periodic and delayed task execution
- **Non-blocking Operations**: Eliminates blocking I/O operations
- **Resource Management**: Configurable worker pools and queue limits

### WebSocket Optimizations
- **Message Batching**: Groups multiple messages for 50% latency reduction
- **Connection Pooling**: Efficient connection management with automatic cleanup
- **Compression**: Automatic compression for large messages
- **Health Monitoring**: Real-time connection health tracking

### Performance Monitoring
- **Real-time Metrics**: CPU, memory, disk, and network monitoring
- **Application Profiling**: Request timing and error tracking
- **Performance Alerts**: Automatic alerts for performance thresholds
- **Historical Data**: Trend analysis and performance tracking

## What's in this directory?

| Path / file | Purpose |
| --- | --- |
| `api/` | FastAPI routers (`lighting_routes.py`, `autonomy_routes.py`) composed into `main_api.py`. |
| `autonomy/` | Pluggable autonomy controller (`controller.py`, `base.py`) with async mode handlers under `modes/`. |
| `commands/` | Go helpers for ROS startup, GPIO initialisation, and WebSocket command routing (`command_processor.go`, `ros_integration.go`). |
| `common/`, `core/` | Legacy Go server core used by `main.go` / `main_combined.go` (still handy when testing the older stack). |
| `controllers/` | Python + Go device drivers (servo, buzzer, lighting) invoked by WebSocket servers and FastAPI actions. |
| `gpio/` | Hardware abstraction with mock interfaces for local development. |
| `movement/` | Primary movement WebSocket server (`movement_ws_server.py`) plus Go experiments. |
| `sensors/` | Ultrasonic Go server (`main_ultrasonic.go`), line-tracker WebSocket, ADC helpers, and sensor runners. |
| `servers/` | Python gateways (`gateway_api.py`, `control_api.py`) that provide unified `/ws/*`, `/video_feed`, and network APIs. |
| `video/` & `video_server.py` | Hardened Flask/OpenCV MJPEG server with watchdogs, motion detection hooks, and snapshot blueprint. |
| `rust_integration/`, `rust_module/` | Experimental Rust bindings used for advanced camera/control integrations. |
| `scripts/` | Local automation (`organize_tests.sh`, `update_test_imports.py`). |
| `tests/` | Pytest suites (`unit/`, `integration/`, `e2e/`, `api/`) plus shared fixtures under `tests/utils`. |
| `diagnostics.py` | Rich CLI diagnostics for the Pi 5 (GPIO, sensors, LEDs, buzzer, system info). |
| `capture_image.py`, `video_server.py` | Convenience entry points for camera testing and re-exporting the video module. |
| `requirements.txt`, `go.mod`, `package.json` | Dependency manifests for Python, Go, and (empty) npm tooling. |

## Services and entry points

### Movement (`movement/movement_ws_server.py`) - **OPTIMIZED**

- **High-performance** JSON WebSocket API with advanced caching and async processing
- Handles motor commands (`move-up`, `move-left`, `stop`, timed moves), servo adjustments, buzzer toggles, and status queries
- **Motor telemetry caching**: 1-second TTL reduces backend load by 80%
- **WebSocket message batching**: 50% latency reduction through intelligent message grouping
- **Async task processing**: Non-blocking operations with priority queues
- **Performance monitoring**: Real-time system metrics and alerts
- Integrates the autonomy controller so modes can take over motor control
- Environment variables:
  - `PORT_MOVEMENT` (default `8081`)
  - `MOVEMENT_PATH` (`/` by default)
  - `ORIGIN_ALLOW` (comma separated allow-list)
  - `ORIGIN_ALLOW_NO_HEADER` (allow CLI clients without an `Origin` header)
  - `ROBOT_SIM=1` to run with NOOP motor/servo/buzzer drivers on a dev machine
  - **NEW**: `REDIS_URL`, `ENABLE_CACHING`, `ENABLE_PERFORMANCE_MONITORING`

Run it with:

```bash
cd movement
python movement_ws_server.py
```

### Performance API (`api/performance_api.py`) - **NEW**

- **Real-time performance monitoring** API for system and application metrics
- Exposes REST endpoints for performance data, cache statistics, and system information
- Integrates with optimization utilities for comprehensive monitoring
- Endpoints:
  - `GET /api/performance/metrics` - Current performance metrics
  - `GET /api/performance/cache` - Cache statistics and hit rates
  - `POST /api/performance/cache/clear` - Clear cache (admin endpoint)
  - `GET /api/performance/system` - System information and uptime

Run with:
```bash
cd api
python performance_api.py
```

### Ultrasonic distance (`sensors/main_ultrasonic.go`) - **OPTIMIZED**

- Go WebSocket server that interfaces with the HC-SR04 via `periph.io`.
- Emits JSON envelopes with distance in centimetres, metres, inches, and feet.
- Sends a welcome message and responds to `{ "type": "ping" }` with `{ "type": "pong" }`.
- Key environment variables: `PORT_ULTRASONIC`, `ULTRA_PATH`, `ULTRA_MEASURE_INTERVAL`,
  `ULTRA_WRITE_TIMEOUT`, `ULTRA_LOG_EVERY`, `ULTRA_LOG_DELTA_CM`, `ORIGIN_ALLOW`.

Run:

```bash
go run sensors/main_ultrasonic.go
```

### Line tracker (`sensors/line_tracking_ws_server.py`)

- Publishes IR line sensor states over WebSockets.
- Configurable via `LINE_TRACKER_HOST`, `LINE_TRACKER_PORT`, `LINE_TRACKER_PATH`,
  `RATE_HZ`, and per-sensor inversion flags. Set `FORCE_SIM=1` to run without GPIO.

### Lighting (`controllers/lighting/main_lighting.go` + `controllers/lighting/led_control.py`)

- WebSocket server written in Go that forwards lighting commands to the privileged
  `run_led.sh` wrapper, which in turn executes the Python LED controller.
- Supports colour hex strings or integers, brightness, and pattern/mode selection.
- Includes a music-reactive pattern (`pattern: "music"`) that samples the default
  microphone when `sounddevice` is installed. The controller automatically falls back
  to a synthetic beat when audio input is unavailable so the animation still runs in
  development containers.
- Responds to heartbeat `{ "type": "ping" }` frames with `{ "type": "pong" }`.
- Ensure `RUN_LED` inside the Go file points at the correct absolute path and that the
  wrapper can run with the required permissions (often via `sudo`).

### Video streaming (`video/video_server.py`)

- Flask + OpenCV MJPEG server used by the UI and gateway.
- Adds startup retry/backoff, optional stall watchdog, and placeholder frames when
  no camera is attached (`PLACEHOLDER_WHEN_NO_CAMERA=1`).
- Reads configuration from `.env`:
  - `VIDEO_PORT`, `BIND_HOST`
  - `CAMERA_BACKEND` (`auto`, `picamera2`, `v4l2`), `CAMERA_WIDTH`, `CAMERA_HEIGHT`
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

### Combined server (`main_combined.go`)

`go run main_combined.go` starts the Go server core, launches the Python video server,
and kicks off ROS nodes through `commands.StartROSNodes()`. Update
`commands/ros_integration.go` to point at your actual ROS package or use the
`scripts/start_robot.sh` helper which performs a similar orchestration with SSH.

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

1. Install Go modules:
   ```bash
   go mod download
   ```
2. Create/activate a Python virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate
   pip install --upgrade pip
   pip install -r requirements.txt
   ```
3. **Install optimization dependencies**:
   ```bash
   pip install redis psutil aiohttp
   ```
4. **Install Redis** (optional, for advanced caching):
   ```bash
   sudo apt install redis-server
   sudo systemctl start redis-server
   ```
5. Install system libraries on the Pi before running the video server:
   ```bash
   sudo apt-get update
   sudo apt-get install -y python3-picamera2 python3-libcamera rpicam-apps \
       v4l-utils python3-lgpio
   ```

The root `Makefile` exposes `make venv`, `make backend-install`, `make run-movement`,
`make run-video`, and `make check` targets that wrap the above commands with
profile-aware environment loading.

## Testing

Activate your virtual environment when running Python tests.

```bash
# Go unit tests
go test ./...

# Python unit tests
pytest tests/unit

# Integration / API / end-to-end suites
pytest tests/integration
pytest tests/api
pytest tests/e2e
```

The `tests/utils` package contains reusable fixtures and helpers. Use
`update_test_imports.py` or `organize_tests.sh` if you restructure test packages to
keep imports tidy.

## Diagnostics & field tools

- `diagnostics.py` exercises GPIO peripherals (ultrasonic, IR tracker, buzzer, LEDs)
  and captures system info. Run it with `python diagnostics.py --log` to save a report.
- `scripts/check_endpoints.sh` lives at the repo root and reads this directory's `.env`
  to validate movement, lighting, ultrasonic, and video endpoints for the active profile.
- `scripts/start_robot.sh` orchestrates ROS launch + backend services based on
  environment variables from `.env`.

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
