# Robot Controller Backend

This directory contains the backend services for Omega-Code. The stack combines a Go 1.22
WebSocket server with Python helpers, FastAPI REST endpoints, ROS node orchestration, and a
Python MJPEG video streamer. Together they expose movement, lighting, sensor, and autonomy
capabilities to the frontend UI and hardware controllers running on the Raspberry Pi.

## Capabilities

- WebSocket control channels for driving, ultrasonic sensing, lighting, line tracking, and
  speed adjustments (`/ws/*`).
- PCA9685 motor/servo management and GPIO sensor drivers (backed by `lgpio` on the Pi 5).
- Optional FastAPI gateway (`main_api.py`) that proxies the WebSocket services and exposes
  REST endpoints for Wi-Fi/Bluetooth management and diagnostics.
- Python MJPEG video server (`video/video_server.py`) with libcamera/picamera2 backends and
  a health-check API used by the frontend.
- ROS bootstrap hooks (`commands/ros_integration.go`) used by `main_combined.go` to start
  autonomy services alongside the Go server.

## Directory map

| Path | Description |
| --- | --- |
| `api/` | FastAPI routers grouped by domain (movement, lighting, sensors, autonomy). |
| `commands/` | Go command handlers that translate WebSocket payloads into hardware calls. |
| `controllers/` | Python classes for servo, LED, and line tracking control. |
| `gpio/` | Hardware abstraction layer with mocks for local development. |
| `movement/`, `sensors/` | Python modules that speak to the PCA9685, ultrasonic sensors, etc. |
| `video/` | Streaming server and helpers around libcamera/picamera2. |
| `utils/` | Cross-cutting helpers (threading, camera selection, logging). |
| `tests/` | Unit, integration, and end-to-end suites for both Go and Python components. |
| `scripts/` | Local utilities for reorganising tests and updating imports. |

Large third-party dependencies (libcamera, mojo tooling) live under `libcamera/` and
`rust_module/` to support advanced camera paths. They are only needed on the Pi.

## Requirements

Install these tools on the machine that will run the backend:

- Go 1.22+
- Python 3.10+ (3.11 recommended) with `pip`
- `libcamera`/`picamera2`, `lgpio`, and other Pi-specific packages when running on hardware
- (Optional) ROS 2 Humble or later for the autonomy hooks

When developing locally without hardware you can enable the mock GPIO implementation by
setting `USE_RPI=false` in the environment.

## Environment configuration

Copy the template and adjust it for your deployment:

```bash
cp .env.example .env
```

Key variables from `.env`:

| Variable | Purpose |
| --- | --- |
| `PI_IP` | Address the Go server binds to (also used by helpers to reach the Pi). |
| `BIND_HOST` | Hostname for FastAPI and video services (`0.0.0.0` by default). |
| `PORT_*` | Individual WebSocket service ports (movement, ultrasonic, lighting, etc.). |
| `VIDEO_PORT` | Port for the MJPEG stream. |
| `CERT_PATH` / `KEY_PATH` | TLS certificate/key used when enabling HTTPS/WSS. |
| `USE_RPI`, `USE_JETSON_NANO` | Toggle hardware-specific drivers. |
| `CAMERA_BACKEND` | Choose between `picamera2`, `v4l2`, or `auto`. |
| `ORIGIN_ALLOW` | Comma-separated list of allowed frontend origins for CORS. |

See the template for Tailscale-specific values and additional tuning knobs.

## Installation

1. Install Go dependencies
   ```bash
   go mod download
   ```
2. Create a virtual environment for Python components
   ```bash
   python -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```
3. (Optional) Install ROS packages and camera libraries on the Pi following your distro's
   documentation.

## Running the services

Choose the mode that suits your workflow:

- **Go WebSocket server only**
  ```bash
  go run main.go
  ```
- **Go server + Python video + ROS bootstrap**
  ```bash
  go run main_combined.go
  ```
- **FastAPI gateway** (exposes REST endpoints and proxies WebSockets)
  ```bash
  python main_api.py
  ```
- **Standalone video streamer**
  ```bash
  python video/video_server.py
  ```

When running locally without hardware, export `USE_RPI=false` to switch GPIO access to the
mock implementation (`gpio/mock_gpio.go`).

## Testing

Activate your virtual environment first (`source .venv/bin/activate`) when running Python
commands.

- Go unit tests
  ```bash
  go test ./...
  ```
- Python unit/integration suites
  ```bash
  export PYTHONPATH=$(pwd)
  pytest tests/unit
  pytest tests/integration
  pytest tests/e2e
  ```
- FastAPI checks (optional)
  ```bash
  pytest tests/api
  ```

Continuous integration executes these suites on pull requests.

## Hardware notes

The backend is optimised for the Raspberry Pi 5. Because the Pi 5 replaces `RPi.GPIO` with
`libgpiod`, ensure the `lgpio` Python package is installed and grant the GPIO group access
before running the services:

```bash
sudo chown root:gpio /dev/gpiochip0
sudo chmod g+rw /dev/gpiochip0
```

With permissions in place the rest of the code behaves the same as it did on a Pi 4 while
benefiting from the Pi 5 performance uplift.

## Troubleshooting

- **WebSocket refuses connection**: Confirm the frontend origin is listed in `ORIGIN_ALLOW`
  and that the IP/port in `.env` matches the machine running the Go server.
- **Video feed is blank**: Check `CAMERA_BACKEND` and verify the upstream `libcamera`
  pipeline is installed. Run `python video/video_server.py` manually to confirm.
- **Running without hardware**: Set `USE_RPI=false` and use the mock PCA9685 and GPIO
  modules for development.

## License

This backend is covered by the repository's MIT license.
