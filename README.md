# Omega-Code

Omega-Code is a full robotics control stack for a Raspberry Pi powered rover. The
repository hosts:

- Hardware-facing Go and Python services that stream video, drive motors, manage
  lighting, and expose a FastAPI/Flask gateway for operators.
- A modern Next.js UI that provides real-time teleoperation, telemetry, and
  connectivity tooling.
- ROS launch files and simulation packages for autonomy experiments.
- Bash utilities that simplify provisioning, Bluetooth PAN setup, and network
  diagnostics when you are in the field.

Everything can run on the Pi, but the services are split so you can develop the UI
on your laptop while tunnelling to the robot, or spin everything up locally with
mock hardware for rapid iteration.

## Repository layout

| Path | Description |
| --- | --- |
| `Makefile` | Shortcut targets for installing deps, probing endpoints, and starting the UI/back-end services with a chosen profile. |
| `servers/robot-controller-backend/` | Multi-language backend: Go WebSocket servers, FastAPI + Flask gateways, Python controllers, ROS bootstrap helpers, video streaming, diagnostics, and autonomy orchestration. |
| `ui/robot-controller-ui/` | Next.js 14 + React 18 dashboard with Redux Toolkit, Tailwind, Cypress, and Jest. Includes a network wizard, video proxy routes, and autonomy controls. |
| `ros/` | Launch files, packages, and helper scripts used for ROS-based autonomy and simulation. |
| `scripts/` | Bash utilities (`check_endpoints.sh`, `start_robot.sh`, PAN helpers, hotspot setup, etc.) shared by operators in the field. |
| `image/` | Reference screenshots and assets referenced in the documentation. |
| `.env.example` | Root-level template for hotspot/Bluetooth variables consumed by helper scripts. |

See the READMEs inside `servers/robot-controller-backend` and `ui/robot-controller-ui`
for component-specific details.

## Technology stack

- **Hardware**: Raspberry Pi 5 primary target (Pi 4 works with the `lgpio`
  compatibility layer). Optional Jetson Nano hooks exist in scripts.
- **Backend**: Go 1.22 WebSocket services, Python 3.11 controllers, FastAPI 0.111
  REST endpoints, Flask-based MJPEG video streaming with OpenCV, `websockets`
  proxies, and ROS launch helpers.
- **Frontend**: Next.js 14, React 18, TypeScript, Redux Toolkit, Radix UI, Tailwind
  CSS, Leaflet, and a comprehensive Jest + Cypress test suite.
- **Automation**: Modular autonomy controller with async mode handlers, ROS launch
  files, and bash orchestration scripts.

## Prerequisites

Install the tooling required for the parts you plan to work on:

- Go 1.22+
- Python 3.10+ (3.11 recommended) with `pip` (and optionally `virtualenv`)
- Node.js 18.17+ (Node 20 LTS recommended) and npm 9+
- Bash, `make`, and `git`
- Optional on the Pi: `python3-picamera2`, `python3-libcamera`, `rpicam-apps`,
  `v4l-utils`, `python3-lgpio`, `bluez` tooling, and ROS (Noetic/Humble depending on
  your launch files)

## Environment configuration

1. Copy the root template if you use the helper scripts for hotspot/Bluetooth:
   ```bash
   cp .env.example .env
   ```
2. Configure the backend services:
   ```bash
   cd servers/robot-controller-backend
   cp .env.example .env
   # Edit ports, Pi/Tailscale IPs, TLS paths, origin allow-list, camera backend, etc.
   ```
3. Configure the UI environment:
   ```bash
   cd ../../ui/robot-controller-ui
   cp .env.local.example .env.local
   # Set NEXT_PUBLIC_GATEWAY_HOST, per-profile WebSocket URLs, and video stream targets.
   ```
   The UI supports `local`, `lan`, and `tailscale` profiles. Profile-specific URLs are
   derived from the `NEXT_PUBLIC_*` variables and can be overridden at runtime via the
   `?profile=` query parameter.

Optional: create `scripts/.env` if you want the Makefile helpers to preload operator
values such as `PHONE_MAC` or SSH hosts.

## Backend setup

From `servers/robot-controller-backend`:

```bash
# Go modules
go mod download

# Python environment
python -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

The requirements file documents the apt packages you need on the Pi before installing
Python dependencies. For convenience you can also use `make venv` and
`make backend-install` from the repository root.

## Frontend setup

From `ui/robot-controller-ui`:

```bash
npm install
```

Install optional global tooling (`npm i -g wscat`) if you plan to use the Makefile
WebSocket helpers.

## Running the stack

You can launch services individually or use the Makefile shortcuts (which load
environment files automatically).

### Individual components

| Component | Command | Notes |
| --- | --- | --- |
| Movement WebSocket | `cd servers/robot-controller-backend/movement && python movement_ws_server.py` | Implements movement, servo, buzzer, and autonomy hand-off logic. Honours `PORT_MOVEMENT`, `MOVEMENT_PATH`, `ROBOT_SIM`, and `ORIGIN_ALLOW`. |
| Ultrasonic WebSocket | `cd servers/robot-controller-backend/sensors && go run main_ultrasonic.go` | Streams HC-SR04 readings, supports configurable paths and origin allow-lists via `ULTRA_*` vars. |
| Line tracker feed | `cd servers/robot-controller-backend/sensors && python line_tracking_ws_server.py` | Publishes line-tracker states over WebSockets. |
| Lighting control | `cd servers/robot-controller-backend/controllers/lighting && go run main_lighting.go` | Proxies lighting commands to the privileged `run_led.sh` wrapper / Python LED controller. |
| Video server | `cd servers/robot-controller-backend && python video/video_server.py` | Flask + OpenCV MJPEG server with watchdog and placeholder frames. Configure with `VIDEO_PORT`, `CAMERA_*`, `STARTUP_RETRY_SEC`, etc. |
| FastAPI REST API | `cd servers/robot-controller-backend && uvicorn main_api:app --host 0.0.0.0 --port 8000 --reload` | Exposes `/lighting` and `/autonomy` routes composed from `api/`. |
| Gateway proxy | `cd servers/robot-controller-backend && uvicorn servers.gateway_api:app --host 0.0.0.0 --port 7070` | Aggregates `/ws/*`, `/video_feed`, and `/api/net/*` endpoints. Configure downstreams with `DS_MOVE`, `DS_ULTRA`, `DS_LIGHT`, `DS_LINE`, and `VIDEO_UPSTREAM`. |

### Makefile helpers

From the repository root:

```bash
# Print resolved URLs for a profile (local | lan | tailscale)
make check PROFILE=tailscale

# Start the movement server, video server, or UI using the active profile
make run-movement
make run-video
make ui-dev PROFILE=lan
```

Additional targets exist for PAN setup (`make pan`, `make mac-pan`), environment
inspection (`make env-print`), and backend installation (`make backend-install`).

### UI development

```bash
cd ui/robot-controller-ui
npm run dev
# or use the Makefile shortcut
make ui-dev PROFILE=local
```

The app listens on <http://localhost:3000> and reads WebSocket/video URLs from
`.env.local`. The network wizard and status bar display which profile is active and
let you trigger gateway actions when the backend exposes them.

### ROS tooling and orchestration

ROS launch files live under `ros/launch`, and example packages live in
`ros/robot_simulation`. The `scripts/start_robot.sh` helper shows how to bring up ROS,
launch Pi nodes over SSH, run `main_combined.go`, and start the UI on a MacBook in one
shot. Adjust the script to point at your actual package names and hosts.

## Diagnostics and field utilities

- `scripts/check_endpoints.sh` – Profile-aware reachability checker for movement,
  ultrasonic, lighting, and video services.
- `servers/robot-controller-backend/diagnostics.py` – Rich CLI diagnostic sweep for
  GPIO peripherals on the Pi 5 (ultrasonic, IR line tracker, LEDs, buzzer, camera, system info).
- `scripts/connect_*` – Bluetooth PAN and hotspot automation scripts.

## Testing and linting

Run the suites that correspond to your changes:

- **Backend Go**: `cd servers/robot-controller-backend && go test ./...`
- **Backend Python**: activate the virtual environment and run `pytest tests/unit`,
  `pytest tests/integration`, `pytest tests/e2e`, plus `pytest tests/api` for the
  FastAPI layer.
- **Frontend**: `cd ui/robot-controller-ui && npm run lint`, `npm test`, `npx cypress run`
  (Cypress expects the backend/gateway to be running).

Continuous integration executes the same commands on pull requests.

## Contributing

Bug reports and pull requests are welcome. Please update the backend and frontend
READMEs whenever you change ports, environment variables, or the control surface so
other operators stay in sync. Review the existing scripts under `scripts/` before
adding new automation—many common workflows already have helpers.

## License

Omega-Code is distributed under the MIT License. See `LICENSE` for details.
