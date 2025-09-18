# Omega-Code

Omega-Code is a full robotics control stack for a Raspberry Pi based rover. The repository
hosts the backend services that talk to the hardware, a modern web frontend for operators,
ROS packages for autonomy experiments, and helper scripts for provisioning the device. The
project is designed so you can run the backend on a Pi while developing the UI on your
laptop, or run everything locally for fast iteration.

## Repository layout

| Path | Description |
| --- | --- |
| `servers/robot-controller-backend` | Go + Python services that expose WebSocket and REST APIs, talk to the PCA9685, sensors, lighting, and stream video. |
| `ui/robot-controller-ui` | Next.js/React single-page application that drives the rover, renders telemetry, and manages lighting scenes. |
| `ros` | ROS 2 packages, launch files, and scripts for autonomy features such as A\* and RRT path planners. |
| `scripts` | Shell utilities for networking (Wi-Fi/Bluetooth PAN), provisioning, and diagnostics when deploying to a Pi. |
| `image` | Reference screenshots used in the documentation. |
| `.env.example` | Root environment template with shared settings (phone hotspot, default backend port). |

Other subdirectories inside `servers/robot-controller-backend` contain hardware shims,
Rust helpers, and vendored libcamera utilities that support the capture pipeline.

## Technology stack

- **Hardware**: Raspberry Pi 5 (works on Pi 4 with `lgpio` compatibility layer).
- **Backend**: Go 1.22 WebSocket server plus Python controllers, FastAPI gateway, and
  optional ROS integration.
- **Frontend**: Next.js 14 with React 18, Redux Toolkit, Radix UI primitives, Leaflet
  for mapping, and Cypress/Jest for testing.
- **Autonomy**: ROS nodes for path planning, simulation, and sensor fusion.

## Prerequisites

Install the tooling you need for the parts you plan to work on:

- Go 1.22+
- Python 3.10+ with `pip` and (optionally) `virtualenv`
- Node.js 18.17+ (Node 20 LTS recommended) and npm
- Optional hardware libraries on the Pi: `lgpio`, `libcamera`, `picamera2`, and ROS 2
  (see the backend README for exact packages)

## Getting started

1. **Clone the repository**
   ```bash
   git clone https://github.com/abelxmendoza/Omega-Code.git
   cd Omega-Code
   ```
2. **Configure shared environment variables**
   ```bash
   cp .env.example .env
   # Edit .env with the hotspot MAC address, Pi IP, and backend port.
   ```
3. **Install backend dependencies (on the Pi or your dev machine)**
   ```bash
   cd servers/robot-controller-backend
   go mod download
   python -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   cp .env.example .env  # customise IPs, ports, TLS paths, camera backend...
   ```
4. **Install frontend dependencies**
   ```bash
   cd ../../ui/robot-controller-ui
   npm install
   cp .env.local.example .env.local  # update URLs to point at your backend
   ```

## Running the stack

### Backend services

From `servers/robot-controller-backend`:

- WebSocket controller only (movement, sensors, lighting):
  ```bash
  go run main.go
  ```
- Full stack with Python video server and ROS bootstrapping:
  ```bash
  go run main_combined.go
  ```
- REST API gateway (FastAPI + Uvicorn, optional):
  ```bash
  python main_api.py
  ```
- Standalone MJPEG video stream if you need to debug camera output:
  ```bash
  python video/video_server.py
  ```

The `.env` file in this directory governs which IP/ports to bind, TLS certificate
locations, and whether to use Tailscale or LAN hosts. See the backend README for the
complete list.

### Frontend UI

From `ui/robot-controller-ui`:
```bash
npm run dev
```
The UI will start on <http://localhost:3000>. It uses the WebSocket URLs from
`.env.local` to connect to the backend and adapts automatically between `local`, `lan`,
and `tailscale` profiles.

### ROS tooling

ROS components live in `ros/`. Use the supplied launch files and scripts from that
folder when working on autonomy features. The shell helpers in `scripts/` provide common
networking tasks such as connecting the Pi to a hotspot (`connect_hotspot_v2.sh`) and
configuring Bluetooth PAN (`pi_bt_pair_and_pan.sh`).

## Testing and linting

Run the suites relevant to your changes:

- **Backend Go**: `go test ./...`
- **Backend Python**: `pytest` (with `PYTHONPATH=$(pwd)` when running from the backend
  directory)
- **Frontend**: `npm run lint`, `npm test`, and `npx cypress run`
- **ROS**: ROS-specific tests live under `ros/tests`

Continuous integration runs the same commands on pull requests.

## Contributing

Bug reports and pull requests are welcome. Please include tests when possible and keep the
backend/frontend READMEs up to date with any new configuration. Review the existing scripts
in the `scripts/` directory before adding new automation—there may already be a helper for
what you need.

## License

Omega-Code is distributed under the MIT License. See `LICENSE` for details.
