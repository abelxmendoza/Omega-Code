# OMEGA-1 — Autonomous Robotics Platform (Omega Technologies)

Omega-1 is not a "remote-control car."  

It is a **modular, full-stack robotics platform** engineered to scale from:

- GPIO-level control →  
- Real-time perception →  
- Multi-sensor autonomy →  
- Jetson-powered machine learning.

This project represents **Season 1** of the Omega Technologies robotic ecosystem.

---

## 🚀 Quick Start — Running the Robot

### Prerequisites

- **Raspberry Pi 4B** (or compatible) running Raspberry Pi OS
- **Python 3.9+** and **Go 1.19+** installed
- **Network connectivity** (Wi-Fi or Ethernet)
- **Hardware components** connected (motors, sensors, camera, LEDs)

### Step 1: Clone and Setup

```bash
# Clone the repository
git clone https://github.com/your-repo/Omega-Code.git
cd Omega-Code

# Navigate to backend
cd servers/robot_controller_backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install Python dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Install Go dependencies
go mod download
```

### Step 2: Configure Environment

```bash
# Copy example environment file
cp .env.example .env

# Edit .env with your settings
nano .env
```

**Key environment variables:**
- `PI_IP` - Your Pi's IP address (e.g., `192.168.1.100`)
- `PORT_MOVEMENT` - Movement WebSocket port (default: `8081`)
- `VIDEO_PORT` - Video server port (default: `5000`)
- `CAMERA_BACKEND` - Camera backend (`picamera2` or `v4l2`)

### Step 3: Start the Robot Services

#### Option A: Using OmegaOS Service Orchestrator (Recommended)

The **OmegaOS Service Orchestrator** automatically manages all robot services:

```bash
# Install the orchestrator
cd omega_services
sudo ./install.sh

# Start orchestrator (auto-starts all services)
sudo systemctl start omega-orchestrator

# Enable auto-start on boot
sudo systemctl enable omega-orchestrator

# Check status
sudo systemctl status omega-orchestrator

# View logs
sudo journalctl -u omega-orchestrator -f
```

**What it does:**
- ✅ Automatically starts all configured services
- ✅ Monitors and restarts crashed services
- ✅ Provides REST API for service management
- ✅ Logs everything to `/var/log/omega/`

**Manage services via Web UI:**
Navigate to `http://omega1.local:3000/services` (or `http://<pi-ip>:3000/services`)

**Manage services via API:**
```bash
# List all services
curl http://localhost:8000/api/services/list

# Start a service
curl -X POST http://localhost:8000/api/services/start/movement_ws_server

# Check service status
curl http://localhost:8000/api/services/status/video_server
```

#### Option B: Manual Service Startup

If you prefer to start services manually:

```bash
# Terminal 1: Movement WebSocket Server
cd servers/robot_controller_backend
python movement/movement_ws_server.py

# Terminal 2: Video Server
python video/video_server.py

# Terminal 3: Main API Server
uvicorn main_api:app --host 0.0.0.0 --port 8000

# Terminal 4: Gateway API (optional)
uvicorn servers.gateway_api:app --host 0.0.0.0 --port 7070

# Terminal 5: Ultrasonic Sensor Server (Go)
go run sensors/main_ultrasonic.go

# Terminal 6: Line Tracker Server
python sensors/line_tracking_ws_server.py
```

#### Option C: Using Startup Scripts

```bash
# Start all services with monitoring
cd servers/robot_controller_backend
bash scripts/start.sh

# Or use the all-in-one script
bash scripts/start_all.sh
```

### Step 4: Access the Web UI

1. **Start the frontend** (on your development machine or Pi):

```bash
cd ui/robot-controller-ui
npm install
npm run dev
```

2. **Open in browser:**
   - Local: `http://localhost:3000`
   - Remote: `http://<pi-ip>:3000` or `http://omega1.local:3000`

3. **Available pages:**
   - `/` - Main control dashboard
   - `/control` - Joystick control
   - `/services` - Service management (OmegaOS)
   - `/network` - Network configuration
   - `/ros` - ROS 2 dashboard
   - `/telemetry` - System metrics

### Step 5: Verify Everything Works

**Check service status:**
```bash
# Via API
curl http://localhost:8000/api/services/list

# Via Web UI
# Navigate to http://<pi-ip>:3000/services
```

**Test endpoints:**
```bash
# Health check
curl http://localhost:8000/health

# Video feed
curl http://localhost:5000/video_feed

# Movement WebSocket (test with wscat)
wscat -c ws://localhost:8081/
```

---

## 🔥 Vision

Build an affordable, modular robot capable of:

- Real-time video streaming
- Remote operation
- Lighting + signaling systems
- Sensor-driven safety
- Hybrid autonomy
- Future ML-assisted navigation

Omega-1 is designed as the **prototype brain and skeleton** for future upgrades (Omega-2 on Jetson).

---

## 🧠 System Architecture Overview

```
Frontend (Next.js UI)
    ↓ WebSocket / REST APIs
OmegaOS Service Orchestrator
    ↓
Go Lighting Server (8082)
Python Backend (movement, video, sensors)
    ↓
Raspberry Pi 4B Hardware Layer
```

### Subsystems:

- **OmegaOS Service Orchestrator** - Unified service management
- **Movement Engine V2** - Advanced motor control with profiles
- **Lighting Engine (Go + Python)** - LED pattern control
- **Video Streaming Server** - Real-time camera feed
- **Hybrid System Manager** - Multi-platform support
- **Network Wizard** - AP/Client mode switching
- **Diagnostics + Health Monitoring** - System health tracking
- **Modular Communication Layers** - WebSocket + REST APIs
- **Autonomy-Ready Hooks** - ROS 2 integration

Each subsystem is isolated, documented, and future-extendable.

---

## 📋 Service Management (OmegaOS)

Omega-1 uses **OmegaOS Service Orchestrator** for unified service management.

### Pre-configured Services

1. **movement_ws_server** - Movement WebSocket (autostart, always restart)
2. **video_server** - Video streaming (autostart, on-failure restart)
3. **main_api** - FastAPI server (autostart, always restart)
4. **ultrasonic_ws_server** - Ultrasonic sensors (autostart, on-failure restart)
5. **line_tracking_ws_server** - Line tracker (autostart, on-failure restart)
6. **lighting_server** - LED control (manual start, no restart)
7. **ros_core** - ROS 2 Docker (manual start, on-failure restart)
8. **gateway_api** - Gateway proxy (autostart, always restart)

### Managing Services

**Via Web UI:**
- Navigate to `/services` page
- Click Start/Stop/Restart buttons
- View logs and health status

**Via API:**
```bash
# List services
GET /api/services/list

# Start service
POST /api/services/start/{name}

# Stop service
POST /api/services/stop/{name}

# Restart service
POST /api/services/restart/{name}

# View logs
GET /api/services/logs/{name}?lines=100

# Check health
GET /api/services/health/{name}
```

**Via CLI:**
```bash
# Start orchestrator
python3 -m omega_services.service_manager

# Check service status
curl http://localhost:8000/api/services/status/movement_ws_server
```

See `servers/robot_controller_backend/omega_services/README.md` for complete documentation.

---

## 🎥 Video Server (Real-Time Perception)

**Tech:** Flask-SocketIO, OpenCV, Picamera2, V4L2, ArUco

**Features:**

- Live MJPEG video streaming  
- Motion detection  
- ArUco marker detection  
- Object tracking (KCF)  
- Hardware-aware performance tuning  
- Automatic camera fallback  
- Frame buffering + overlay system  
- Diagnostic module (`hw_check.py`)

> If the camera fails, Omega-1 self-diagnoses and prints repair instructions.

**Endpoints:**
- `GET /video_feed` - Main MJPEG stream
- `GET /video_feed_low` - Low quality (320x240)
- `GET /video_feed_high` - High quality (1280x720)
- `GET /health` - Health check with camera status
- `GET /snapshot` - Single frame capture
- `POST /recording/start` - Start video recording
- `POST /recording/stop` - Stop recording

**Start:**
```bash
python video/video_server.py
```

---

## 💡 Lighting Engine (Go + Python Hybrid)

**Tech:** Go WebSocket server + Python rpi_ws281x

**Capabilities:**

- 13+ lighting patterns (rainbow, aurora, matrix, fire, rave, breathing, **omega_signature**)  
- Omega Technologies custom theme (purple/red cyberpunk palette)  
- Real-time WebSocket control  
- Hardware fallback mode  
- Ultra-low latency LED updates  
- Pattern dispatching + color caching  
- REST + WS control

**Custom Omega Patterns:**
- `omega_signature` - Brand signature pattern (purple/red pulsing)

**Start:**
```bash
# Go WebSocket server
go run controllers/lighting/main_lighting.go

# Or via orchestrator
curl -X POST http://localhost:8000/api/services/start/lighting_server
```

---

## 🌐 Network Setup

Omega-1 includes **two network management tools**:

### Omega-NetToggle (Recommended for Recovery)

**Clean network recovery + AP mode script** - Best for fixing broken WiFi:

```bash
# Restore normal WiFi mode (recover from breakage)
sudo omega-nettoggle restore

# Enable AP mode
sudo omega-nettoggle ap

# Show network diagnostics
sudo omega-nettoggle status
```

**Features:**
- ✅ Complete WiFi recovery (driver reload, NetworkManager restart)
- ✅ Clean AP mode switching
- ✅ Safety backups + comprehensive logging
- ✅ NetworkManager compatible (Raspberry Pi OS Bookworm)

See `servers/robot_controller_backend/network/OMEGA_NETTOGGLE.md` for complete documentation.

### Network Setup Wizard (Full Configuration)

**Comprehensive network configuration** - Best for initial setup:

```bash
# Enable AP mode
sudo omega-network ap

# Enable client mode
sudo omega-network client

# Scan and connect
sudo omega-network wifi-scan
sudo omega-network wifi-connect "YourWiFi" "password"
```

**Via Web UI:**
Navigate to `/network` page for visual network management.

See `servers/robot_controller_backend/network/README.md` for complete documentation.

---

## 🔧 Configuration

### Environment Variables

Key variables in `.env`:

```bash
# Network
PI_IP=192.168.1.100
TAILSCALE_IP_PI=100.x.x.x

# Ports
PORT_MOVEMENT=8081
VIDEO_PORT=5000
PORT_ULTRASONIC=8080

# Camera
CAMERA_BACKEND=picamera2
CAMERA_WIDTH=640
CAMERA_HEIGHT=480
CAMERA_FPS=30

# Hardware
USE_RPI=1
ROBOT_SIM=0  # Set to 1 for development without hardware
```

### Service Registry

Edit `servers/robot_controller_backend/omega_services/service_registry.json` to:
- Add new services
- Change autostart settings
- Modify restart policies
- Update health checks

---

## 🛠️ Troubleshooting

### Services Won't Start

1. **Check logs:**
   ```bash
   # Orchestrator logs
   sudo journalctl -u omega-orchestrator -f
   
   # Service logs
   tail -f /var/log/omega/{service_name}.stdout.log
   tail -f /var/log/omega/{service_name}.stderr.log
   ```

2. **Check permissions:**
   ```bash
   # GPIO access
   sudo chown root:gpio /dev/gpiochip0
   sudo chmod g+rw /dev/gpiochip0
   ```

3. **Verify dependencies:**
   ```bash
   pip install -r requirements.txt
   go mod download
   ```

### Camera Not Working

1. **Run diagnostics:**
   ```bash
   python video/hw_check.py
   ```

2. **Check camera:**
   ```bash
   vcgencmd get_camera
   libcamera-still -o test.jpg
   ```

3. **Verify backend:**
   ```bash
   # Check .env
   CAMERA_BACKEND=picamera2
   CAMERA_DEVICE=/dev/video0
   ```

### Network Issues

1. **Check network status:**
   ```bash
   sudo omega-network status
   ```

2. **Restart network services:**
   ```bash
   sudo systemctl restart hostapd
   sudo systemctl restart dnsmasq
   ```

### Web UI Not Connecting

1. **Check backend is running:**
   ```bash
   curl http://localhost:8000/health
   ```

2. **Verify CORS settings:**
   ```bash
   # In .env
   ORIGIN_ALLOW=http://localhost:3000,http://omega1.local:3000
   ```

3. **Check firewall:**
   ```bash
   sudo ufw allow 8000/tcp
   sudo ufw allow 3000/tcp
   ```

---

## 📚 Documentation

- **Service Orchestrator:** `servers/robot_controller_backend/omega_services/README.md`
- **Network Wizard:** `servers/robot_controller_backend/network/README.md`
- **Backend API:** `servers/robot_controller_backend/README.md`
- **Video Server:** `servers/robot_controller_backend/video/README.md`
- **Movement V2:** `servers/robot_controller_backend/movement/MOVEMENT_V2_SUMMARY.md`

---

## 🧪 Testing

```bash
# Python tests
cd servers/robot_controller_backend
pytest tests/

# Go tests
go test ./...

# Frontend tests
cd ui/robot-controller-ui
npm test
```

---

## 📦 Deployment

### Production Setup

1. **Install orchestrator:**
   ```bash
   cd servers/robot_controller_backend/omega_services
   sudo ./install.sh
   ```

2. **Enable services:**
   ```bash
   sudo systemctl enable omega-orchestrator
   sudo systemctl start omega-orchestrator
   ```

3. **Configure network:**
   ```bash
   sudo omega-network ap  # For field use
   # or
   sudo omega-network client  # For home use
   ```

4. **Access Web UI:**
   - Deploy frontend to Vercel or run on Pi
   - Navigate to `http://omega1.local:3000`

---

## 🎯 Development

### Running in Development Mode

```bash
# Set simulation mode (no hardware)
export ROBOT_SIM=1

# Run services individually
python movement/movement_ws_server.py
python video/video_server.py
uvicorn main_api:app --reload
```

### Adding New Services

1. **Add to service registry:**
   ```json
   {
     "name": "my_service",
     "cmd": "python3",
     "args": ["my_service.py"],
     "autostart": true,
     "restart_policy": "always"
   }
   ```

2. **Add health check** (optional):
   ```python
   # In omega_services/health_checks.py
   def my_service_check():
       return {"healthy": True, "message": "OK"}
   ```

---

## 📄 License

MIT License - See LICENSE file for details.

---

## 🤝 Contributing

Contributions welcome! Please read CONTRIBUTING.md for guidelines.

---

## 📞 Support

- **Issues:** GitHub Issues
- **Documentation:** See `/docs` directory
- **Wiki:** GitHub Wiki (coming soon)

---

**Built with ❤️ by Omega Technologies**
