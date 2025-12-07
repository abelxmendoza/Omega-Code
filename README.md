# OMEGA-1 — Autonomous Robotics Platform (Omega Technologies)

Omega-1 is not a "remote-control car."  

It is a **modular, full-stack robotics platform** engineered to scale from:

- GPIO-level control →  
- Real-time perception →  
- Multi-sensor autonomy →  
- Jetson-powered machine learning.

This project represents **Season 1** of the Omega Technologies robotic ecosystem.

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
Frontend (Vercel UI)
    ↓ WebSocket / REST APIs
Go Lighting Server (8082)
    ↓
Python Backend (movement, video, sensors)
    ↓
Raspberry Pi 4B Hardware Layer
```

### Subsystems:

- **Movement Engine V2**  
- **Lighting Engine (Go + Python)**  
- **Video Streaming Server**  
- **Hybrid System Manager**  
- **Diagnostics + Health Monitoring**  
- **Modular Communication Layers**  
- **Autonomy-Ready Hooks**

Each subsystem is isolated, documented, and future-extendable.

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

- `omega_signature` — Multi-stage brand showcase (awakening pulse → surge → heartbeat → breathing)  
- `omega_pulse` — Purple core, red edge, breathing pulse  
- `omega_flux` — Alternating pulse for direction indicators  
- `omega_core` — "AI alive" mode for idle state  

**Port:** 8082 (WebSocket: `ws://<pi-ip>:8082/lighting`)

---

## 🏎️ Movement Engine (Movement V2)

**Features:**

- PID-based speed regulation  
- Ramping (accel/decel profiles)  
- Thermal safety  
- Watchdog system  
- Movement profiles (smooth, aggressive, precision)  
- Direction correction via sensor fusion (future)
- Servo centering on startup
- Straight backward motion (trim fix applied)

**Movement V2 Components:**

- **ProfileManager** - Movement profiles with speed/ramp tuning
- **ThermalSafety** - Motor temperature and current monitoring
- **MovementRamp** - Smooth acceleration/deceleration
- **MovementWatchdog** - Auto-stop on command timeout
- **SpeedPID** - PID-based speed control

**Port:** 7070 (WebSocket: `ws://<pi-ip>:7070/ws/movement`)

---

## 📡 Communication + Control

**Supports:**

- **Direct LAN control**
- **Tailscale remote operation**
- **Bluetooth tether** (in development)
- **Vercel frontend UI**

**API Layer:**

- `/movement` - Motor control, servo control, speed management
- `/lighting` - LED strip control, patterns, brightness
- `/video` - Video streaming, recording, camera management
- `/health` - System health checks
- `/diagnostics` - Hardware diagnostics

**WebSocket Endpoints:**

- `ws://<pi-ip>:7070/ws/movement` - Movement control
- `ws://<pi-ip>:8082/lighting` - Lighting control
- `ws://<pi-ip>:5000/socket.io` - Video metrics (optional)

---

## 🛡️ Diagnostics & Reliability

**Subsystem health checks:**

- File integrity
- Sensor fallbacks
- Camera backend verification
- FPS stability monitoring
- Thermal + current monitoring for motors

**`hw_check.py` includes:**

- Ribbon cable detection heuristics  
- V4L2 device inspection  
- libcamera test automation  
- Permission verification
- Hardware status reporting

**Auto-recovery:**

- Camera initialization retries
- Hardware fallback modes
- Graceful degradation
- Detailed error messages with repair instructions

---

## ⚡ Current Capabilities (Omega-1)

- ✅ Real-time control via browser  
- ✅ Full lighting control + 13+ effects  
- ✅ Live video streaming  
- ✅ Motion + ArUco detection  
- ✅ Object tracking via KCF  
- ✅ Sensor abstraction layer
- ✅ Configurable camera backend (PiCamera2 / V4L2)  
- ✅ Hardware diagnostics system  
- ✅ Hybrid system manager (Pi-only, future Jetson integration)  
- ✅ Modular backend designed for scaling  
- ✅ Movement V2 with profiles and safety systems
- ✅ Omega Signature lighting pattern
- ✅ Camera diagnostic module

**This is already more than 90% of hobby robotics projects online.**

---

## 🧬 Future Roadmap (Omega-2, Omega-3)

The platform is engineered for expansion:

### 🔮 Phase 1 — Autonomy Foundation (Pi)

- Line following with PID  
- Dead-reckoning + basic odometry  
- Collision avoidance  
- Waypoint navigation (no ML)

### 🤖 Phase 2 — Jetson Orin Nano Upgrade

- Stereo camera add-on  
- Real-time object detection (YOLO-Nano / YOLO-Fastest)  
- Lane detection  
- Improved tracking + multi-object classification  

### 🧭 Phase 3 — ML Navigation

- Visual servoing  
- Color-based target tracking  
- Re-identification models  
- Mini-SLAM with ORB-SLAM2 Lite  

### 🚀 Phase 4 — Full Autonomy (Omega-3)

- Behavior trees  
- Sensor fusion  
- Route planning  
- Multi-sensor perception  
- On-board LLM for decision support  

---

## 🛠️ Why This Is More Than a "Toy Robot"

Omega-1 includes:

- ✅ A multi-language stack (Go + Python + TypeScript)
- ✅ Real subsystems with proper architecture
- ✅ Clear separation of concerns  
- ✅ Performance-optimized servers  
- ✅ Production-grade WebSocket systems  
- ✅ Autonomy hooks
- ✅ ML-ready pipelines
- ✅ Diagnostic and fallback modes
- ✅ A roadmap for high-level autonomy

This is the **foundation of a robotics company** — not a toy car.

---

## ⚙️ Quick Start

### Prerequisites

- Raspberry Pi 4B (8GB recommended)
- WS2812/WS2811 LED strip (optional, for lighting)
- Camera module (CSI ribbon or USB webcam)
- PCA9685 servo driver board
- Motor drivers + motors

### Installation

```bash
# Clone repository
git clone https://github.com/abelxmendoza/Omega-Code
cd Omega-Code

# Install Python dependencies
cd servers/robot_controller_backend
pip install -r requirements.txt

# Install Go dependencies (for lighting server)
cd controllers/lighting
go mod tidy
```

### Configuration

#### Network Setup (First Boot)

**Omega-1 Network Wizard** - Headless Wi-Fi setup for field operations:

```bash
cd servers/robot_controller_backend/network
sudo bash install.sh
sudo omega-network ap
```

This enables **Access Point mode** so you can:
- Connect to `Omega1-AP` Wi-Fi network (password: `omegawifi123`)
- SSH into `omega1@192.168.4.1` without a router or monitor
- Perfect for field operations and headless setup

**Quick commands:**
```bash
sudo omega-network ap       # Enable AP mode (field mode)
sudo omega-network client   # Enable client mode (home Wi-Fi)
sudo omega-network status   # Show network status
sudo omega-network          # Interactive menu
```

See `servers/robot_controller_backend/network/README.md` for full documentation.

#### Backend Environment Variables

Copy `.env.example` to `.env` and configure:

```bash
cd servers/robot_controller_backend
cp .env.example .env
nano .env  # Edit with your settings
```

**Required settings:**
- `CAMERA_BACKEND=picamera2` (or `v4l2` for USB)
- `CAMERA_DEVICE=/dev/video0`
- `CAMERA_WIDTH=640`
- `CAMERA_HEIGHT=480`
- `CAMERA_FPS=30`

**Optional settings:**
- `CAMERA_TEST_MODE=1` - Run hardware diagnostics on startup
- `HOST=0.0.0.0` - Server bind address
- `PORT=8000` - API server port
- `LOG_LEVEL=INFO` - Logging verbosity

#### Frontend Environment Variables

Create `ui/robot-controller-ui/.env.local`:

```bash
# Network profile (lan | tailscale | local)
NEXT_PUBLIC_NETWORK_PROFILE=lan

# Robot host IPs (replace with your Pi's IP)
NEXT_PUBLIC_ROBOT_HOST_LAN=192.168.6.164
NEXT_PUBLIC_ROBOT_HOST_TAILSCALE=100.93.225.61

# Video stream URLs
NEXT_PUBLIC_VIDEO_STREAM_URL_LAN=http://192.168.6.164:5000/video_feed

# WebSocket URLs
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN=ws://192.168.6.164:7070/ws/movement
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN=ws://192.168.6.164:8082/lighting
```

**Note:** The frontend automatically detects Vercel deployment and runs in "Robot Offline Mode" for portfolio demos.

### Running Locally (Development)

```bash
# Terminal 1: Lighting Server
cd servers/robot_controller_backend/controllers/lighting
go run main_lighting.go
# Or use startup script:
./start_lighting_server.sh

# Terminal 2: Video Server
cd ../video
python3 video_server.py

# Terminal 3: Movement Server
cd ../movement
python3 movement_ws_server.py
```

### Frontend UI

The frontend is deployed on Vercel. For local development:

```bash
cd ui/robot-controller-ui
npm install
npm run dev
```

Access at: `http://localhost:3000`

---

## 📚 Additional Documentation

### Server-Specific Documentation

- **[Video Server](servers/robot_controller_backend/video/VIDEO_SERVER_FEATURES.md)** - Complete video system features and API
- **[Camera Diagnostics](servers/robot_controller_backend/video/CAMERA_DIAGNOSTIC_MODULE.md)** - Hardware troubleshooting guide
- **[Movement V2](servers/robot_controller_backend/movement/MOVEMENT_V2_SUMMARY.md)** - Movement engine architecture and features
- **[Hybrid System](servers/robot_controller_backend/video/HYBRID_SYSTEM_README.md)** - Pi + Jetson integration architecture

### Testing

The project includes comprehensive test suites:
- **Backend**: 569+ test cases covering API routes, video processing, hybrid system, and fault injection
- **Frontend**: 828+ test cases covering components, hooks, integration, and E2E workflows
- **Coverage**: Critical paths at 100%, overall >80%
- Run tests: `make test` (all) or see [TEST_SUITE.md](TEST_SUITE.md) for details

### Demo Mode

The frontend UI includes a **Demo Mode** for portfolio deployments:
- Automatically enabled on Vercel (cloud deployments)
- All UI controls work without hardware connection
- Lighting modal fully functional with simulated commands
- Perfect for showcasing capabilities without physical robot

---

## 🔧 Hardware Requirements

**Minimum:**
- Raspberry Pi 4B (4GB)
- Camera module (CSI or USB)
- PCA9685 servo driver
- Motor drivers

**Recommended:**
- Raspberry Pi 4B (8GB)
- WS2812 LED strip (16+ LEDs)
- Ultrasonic sensors
- Line tracking sensors

**Future (Omega-2):**
- Jetson Orin Nano
- Stereo camera
- Additional sensors

---

## 🐛 Troubleshooting

### Camera Issues

Run hardware diagnostic:
```bash
cd servers/robot_controller_backend/video
python3 hw_check.py
```

Common fixes:
- Reseat CSI ribbon cable (silver contacts face HDMI side)
- Check permissions: `sudo usermod -a -G video $USER`
- Verify camera: `vcgencmd get_camera`

### Lighting Issues

Check lighting server:
```bash
curl http://localhost:8082/health
```

Verify LED strip connection and GPIO pin 18.

### Movement Issues

Check movement server logs:
```bash
tail -f servers/robot_controller_backend/movement/movement_ws_server.log
```

Verify motor connections and PCA9685 I2C address.

---

## 🤝 Contributing

This is a private project, but contributions are welcome through:

1. Issue reports
2. Feature requests
3. Documentation improvements
4. Code optimizations

---

## 📄 License

See [LICENSE](LICENSE) file for details.

---

## 🎯 Status

**Current Version:** Omega-1 (Season 1)  
**Status:** Production Ready ✅  
**Next Phase:** Omega-2 (Jetson Integration)

---

**Built with ❤️ by Omega Technologies**
