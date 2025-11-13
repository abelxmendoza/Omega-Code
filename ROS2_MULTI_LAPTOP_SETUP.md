# Multi-Laptop Development Setup

This guide covers development with **two laptops** plus **Raspberry Pi**:

- **Lenovo (Ubuntu)**: ROS2 Rolling - Development & ROS2 work
- **MacBook (macOS)**: No ROS2 - UI/Backend development
- **Raspberry Pi**: Docker ROS2 Humble - Hardware control

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              Multi-Laptop + Pi Development                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐ │
│  │   Lenovo     │    │   MacBook    │    │  Raspberry   │ │
│  │  (Ubuntu)    │    │   (macOS)    │    │     Pi 4B    │ │
│  │              │    │              │    │              │ │
│  │ Native ROS2  │    │  No ROS2     │    │ Docker ROS2  │ │
│  │   Rolling    │    │              │    │   Humble     │ │
│  │              │    │ • UI Dev     │    │              │ │
│  │ • ROS2 Dev   │    │ • Backend    │    │ • Hardware   │ │
│  │ • RViz       │    │ • Testing   │    │ • Sensors    │ │
│  │ • colcon     │    │              │    │              │ │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘ │
│         │                   │                   │         │
│         └───────────────────┴───────────────────┘         │
│                    ROS2 DDS Network                        │
│              (Lenovo ↔ Pi only)                            │
└─────────────────────────────────────────────────────────────┘
```

## Device Roles

### Lenovo (Ubuntu) - ROS2 Development

**Capabilities:**
- ✅ Native ROS2 Rolling
- ✅ ROS2 package development
- ✅ colcon builds
- ✅ RViz visualization
- ✅ ROS2 topic monitoring
- ✅ SSH to Pi

**Setup:**
```bash
# Already configured
source ~/.ros2_rolling_setup.bash
source ~/omega_ws/install/setup.bash
```

### MacBook (macOS) - UI/Backend Development

**Capabilities:**
- ✅ Next.js UI development
- ✅ FastAPI backend development
- ✅ Docker management (for Pi)
- ✅ Testing without ROS2
- ❌ No native ROS2 (macOS limitation)

**Setup:**
```bash
# No ROS2 needed - backend handles gracefully
cd servers/robot-controller-backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Raspberry Pi - Hardware Control

**Capabilities:**
- ✅ Docker ROS2 Humble containers
- ✅ Hardware IO (GPIO, motors, sensors)
- ✅ Camera capture
- ✅ Real-time control

## Development Workflows

### Workflow 1: ROS2 Development (Lenovo)

```bash
# On Lenovo
cd ~/Omega-Code

# Develop ROS2 packages
cd ~/omega_ws/src/omega_robot
# Edit packages...

# Build
cd ~/omega_ws
colcon build

# Test
ros2 run omega_robot telemetry_publisher
```

### Workflow 2: UI/Backend Development (MacBook)

```bash
# On MacBook
cd ~/Omega-Code

# Backend (no ROS2 needed)
cd servers/robot-controller-backend
source venv/bin/activate
python main_api.py

# UI
cd ui/robot-controller-ui
npm run dev
```

The backend will:
- ✅ Work without ROS2 (graceful fallback)
- ✅ Control Pi Docker containers via SSH
- ✅ Show ROS2 features as "unavailable" on MacBook
- ✅ All other features work normally

### Workflow 3: Full System (Lenovo + Pi)

```bash
# On Lenovo - Start backend with ROS2
export ROS_NATIVE_MODE=true
export PI_SSH_HOST=pi@192.168.1.107
cd servers/robot-controller-backend
source venv/bin/activate
python main_api.py

# Backend connects to:
# - Local native ROS2 (Lenovo)
# - Pi Docker ROS2 (via SSH)
```

## Backend Configuration

### On Lenovo (with ROS2)

```bash
export ROS_NATIVE_MODE=true
export PI_SSH_HOST=pi@192.168.1.107
export PI_SSH_KEY=~/.ssh/id_rsa
```

### On MacBook (without ROS2)

```bash
# Don't set ROS_NATIVE_MODE
export PI_SSH_HOST=pi@192.168.1.107
export PI_SSH_KEY=~/.ssh/id_rsa

# Backend will:
# - Skip native ROS2 features
# - Still control Pi Docker containers
# - Show ROS2 status as "unavailable"
```

## API Behavior

### On Lenovo (with ROS2)

```bash
curl http://localhost:8000/api/ros/status
```

Response:
```json
{
  "containers": [...],
  "topics": [...],
  "mode": "native",
  "location": "local"
}
```

### On MacBook (without ROS2)

```bash
curl http://localhost:8000/api/ros/status
```

Response:
```json
{
  "containers": [...],
  "topics": [...],
  "mode": "docker_pi",
  "location": "pi"
}
```

ROS2 features gracefully degrade - backend still works!

## Code Sharing

### Git Workflow

```bash
# On Lenovo - develop ROS2 packages
git add ros/src/omega_robot/
git commit -m "Add new ROS2 node"
git push

# On MacBook - develop UI/backend
git pull
cd ui/robot-controller-ui
npm install  # Get latest changes
```

### Shared Code

Both laptops share:
- ✅ `servers/robot-controller-backend/` - Backend code
- ✅ `ui/robot-controller-ui/` - UI code
- ✅ `docker/ros2_robot/` - Docker configs
- ✅ `scripts/` - Setup scripts

## Testing Strategy

### On MacBook

```bash
# Test backend without ROS2
cd servers/robot-controller-backend
source venv/bin/activate
pytest tests/  # Tests that don't require ROS2

# Test UI
cd ui/robot-controller-ui
npm test
```

### On Lenovo

```bash
# Test ROS2 integration
cd ~/omega_ws
colcon test

# Test full system
cd servers/robot-controller-backend
pytest tests/  # All tests including ROS2
```

## Environment Variables

### Lenovo `.env`

```bash
# ROS2 Native Mode
ROS_NATIVE_MODE=true
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Pi Control
PI_SSH_HOST=pi@192.168.1.107
PI_SSH_KEY=~/.ssh/id_rsa
```

### MacBook `.env`

```bash
# No ROS2 - backend handles gracefully
# PI_SSH_HOST=pi@192.168.1.107  # Optional: control Pi
# PI_SSH_KEY=~/.ssh/id_rsa
```

## Troubleshooting

### MacBook: "ROS2 not available"

This is **normal** - macOS doesn't support ROS2. The backend:
- ✅ Still works for all non-ROS2 features
- ✅ Can control Pi Docker containers via SSH
- ✅ Shows ROS2 features as unavailable

### Lenovo: ROS2 Not Found

```bash
# Verify ROS2 setup
source ~/.ros2_rolling_setup.bash
ros2 --help

# If missing, run setup
./scripts/setup_ros2_rolling.sh
```

### Pi: Can't Connect from MacBook

```bash
# Test SSH
ssh pi@192.168.1.107

# Verify PI_SSH_HOST is set
echo $PI_SSH_HOST
```

## Best Practices

1. **Develop ROS2 on Lenovo** - Native ROS2 support
2. **Develop UI/Backend on MacBook** - Faster iteration, no ROS2 needed
3. **Test full system on Lenovo** - Complete ROS2 integration
4. **Use Git** - Share code between laptops
5. **Pi for hardware** - Always use Pi for actual robot control

## Quick Reference

| Task | Lenovo | MacBook | Pi |
|------|--------|---------|-----|
| ROS2 Development | ✅ | ❌ | ✅ (Docker) |
| UI Development | ✅ | ✅ | ❌ |
| Backend Development | ✅ | ✅ | ✅ |
| Hardware Control | ❌ | ❌ | ✅ |
| RViz | ✅ | ❌ | ❌ |
| Docker Management | ✅ | ✅ | ✅ |

---

**Last Updated**: 2024  
**Setup**: Lenovo (Ubuntu) + MacBook (macOS) + Pi (Raspberry Pi OS)

