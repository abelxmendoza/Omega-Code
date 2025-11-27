# ROS Docker Container Integration Guide

## Overview

The system manages ROS 2 Docker containers through a Python FastAPI backend that executes Docker commands on the host system. This document explains how the UI, backend, and Docker containers interact.

## Architecture Flow

```
┌─────────────────┐
│   UI Dashboard  │
│  (ros.tsx page) │
└────────┬────────┘
         │ HTTP/WebSocket
         │ /api/ros/*
         ▼
┌─────────────────────────┐
│   Gateway API Server    │
│   (gateway_api.py)      │
│   Port: 7070            │
└────────┬────────────────┘
         │ Proxies or Direct
         │
         ▼
┌─────────────────────────┐
│   Backend API Routes    │
│   (ros_routes.py)       │
│   Port: 8000            │
└────────┬────────────────┘
         │ Executes Docker Commands
         │ subprocess.run()
         ▼
┌─────────────────────────┐
│   Docker Host System    │
│   - docker-compose      │
│   - docker CLI          │
└────────┬────────────────┘
         │ Manages Containers
         ▼
┌─────────────────────────┐
│   ROS 2 Containers     │
│   - telemetry_publisher │
│   - telemetry_listener  │
└─────────────────────────┘
```

## Docker Compose Configuration

**Location:** `docker/ros2_robot/docker-compose.yml`

```yaml
services:
  telemetry_publisher:
    build:
      context: ../..
      dockerfile: docker/ros2_robot/Dockerfile
    image: omega_robot:latest
    network_mode: host
    command: ros2 run omega_robot telemetry_publisher

  telemetry_listener:
    build:
      context: ../..
      dockerfile: docker/ros2_robot/Dockerfile
    image: omega_robot:latest
    network_mode: host
    command: ros2 run omega_robot telemetry_listener
```

**Key Points:**
- Uses `network_mode: host` for ROS DDS communication
- Both containers share the same image (`omega_robot:latest`)
- Run different ROS nodes via `command` parameter
- Containers are named by Docker Compose: `{project}_{service}_1`

## Container Naming Convention

Docker Compose creates containers with the pattern:
- **Project name** (default: directory name) + **Service name** + **Instance number**
- Example: `ros2_robot_telemetry_publisher_1`

However, the backend code uses:
- `CONTAINER_PREFIX = "omega_robot"` 
- Looks for: `omega_robot_telemetry_publisher_1`

**⚠️ Potential Issue:** There's a mismatch between the docker-compose project name and the expected container prefix.

## Backend Implementation

### 1. Container Status Check (`GET /api/ros/status`)

```python
# Uses docker-compose to check status
docker-compose -f /path/to/docker-compose.yml ps --format json

# Fallback: Uses docker ps with filter
docker ps --filter name=omega_robot_telemetry_publisher
```

**Returns:**
- Container list with Name, State, Status
- ROS topics list (if containers are running)
- Compose file path

### 2. Container Control (`POST /api/ros/control`)

```python
# Start all containers
docker-compose -f /path/to/docker-compose.yml up -d

# Start specific service
docker-compose -f /path/to/docker-compose.yml up -d telemetry_publisher

# Stop all containers
docker-compose -f /path/to/docker-compose.yml stop

# Restart specific service
docker-compose -f /path/to/docker-compose.yml restart telemetry_publisher
```

**Actions:**
- `start` - Starts containers in detached mode (`-d`)
- `stop` - Stops containers gracefully
- `restart` - Restarts containers
- Optional `service` parameter for individual container control

### 3. Container Logs (`GET /api/ros/logs/{service}`)

```python
# Gets logs from container
docker logs --tail 50 omega_robot_telemetry_publisher_1
```

**Parameters:**
- `service` - Service name (e.g., `telemetry_publisher`)
- `tail` - Number of log lines (default: 50)

### 4. ROS Topics (`GET /api/ros/topics`)

```python
# Executes inside container
docker exec -i omega_robot_telemetry_publisher_1 \
  bash -c "source /opt/ros/humble/setup.bash && \
           source /root/omega_ws/install/setup.bash && \
           ros2 topic list"
```

**Note:** Requires containers to be running and ROS workspace to be built.

### 5. WebSocket Telemetry (`WebSocket /ws/ros/telemetry`)

```python
# Executes ros2 topic echo inside container
docker exec -i container_name \
  bash -c "ros2 topic echo /omega/telemetry"
```

**Process:**
1. Finds running container
2. Executes `ros2 topic echo` inside container
3. Streams output to WebSocket client
4. Restarts process if it exits

## Environment Variables

**Backend Configuration:**
- `ROS_DOCKER_COMPOSE_PATH` - Path to docker-compose.yml (default: `/home/omega1/Omega-Code/docker/ros2_robot/docker-compose.yml`)
- `ROS_WORKSPACE_PATH` - Workspace root (default: `/home/omega1/Omega-Code`)
- `CONTAINER_PREFIX` - Container name prefix (hardcoded: `omega_robot`)

**Docker Compose Environment:**
- `ROS_DOMAIN_ID` - ROS domain (default: 0)
- `RMW_IMPLEMENTATION` - DDS implementation (default: `rmw_cyclonedds_cpp`)
- `CYCLONEDDS_URI` - CycloneDDS config file path

## Container Requirements

### Backend Server Requirements:
1. **Docker installed** on host system
2. **Docker Compose** installed (or `docker compose` plugin)
3. **Permission to execute** Docker commands (user in `docker` group)
4. **Network access** to Docker daemon

### ROS Container Requirements:
1. **ROS 2 Humble** installed in container
2. **CycloneDDS** for DDS communication
3. **Built ROS workspace** (`/root/omega_ws`)
4. **ROS packages** installed (`omega_robot`)

## Common Issues & Solutions

### Issue 1: Container Name Mismatch

**Problem:** Backend looks for `omega_robot_*` but Docker Compose creates `ros2_robot_*`

**Solution:** Set docker-compose project name explicitly:
```bash
docker-compose -p omega_robot -f docker-compose.yml up -d
```

Or update backend to use correct naming:
```python
CONTAINER_PREFIX = "ros2_robot"  # Match docker-compose project name
```

### Issue 2: Permission Denied

**Problem:** Backend can't execute Docker commands

**Solution:**
```bash
# Add user to docker group
sudo usermod -aG docker $USER
# Log out and back in
```

### Issue 3: Docker Compose Not Found

**Problem:** `docker-compose` command not available

**Solution:** Use `docker compose` (newer syntax):
```python
cmd = ["docker", "compose", "-f", ROS_DOCKER_COMPOSE_PATH]
```

### Issue 4: Containers Not Found

**Problem:** Containers exist but backend can't find them

**Solution:** Check actual container names:
```bash
docker ps --format "{{.Names}}"
```

Update `CONTAINER_PREFIX` or use docker-compose project name.

## Testing the Integration

### 1. Manual Docker Compose Test
```bash
cd /home/omega1/Omega-Code/docker/ros2_robot
docker-compose up -d
docker-compose ps
docker-compose logs telemetry_publisher
```

### 2. Test Backend API
```bash
# Check status
curl http://localhost:7070/api/ros/status

# Start containers
curl -X POST http://localhost:7070/api/ros/control \
  -H "Content-Type: application/json" \
  -d '{"action": "start"}'

# Get logs
curl http://localhost:7070/api/ros/logs/telemetry_publisher
```

### 3. Test ROS Topics
```bash
# Execute inside container
docker exec -it ros2_robot_telemetry_publisher_1 \
  bash -c "source /opt/ros/humble/setup.bash && \
           source /root/omega_ws/install/setup.bash && \
           ros2 topic list"
```

## Security Considerations

⚠️ **Important:** The backend executes Docker commands with host system privileges.

**Recommendations:**
1. Run backend in isolated environment
2. Use Docker socket proxy for remote access
3. Validate container names before executing
4. Implement rate limiting on control endpoints
5. Add authentication/authorization for container control
6. Log all Docker command executions

## ROS Dashboard & Debugging

### Overview

The ROS 2 Dashboard (`/ros`) provides a comprehensive interface for managing ROS 2 Docker containers with built-in debugging tools that leverage browser DevTools.

### Features

**Core Features**:
- **ROS Management Panel**: Start/stop/restart containers, view logs, monitor status
- **Telemetry Visualization**: Real-time display of `/omega/telemetry` topic
- **Auto-refresh**: Configurable polling interval (1s, 2s, 5s, 10s)
- **WebSocket Monitoring**: Visual connection status with reconnect attempts

**Built-in Debug Tools**:

1. **Console Logging**
   - All actions are logged to the browser console with color-coded prefixes:
   - 🟢 **Green** `[ROS Debug]` - Info messages
   - 🔴 **Red** `[ROS Error]` - Errors
   - 🟡 **Yellow** `[ROS Warning]` - Warnings
   - 🔵 **Blue** `[ROS Network]` - API requests
   - 🟣 **Purple** `[ROS WebSocket]` - WebSocket events

2. **Network Monitoring**
   - Open DevTools → Network tab to see:
   - All API requests (`/api/ros/*`)
   - Request/response headers and bodies
   - Timing information (latency, duration)
   - Filter by "WS" to see WebSocket frames

3. **WebSocket Debugging**
   - Real-time WebSocket message inspection
   - Connection state monitoring
   - Reconnection attempt tracking
   - Message payload inspection

4. **In-Page Debug Panel**
   - Live debug log viewer (last 200 entries)
   - Color-coded by type (API, WebSocket, Error, Status)
   - Expandable data inspection
   - Timestamp for each entry

### Accessing the Dashboard

1. **From Header**: Click the "ROS" badge in the header
2. **Direct URL**: Navigate to `/ros`
3. **Quick Access**: Bookmark for easy access

### Debugging Workflow

1. Open browser DevTools (F12)
2. Navigate to `/ros` page
3. Check Console tab for debug messages
4. Check Network tab for API requests
5. Use in-page debug panel for quick reference

## Optional ROS Configuration

### Overview

ROS (Robot Operating System) features are **optional** and can be enabled or disabled via environment variables. The system will work perfectly fine without ROS installed.

### Environment Variables

**`ROS_ENABLED` (Default: `true`)**

Controls whether ROS features are enabled at all.

- **`true`** (default): ROS features are enabled if available
- **`false`**: ROS features are completely disabled, even if ROS is installed

**Example:**
```bash
# Disable ROS completely
export ROS_ENABLED=false

# Enable ROS (default)
export ROS_ENABLED=true
```

**`ROS_NATIVE_MODE` (Default: `false`)**

Controls whether to use native ROS2 integration (rclpy) instead of Docker containers.

- **`true`**: Use native ROS2 Python bindings (requires `rclpy` installed)
- **`false`**: Use Docker-based ROS2 (default)

**Example:**
```bash
# Enable native ROS2 mode
export ROS_NATIVE_MODE=true

# Use Docker ROS2 (default)
export ROS_NATIVE_MODE=false
```

### Operating Modes

**1. No ROS** (ROS_ENABLED=false or ROS not installed)
- ✅ All core robot features work (movement, sensors, lighting)
- ✅ WebSocket connections work normally
- ✅ REST API endpoints work normally
- ❌ ROS-specific endpoints return "disabled" status

**2. Docker ROS2** (ROS_ENABLED=true, ROS_NATIVE_MODE=false)
- ✅ ROS2 nodes run in Docker containers
- ✅ Isolated from host system
- ✅ Easy to manage and update
- ✅ Works on any system with Docker

**3. Native ROS2** (ROS_ENABLED=true, ROS_NATIVE_MODE=true)
- ✅ Direct ROS2 integration using rclpy
- ✅ Lower overhead than Docker
- ✅ Requires ROS2 installed on host system
- ✅ Better performance for development

### Checking ROS Status

```bash
# Check if ROS is enabled
curl http://localhost:8000/api/ros/status

# Response when disabled:
{
  "enabled": false,
  "message": "ROS features are disabled"
}

# Response when enabled:
{
  "enabled": true,
  "mode": "docker",
  "containers": [...]
}
```

## Future Improvements

1. **Fix Container Naming:** Align Docker Compose project name with backend expectations
2. **Use Docker SDK:** Replace `subprocess` calls with Docker Python SDK
3. **Add Health Checks:** Implement container health monitoring
4. **Better Error Handling:** More descriptive error messages
5. **Resource Limits:** Add CPU/memory limits to containers
6. **Container Logs Streaming:** Stream logs via WebSocket instead of polling
7. **Enhanced Debugging:** Add more detailed debug information in dashboard
8. **ROS Configuration UI:** Add UI for configuring ROS environment variables


