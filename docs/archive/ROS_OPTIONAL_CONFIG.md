# ROS Optional Configuration Guide

The robot controller can now operate **with or without ROS**. This guide explains how to configure ROS features.

## Overview

ROS (Robot Operating System) features are **optional** and can be enabled or disabled via environment variables. The system will work perfectly fine without ROS installed.

## Environment Variables

### `ROS_ENABLED` (Default: `true`)

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

### `ROS_NATIVE_MODE` (Default: `false`)

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

## Operating Modes

### 1. **No ROS** (ROS_ENABLED=false or ROS not installed)

- ✅ All core robot features work (movement, sensors, lighting)
- ✅ WebSocket connections work normally
- ✅ REST API endpoints work normally
- ❌ ROS-specific endpoints return "disabled" status
- ❌ ROS dashboard shows "disabled" message
- ❌ Autonomy modes that require ROS won't use ROS features

**Use Case:** Development on macOS, testing without ROS, or when ROS is not needed.

### 2. **Docker ROS** (ROS_ENABLED=true, ROS_NATIVE_MODE=false)

- ✅ All core robot features work
- ✅ ROS2 runs in Docker containers
- ✅ ROS dashboard can manage containers
- ✅ ROS topics available via Docker exec

**Use Case:** Standard ROS deployment on Raspberry Pi or Jetson.

### 3. **Native ROS** (ROS_ENABLED=true, ROS_NATIVE_MODE=true)

- ✅ All core robot features work
- ✅ Direct ROS2 integration via rclpy
- ✅ Lower latency, no Docker overhead
- ✅ Real-time topic subscription/publishing

**Use Case:** High-performance ROS integration, development with ROS installed locally.

## Configuration Examples

### Development on macOS (No ROS)

```bash
# .env or environment
export ROS_ENABLED=false

# Start backend
cd servers/robot_controller_backend
python main_api.py
```

The backend will start successfully and all non-ROS features will work.

### Raspberry Pi with Docker ROS

```bash
# .env or environment
export ROS_ENABLED=true
export ROS_NATIVE_MODE=false
export ROS_DOCKER_COMPOSE_PATH=/home/pi/Omega-Code/docker/ros2_robot/docker-compose.yml

# Start backend
cd servers/robot_controller_backend
python main_api.py
```

### Raspberry Pi with Native ROS

```bash
# Source ROS2 first
source /opt/ros/rolling/setup.bash
source ~/omega_ws/install/setup.bash

# Set environment
export ROS_ENABLED=true
export ROS_NATIVE_MODE=true

# Start backend
cd servers/robot_controller_backend
python main_api.py
```

## API Behavior

### When ROS is Disabled

**GET `/api/ros/status`**
```json
{
  "containers": [],
  "topics": [],
  "mode": "disabled",
  "message": "ROS features are disabled. Set ROS_ENABLED=true to enable."
}
```

**POST `/api/ros/control`**
```json
{
  "detail": "ROS features are disabled. Set ROS_ENABLED=true to enable."
}
```
Returns HTTP 503.

**GET `/api/ros/topics`**
```json
{
  "topics": [],
  "message": "ROS features are disabled. Set ROS_ENABLED=true to enable."
}
```

**WebSocket `/api/ros/telemetry`**
Closes immediately with error message.

### When ROS is Enabled but Not Available

If `ROS_ENABLED=true` but ROS is not installed or rclpy is missing:

- Backend logs warnings but continues running
- ROS endpoints return empty results
- Frontend shows appropriate error messages
- All non-ROS features continue to work

## Frontend Behavior

The frontend gracefully handles ROS unavailability:

- **ROS Dashboard** (`/ros`): Shows "disabled" message when ROS is disabled
- **Status Hook** (`useROS2Status`): Returns `available: false` and error message
- **Management Panel**: Shows error banner when ROS is disabled
- **All other features**: Work normally regardless of ROS status

## Autonomy Modes

Autonomy modes that use ROS will automatically fall back to non-ROS implementations when ROS is unavailable:

- ✅ Basic autonomy modes work without ROS
- ✅ ROS-enhanced modes check for ROS availability
- ✅ System continues to function normally

## Troubleshooting

### Backend won't start because of ROS errors

**Solution:** Set `ROS_ENABLED=false` in your environment.

### ROS features not working

1. Check `ROS_ENABLED` is set to `true`
2. For native mode, ensure ROS2 is sourced: `source /opt/ros/rolling/setup.bash`
3. For Docker mode, ensure Docker is running and containers exist
4. Check backend logs for ROS initialization messages

### Frontend shows "ROS disabled" but ROS is installed

1. Verify backend has `ROS_ENABLED=true`
2. Restart backend after changing environment variables
3. Check browser console for API errors
4. Verify network connectivity to backend

## Migration Guide

### From ROS-Required to ROS-Optional

If you have existing code that assumes ROS is always available:

**Before:**
```python
from api.ros_native_bridge import get_bridge
bridge = get_bridge()
bridge.publish_twist("/cmd_vel", 1.0, 0.0)
```

**After:**
```python
from api.ros_native_bridge import get_bridge
bridge = get_bridge()
if bridge:
    bridge.publish_twist("/cmd_vel", 1.0, 0.0)
else:
    # Fallback to non-ROS movement
    pass
```

## Summary

- ✅ ROS is **completely optional** - system works without it
- ✅ Control ROS via `ROS_ENABLED` environment variable
- ✅ Choose Docker or Native mode via `ROS_NATIVE_MODE`
- ✅ Frontend gracefully handles ROS unavailability
- ✅ All non-ROS features work regardless of ROS status

