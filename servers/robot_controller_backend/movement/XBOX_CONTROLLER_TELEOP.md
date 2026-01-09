# Xbox Controller Teleoperation Implementation

## Summary

This implementation provides **GTA-style Xbox controller teleoperation** with trigger-based gas/brake controls, pivot turns, camera control, and all essential robot functions mapped to controller buttons.

## Configuration Modes

The controller supports multiple connection configurations:

- **Remote Mode**: Controller on laptop, commands sent over network to robot
- **Local Mode**: Controller on robot, commands processed locally (no network)
- **Auto Mode**: Automatically detects which mode to use based on network connectivity

**📖 See [XBOX_CONTROLLER_CONFIGURATIONS.md](XBOX_CONTROLLER_CONFIGURATIONS.md) for detailed setup instructions for:**
- Wired test (Controller → Laptop → Ethernet → Robot)
- Direct robot (Controller → Robot, no network)
- Wireless controller → Laptop → Robot
- Wireless controller → Robot directly

## What Was Already Implemented

1. **`test_movement.py`**: Uses `evdev` library but only handles keyboard input, not gamepad
2. **Network Architecture**: 
   - WebSocket server on port 8081 for discrete movement commands
   - ROS2 bridge supports `cmd_vel` (Twist messages) with linear/angular velocity
3. **Movement Commands**: Discrete commands (forward, backward, left, right, stop) via WebSocket

## What Was Added

### 1. `xbox_controller_teleop.py` (GTA-Style Controls - UDP Version)
**Location**: `servers/robot_controller_backend/movement/xbox_controller_teleop.py`

**Features**:
- **GTA-style driving controls** (triggers for gas/brake)
- Detects and reads Xbox controller using `evdev`
- **Right Trigger**: Forward (gas) - acts as deadman
- **Left Trigger**: Backward (brake/reverse)
- **Left Stick X**: Pivot turns (in-place rotation left/right)
- **D-Pad**: Camera control (up/down/left/right)
- **B Button**: Horn/Buzzer toggle
- **X Button**: Lights on/off toggle
- **A Button**: Emergency stop
- Sends commands over UDP to robot at `192.168.50.2:8888`
- Safe defaults: Zero velocity if triggers released or connection lost
- Automatic controller detection

**Usage**:
```bash
# Basic usage (defaults: 192.168.50.2:8888)
python3 xbox_controller_teleop.py

# Custom robot IP and port
python3 xbox_controller_teleop.py --robot-ip 192.168.50.2 --port 8888
```

**GTA-Style Controller Mapping**:
- **Right Trigger**: Forward (gas) - hold to move
- **Left Trigger**: Backward (brake/reverse)
- **Left Stick X**: Pivot turns (left/right in place)
- **D-Pad ↑**: Camera tilt up
- **D-Pad ↓**: Camera tilt down
- **D-Pad ←**: Camera pan left
- **D-Pad →**: Camera pan right
- **B Button**: Horn/Buzzer toggle
- **X Button**: Lights on/off toggle
- **A Button**: Emergency stop

### 2. `xbox_controller_teleop_ws.py` (Alternative - WebSocket Version)
**Location**: `servers/robot_controller_backend/movement/xbox_controller_teleop_ws.py`

**Features**:
- Same controller handling as UDP version
- Sends commands via WebSocket (compatible with existing ROS2 bridge)
- Connects to robot WebSocket server (default: `192.168.50.2:8081`)

**Usage**:
```bash
python3 xbox_controller_teleop_ws.py --robot-ip 192.168.50.2 --port 8081
```

### 3. `udp_velocity_receiver.py` (Robot-Side Receiver)
**Location**: `servers/robot_controller_backend/movement/udp_velocity_receiver.py`

**Purpose**: Runs on the robot (Raspberry Pi) to receive UDP velocity commands and forward them to the movement system.

**Features**:
- Listens on UDP port 8888 for velocity commands
- Forwards to WebSocket movement server (optional)
- Forwards to ROS2 `/cmd_vel` topic (optional)
- Auto-stop on timeout (0.5s without commands)

**Usage** (on robot):
```bash
# Forward to WebSocket movement server
python3 udp_velocity_receiver.py --forward-ws

# Forward to ROS2
python3 udp_velocity_receiver.py --forward-ros

# Forward to both
python3 udp_velocity_receiver.py --forward-ws --forward-ros
```

## Input Flow

### UDP Version (Recommended for Low Latency)
```
Xbox Controller (Laptop)
    ↓ (evdev reads gamepad)
xbox_controller_teleop.py
    ↓ (converts axes to linear/angular velocity)
    ↓ (checks deadman switch)
UDP Socket → 192.168.50.2:8888
    ↓ (robot receives)
udp_velocity_receiver.py
    ↓ (forwards to)
Movement System (WebSocket/ROS2)
    ↓
Robot Hardware
```

### WebSocket Version
```
Xbox Controller (Laptop)
    ↓ (evdev reads gamepad)
xbox_controller_teleop_ws.py
    ↓ (converts axes to linear/angular velocity)
    ↓ (checks deadman switch)
WebSocket → 192.168.50.2:8081
    ↓ (robot receives)
ROS2 Bridge / Movement Server
    ↓
Robot Hardware
```

## Safety Features

1. **Deadman Switch**: Robot only moves when triggers are actively pressed (GTA-style)
   - Right trigger (gas) or left trigger (brake) acts as deadman
   - Natural safety: robot stops when you release triggers
   
2. **Safe Defaults**:
   - Zero velocity sent if triggers released
   - Zero velocity sent on connection loss
   - Zero velocity sent on controller disconnect
   - Auto-stop on timeout (receiver side)
   - Emergency stop button (A button) for immediate halt

3. **Velocity Limits**:
   - Linear velocity: ±1.0 m/s (configurable)
   - Angular velocity: ±1.0 rad/s (configurable)
   - Dead zone: 0.1 (ignores small stick movements)

## Configuration

### Velocity Limits
Edit constants in the scripts:
```python
MAX_LINEAR_VELOCITY = 1.0   # m/s
MAX_ANGULAR_VELOCITY = 1.0   # rad/s
DEAD_ZONE = 0.1              # Ignore small movements
COMMAND_RATE = 20            # Hz (commands per second)
```

### Network Settings
- **UDP**: Default port 8888, configurable via `--port`
- **WebSocket**: Default port 8081, configurable via `--port`
- **Robot IP**: Default `192.168.50.2`, configurable via `--robot-ip`

## Dependencies

**On Laptop (Controller Side)**:
```bash
pip install evdev
# For WebSocket version:
pip install websocket-client
```

**On Robot (Receiver Side)**:
```bash
pip install evdev  # Only if using controller on robot
# For WebSocket forwarding:
pip install websocket-client
# For ROS2 forwarding:
# ROS2 must be installed and configured
```

## Testing

1. **Check Controller Detection**:
   ```bash
   # List input devices
   ls /dev/input/
   
   # Test controller
   python3 -c "from evdev import list_devices, InputDevice; print([InputDevice(d).name for d in list_devices()])"
   ```

2. **Test UDP Connection**:
   ```bash
   # On laptop: Send test command
   echo '{"type":"cmd_vel","linear":{"x":0.5,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' | nc -u 192.168.50.2 8888
   
   # On robot: Run receiver in verbose mode
   python3 udp_velocity_receiver.py --forward-ws
   ```

3. **Test Controller**:
   ```bash
   # On laptop
   python3 xbox_controller_teleop.py --robot-ip 192.168.50.2
   ```

## Integration with Existing Code

### Minimal Changes
- **No changes** to existing movement server or hardware code
- New scripts are standalone and optional
- Can run alongside existing keyboard controller

### Future ROS2 Integration
The velocity commands are already in ROS2 `cmd_vel` format:
```json
{
  "type": "cmd_vel",
  "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.3}
}
```

This matches the format expected by:
- `ros/src/omega_robot/omega_robot/robot_controller.py` (ROS2 node)
- `servers/robot_controller_backend/api/ros_web_bridge.py` (WebSocket bridge)

## Files Modified/Added

### New Files
1. `servers/robot_controller_backend/movement/xbox_controller_teleop.py` - UDP teleop client
2. `servers/robot_controller_backend/movement/xbox_controller_teleop_ws.py` - WebSocket teleop client
3. `servers/robot_controller_backend/movement/udp_velocity_receiver.py` - Robot-side UDP receiver
4. `servers/robot_controller_backend/movement/XBOX_CONTROLLER_TELEOP.md` - This documentation

### Existing Files (No Changes)
- `servers/robot_controller_backend/movement/test_movement.py` - Still works for keyboard
- `servers/robot_controller_backend/movement/movement_ws_server.py` - Unchanged
- `servers/robot_controller_backend/movement/keyboard_controller.py` - Unchanged

## Troubleshooting

### Controller Not Detected
```bash
# Check if controller is connected
ls /dev/input/

# Check permissions (may need sudo or add user to input group)
sudo usermod -a -G input $USER
# Log out and back in

# Test with evtest
sudo apt install evtest
sudo evtest
```

### Network Connection Issues
```bash
# Test network connectivity
ping 192.168.50.2

# Test UDP port
nc -u -v 192.168.50.2 8888

# Check firewall
sudo ufw status
```

### Velocity Not Applied
- Verify deadman switch is held
- Check receiver is running on robot
- Verify forwarding is enabled (`--forward-ws` or `--forward-ros`)
- Check robot logs for errors

## Next Steps

1. **Test on actual hardware** with robot at 192.168.50.2
2. **Tune velocity limits** based on robot capabilities
3. **Adjust dead zone** if controller is too sensitive
4. **Integrate with ROS2** if using ROS2 stack (already compatible)
5. **Add logging** for debugging if needed
