# Xbox Controller Configuration Guide

This guide explains how to use the Xbox controller in different configurations.

## Quick Start

### Auto-Detection (Recommended)
```bash
python3 xbox_controller_teleop.py
```
The script will automatically detect whether to use local or remote mode based on network connectivity.

## Configuration Scenarios

### 1. Wired Test (Controller on Laptop → Ethernet → Robot)
**Best for:** Initial testing, low latency, reliable connection

**Setup:**
- Connect Xbox controller via USB to laptop
- Connect laptop to robot via Ethernet cable
- Robot IP: `192.168.50.2` (default)

**Command:**
```bash
# Auto-detect (will use remote mode)
python3 xbox_controller_teleop.py

# Or force remote mode
python3 xbox_controller_teleop.py --mode remote --robot-ip 192.168.50.2
```

**On Robot:**
```bash
# Start UDP receiver (if not using WebSocket directly)
python3 udp_velocity_receiver.py --forward-ws
```

---

### 2. Direct to Robot (Controller on Robot)
**Best for:** Standalone operation, no laptop needed, portable

**Setup:**
- Connect Xbox controller via USB to robot (Raspberry Pi)
- No network connection needed
- Movement server must be running on robot

**Command (on robot):**
```bash
# Force local mode
python3 xbox_controller_teleop.py --mode local
```

**Prerequisites:**
- `movement_ws_server.py` must be running on robot
- Install: `pip install websocket-client`

**Start movement server:**
```bash
# On robot
cd servers/robot_controller_backend/movement
python3 movement_ws_server.py
```

---

### 3. Wireless Controller → Laptop → Robot
**Best for:** Wireless controller, laptop in range, flexible positioning

**Setup:**
- Pair Xbox controller via Bluetooth to laptop
- Connect laptop to robot via WiFi or Ethernet
- Controller appears as `/dev/input/event*` (same as USB)

**Command:**
```bash
# Auto-detect (will use remote mode if robot reachable)
python3 xbox_controller_teleop.py --robot-ip <robot-ip>

# For WiFi connection, use robot's WiFi IP
python3 xbox_controller_teleop.py --robot-ip 192.168.1.100
```

**Pairing Bluetooth Controller:**
```bash
# On laptop (Linux)
bluetoothctl
scan on
# Find your controller, note MAC address
pair <MAC_ADDRESS>
connect <MAC_ADDRESS>
trust <MAC_ADDRESS>
```

**Verify controller:**
```bash
ls /dev/input/
# Should see event* devices including your controller
```

---

### 4. Wireless Controller → Robot Directly
**Best for:** Fully wireless, standalone, maximum portability

**Setup:**
- Pair Xbox controller via Bluetooth to robot
- No laptop needed
- Movement server must be running on robot

**Command (on robot):**
```bash
# Force local mode
python3 xbox_controller_teleop.py --mode local
```

**Pairing on Robot:**
```bash
# On robot (Raspberry Pi)
bluetoothctl
scan on
# Find controller, note MAC address
pair <MAC_ADDRESS>
connect <MAC_ADDRESS>
trust <MAC_ADDRESS>
```

---

## Mode Detection

### Auto Mode (Default)
The script automatically detects the mode:
- **Remote**: If robot IP is reachable → sends commands over network
- **Local**: If robot IP not reachable → processes commands locally

### Manual Mode Selection

**Force Local Mode:**
```bash
python3 xbox_controller_teleop.py --mode local
```

**Force Remote Mode:**
```bash
python3 xbox_controller_teleop.py --mode remote --robot-ip 192.168.50.2
```

---

## Network Configurations

### Ethernet (Wired)
- **Laptop IP**: Usually `192.168.50.1` or auto-assigned
- **Robot IP**: `192.168.50.2` (default)
- **Port**: `8888` (UDP)

### WiFi
- **Robot IP**: Check with `hostname -I` on robot
- **Port**: `8888` (UDP)
- **Example**: `python3 xbox_controller_teleop.py --robot-ip 192.168.1.100`

### Direct Connection (Local)
- **No network needed**
- Uses WebSocket: `ws://127.0.0.1:8081`
- Requires `movement_ws_server.py` running locally

---

## Troubleshooting

### Controller Not Detected
```bash
# List input devices
ls /dev/input/

# Test with evtest
sudo evtest
# Select your controller from the list
```

**Permissions:**
```bash
# Add user to input group
sudo usermod -a -G input $USER
# Log out and back in
```

### Network Connection Issues

**Test connectivity:**
```bash
# Ping robot
ping 192.168.50.2

# Test UDP port
nc -u -v 192.168.50.2 8888
```

**Check robot IP:**
```bash
# On robot
hostname -I
ip addr show
```

### Local Mode Not Working

**Check movement server:**
```bash
# On robot
ps aux | grep movement_ws_server
# Or start it
cd servers/robot_controller_backend/movement
python3 movement_ws_server.py
```

**Test WebSocket connection:**
```bash
# On robot
python3 -c "import websocket; ws = websocket.create_connection('ws://127.0.0.1:8081'); print('Connected!')"
```

### Bluetooth Issues

**Check Bluetooth:**
```bash
# On laptop/robot
bluetoothctl
devices  # List paired devices
info <MAC_ADDRESS>  # Check device info
```

**Re-pair controller:**
```bash
bluetoothctl
remove <MAC_ADDRESS>  # Remove old pairing
scan on
pair <MAC_ADDRESS>
connect <MAC_ADDRESS>
```

---

## Quick Reference

| Configuration | Controller Location | Network | Mode | Command |
|--------------|---------------------|---------|------|---------|
| Wired Test | Laptop (USB) | Ethernet | remote | `python3 xbox_controller_teleop.py` |
| Direct Robot | Robot (USB) | None | local | `python3 xbox_controller_teleop.py --mode local` |
| Wireless Laptop | Laptop (BT) | WiFi/Ethernet | remote | `python3 xbox_controller_teleop.py --robot-ip <ip>` |
| Wireless Robot | Robot (BT) | None | local | `python3 xbox_controller_teleop.py --mode local` |

---

## Testing Checklist

### Before Testing:
- [ ] Controller detected (`ls /dev/input/`)
- [ ] Network connectivity (if remote mode)
- [ ] Movement server running (if local mode)
- [ ] Robot powered on and ready

### Test Sequence:
1. **Wired test first** (most reliable)
   - Controller → Laptop → Ethernet → Robot
2. **Direct robot test** (verify local mode)
   - Controller → Robot (USB)
3. **Wireless test** (verify Bluetooth)
   - Controller → Laptop (BT) → Network → Robot
4. **Fully wireless** (final test)
   - Controller → Robot (BT)

---

## Tips

- **Start with wired** for initial testing
- **Use auto mode** for convenience
- **Check controller permissions** if not detected
- **Verify network IPs** before remote mode
- **Test Bluetooth pairing** separately before wireless tests
- **Keep movement server running** for local mode
