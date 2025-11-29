# Quick Start Guide for Testing Robot Movement

## Prerequisites
- Python 3 with websockets library
- Frontend running on port 3000
- Backend services configured

## Steps to Test Movement

### 1. Start Movement WebSocket Server
```bash
cd servers/robot_controller_backend
source venv/bin/activate  # if using venv
export PORT_MOVEMENT=8081
export ROBOT_SIM=0  # Set to 1 for simulation mode (no hardware)
export ORIGIN_ALLOW=http://localhost:3000
python3 movement/movement_ws_server.py
```

Or use the script:
```bash
cd servers/robot_controller_backend
./scripts/run_movement_full.sh
```

### 2. Test WebSocket Connection
```bash
python3 test_movement_ws.py
```

### 3. Open UI and Test
1. Open http://localhost:3000 in browser
2. Check header for "Pi:" connection status
3. Use WASD keys or click buttons to control movement
4. Check CommandLog panel for command confirmations

## Troubleshooting

### Connection Refused
- Check if movement server is running: `lsof -i :8081`
- Verify port 8081 is not blocked by firewall
- Check server logs for errors

### WebSocket Connection Issues
- Verify ORIGIN_ALLOW includes your UI URL
- Check browser console for WebSocket errors
- Ensure UI is using correct WebSocket URL (ws://localhost:8081)

### Movement Not Working
- Check if ROBOT_SIM=1 (simulation mode won't move hardware)
- Verify GPIO/hardware permissions if running on Pi
- Check motor controller logs

## Test Commands

The UI sends these commands:
- `move-up` - Forward
- `move-down` - Backward  
- `move-left` - Left
- `move-right` - Right
- `stop` - Stop motors
- `set-speed` - Set speed (value: 0-4095)

## Environment Variables

- `PORT_MOVEMENT` - WebSocket server port (default: 8081)
- `ROBOT_SIM` - Simulation mode (0=real hardware, 1=simulation)
- `ORIGIN_ALLOW` - Allowed origins for CORS (comma-separated)


