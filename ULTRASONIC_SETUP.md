# Ultrasonic Sensor Setup Guide

## Overview
This guide ensures the Go ultrasonic WebSocket server (`main_ultrasonic.go`) can communicate with the frontend UI.

## Go Server Configuration

### Default Settings
- **Port**: 8080 (configurable via `PORT_ULTRASONIC`)
- **Path**: `/ultrasonic` (configurable via `ULTRA_PATH`)
- **WebSocket URL**: `ws://host:8080/ultrasonic`

### Environment Variables (optional)
```bash
export PORT_ULTRASONIC=8080          # WebSocket port (default: 8080)
export ULTRA_PATH=/ultrasonic        # WebSocket path (default: /ultrasonic)
export ORIGIN_ALLOW=http://localhost:3000,http://192.168.1.107:3000  # CORS origins (empty = allow all)
export ULTRA_MEASURE_INTERVAL=1s     # Measurement interval (default: 1s)
export ULTRA_LOG_EVERY=5s            # Log frequency (default: 5s)
export ULTRA_LOG_DELTA_CM=10         # Log when change >= this (default: 10cm)
```

### Running on omega1
```bash
cd ~/Omega-Code/servers/robot-controller-backend/sensors
go run main_ultrasonic.go
```

Or with custom port/path:
```bash
PORT_ULTRASONIC=8080 ULTRA_PATH=/ultrasonic go run main_ultrasonic.go
```

## Frontend Configuration

### Environment Variables
The frontend expects these environment variables (in `.env.local` or `.env`):

```bash
# For Tailscale connection
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE=ws://100.93.225.61:8080/ultrasonic

# For LAN connection  
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN=ws://192.168.6.164:8080/ultrasonic

# For localhost
NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL=ws://localhost:8080/ultrasonic
```

### Network Profile
The frontend uses `NEXT_PUBLIC_NETWORK_PROFILE` to select the connection:
- `tailscale` → uses `_TAILSCALE` URL
- `lan` → uses `_LAN` URL  
- `local` → uses `_LOCAL` URL

## Data Format

### Go Server Sends:
```json
{
  "status": "success",
  "distance_cm": 25,
  "distance_m": 0.25,
  "distance_inch": 9.84,
  "distance_feet": 0.82
}
```

Or on error:
```json
{
  "status": "error",
  "error": "timeout"
}
```

Welcome message (on connect):
```json
{
  "status": "connected",
  "service": "ultrasonic",
  "message": "Ultrasonic WebSocket connection established",
  "ts": 1234567890
}
```

### Frontend Expects:
The frontend components (`SensorDashboard.tsx`, `UltrasonicSensorStatus.tsx`) parse:
- `distance_cm` (number)
- `distance_m` (number)
- `distance_inch` (number)
- `distance_feet` (number)

The frontend ignores the welcome message by checking `if (data.distance_cm !== undefined)`.

## Verification Steps

### 1. Start the Go server on omega1:
```bash
ssh omega1-tailscale
cd ~/Omega-Code/servers/robot-controller-backend/sensors
go run main_ultrasonic.go
```

Expected output:
```
🌐 Ultrasonic WS server listening on :8080/ultrasonic
```

### 2. Check server is accessible:
From your laptop:
```bash
# Test WebSocket connection (if you have wscat installed)
wscat -c ws://100.93.225.61:8080/ultrasonic

# Or test HTTP health endpoint
curl http://100.93.225.61:8080/healthz
```

### 3. Verify frontend can connect:
1. Make sure frontend has correct environment variables set
2. Open browser console and check for WebSocket connection logs
3. Look for `[ULTRASONIC] WebSocket connected` message
4. Check that sensor readings appear in the UI

## Troubleshooting

### Connection Issues
- **CORS errors**: Set `ORIGIN_ALLOW` environment variable on omega1
- **Connection refused**: Check firewall/port forwarding
- **Wrong port**: Verify `PORT_ULTRASONIC` matches frontend config

### No Data Displayed
- Check browser console for WebSocket errors
- Verify WebSocket URL matches server configuration
- Ensure `distance_cm` field is present in messages (check Network tab)

### Hardware Issues
- Verify GPIO pins: Trigger=GPIO27 (Pin 13), Echo=GPIO22 (Pin 15)
- Check sensor wiring and power
- Run `diagnostics.py` to test hardware

## Components That Display Ultrasonic Data

1. **SensorDashboard** (`src/components/sensors/SensorDashboard.tsx`)
   - Shows distance in cm, m, inches, feet
   - Connection status indicator

2. **UltrasonicSensorStatus** (`src/components/sensors/UltrasonicSensorStatus.tsx`)
   - Standalone component for ultrasonic readings

3. **Header** (`src/components/Header.tsx`)
   - Service status indicator (connected/disconnected)

4. **ServiceStatusBar** (`src/components/ServiceStatusBar.tsx`)
   - Overall service health indicator

## Notes

- The Go server sends measurements every 1 second by default
- The frontend automatically reconnects on disconnect
- The server responds to `{"type": "ping"}` with `{"type": "pong"}`
- All distance values are calculated from the same measurement (cm)

