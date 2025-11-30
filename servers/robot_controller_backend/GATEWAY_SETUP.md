# Gateway Service Setup Guide

## Overview

The Gateway API (`gateway_api.py`) is a FastAPI service that runs on port **7070** and provides:
- Performance metrics endpoints (`/api/performance/metrics`, `/api/performance/cache`)
- Video feed proxy (`/video_feed`)
- Network summary endpoints (`/api/net/summary`)
- WebSocket proxying to downstream services (movement, lighting, ultrasonic, line tracker)
- Latency endpoints

**The frontend requires this service to be running** - without it, you'll see:
- `ECONNREFUSED` errors on `localhost:7070`
- `503 Service Unavailable` errors
- `403 Forbidden` errors on latency endpoints
- Performance metrics not loading

## Quick Start

### 1. Install Dependencies

```bash
cd ~/Omega-Code/servers/robot_controller_backend

# Activate virtual environment
source venv/bin/activate  # or: direnv allow

# Install FastAPI and uvicorn if not already installed
pip install fastapi uvicorn httpx
```

### 2. Configure Environment Variables (Optional)

Create or edit `.env` file:

```bash
# Gateway port (default: 7070)
PORT=7070

# Downstream WebSocket services (optional - gateway will use mock/echo if not set)
DS_MOVE_WS=ws://127.0.0.1:8081          # Movement server
DS_LIGHT_WS=ws://127.0.0.1:8082/lighting # Lighting server
DS_ULTRA_WS=ws://127.0.0.1:8080/ultrasonic # Ultrasonic server
DS_LINE_WS=ws://127.0.0.1:8090/line-tracker # Line tracker server

# Video upstream (default: http://127.0.0.1:5000/video_feed)
VIDEO_UPSTREAM=http://127.0.0.1:5000/video_feed
```

### 3. Start the Gateway

**Option A: Using the startup script (recommended)**
```bash
cd ~/Omega-Code/servers/robot_controller_backend
./scripts/run_gateway.sh
```

**Option B: Manual start**
```bash
cd ~/Omega-Code/servers/robot_controller_backend
source venv/bin/activate
uvicorn servers.gateway_api:app --host 0.0.0.0 --port 7070
```

**Option C: With environment variables**
```bash
cd ~/Omega-Code/servers/robot_controller_backend
source venv/bin/activate
DS_MOVE_WS=ws://127.0.0.1:8081 \
DS_LIGHT_WS=ws://127.0.0.1:8082/lighting \
uvicorn servers.gateway_api:app --host 0.0.0.0 --port 7070
```

### 4. Verify It's Running

```bash
# Check if port is listening
sudo netstat -tlnp | grep 7070
# or
sudo ss -tlnp | grep 7070

# Test health endpoint
curl http://localhost:7070/health
# Should return: ok

# Test performance metrics
curl http://localhost:7070/api/performance/metrics
# Should return JSON with metrics

# Run diagnostic script
./scripts/check_gateway.sh
```

## Troubleshooting

### Port Already in Use

If port 7070 is already in use:

```bash
# Find what's using the port
sudo lsof -i :7070
# or
sudo netstat -tlnp | grep 7070

# Kill the process if needed
sudo kill <PID>
```

### Dependencies Missing

```bash
# Check if FastAPI is installed
python3 -c "import fastapi; print('FastAPI OK')"

# Check if uvicorn is installed
python3 -c "import uvicorn; print('uvicorn OK')"

# Install missing dependencies
pip install fastapi uvicorn httpx
```

### Connection Refused Errors

If you see `ECONNREFUSED` errors:

1. **Check if gateway is running:**
   ```bash
   ./scripts/check_gateway.sh
   ```

2. **Check firewall:**
   ```bash
   sudo ufw allow 7070/tcp
   ```

3. **Check Tailscale connectivity:**
   ```bash
   tailscale status
   curl http://100.93.225.61:7070/health  # Replace with your Tailscale IP
   ```

### 403 Forbidden Errors

403 errors on latency endpoints usually mean:
- The gateway is running but the upstream video service isn't
- Check `VIDEO_UPSTREAM` environment variable
- Verify video server is running on the configured port

### 503 Service Unavailable

503 errors mean:
- Gateway is not running
- Gateway is running but can't reach downstream services
- Check downstream service URLs in `.env` file

## Running as a Service (Optional)

### Using systemd

Create `/etc/systemd/system/omega-gateway.service`:

```ini
[Unit]
Description=Omega Gateway API Service
After=network.target

[Service]
Type=simple
User=omega1
WorkingDirectory=/home/omega1/Omega-Code/servers/robot_controller_backend
Environment="PATH=/home/omega1/Omega-Code/servers/robot_controller_backend/venv/bin"
ExecStart=/home/omega1/Omega-Code/servers/robot_controller_backend/venv/bin/uvicorn servers.gateway_api:app --host 0.0.0.0 --port 7070
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Then:
```bash
sudo systemctl daemon-reload
sudo systemctl enable omega-gateway
sudo systemctl start omega-gateway
sudo systemctl status omega-gateway
```

## Integration with Other Services

The gateway proxies to these downstream services:

| Service | Port | WebSocket Path | Environment Variable |
|---------|------|----------------|---------------------|
| Movement | 8081 | `/` | `DS_MOVE_WS` |
| Lighting | 8082 | `/lighting` | `DS_LIGHT_WS` |
| Ultrasonic | 8080 | `/ultrasonic` | `DS_ULTRA_WS` |
| Line Tracker | 8090 | `/line-tracker` | `DS_LINE_WS` |

If downstream services aren't configured, the gateway will use mock/echo mode for testing.

## Frontend Configuration

The frontend expects the gateway at:
- **Tailscale**: `http://100.93.225.61:7070` (or your Tailscale IP)
- **LAN**: `http://192.168.6.164:7070` (or your LAN IP)
- **Local**: `http://localhost:7070`

These are configured in `ui/robot-controller-ui/src/utils/unifiedNetworkManager.ts`.

## Next Steps

1. Start the gateway service
2. Verify it's running with `./scripts/check_gateway.sh`
3. Test endpoints from frontend
4. Check browser console for any remaining errors
5. Configure downstream services if needed

