# Connectivity Issues - Resolution Summary

## Issues Identified

1. **ECONNREFUSED on port 7070** - Gateway service not running
2. **503 Service Unavailable** - Gateway endpoints unavailable
3. **403 Forbidden** - Latency endpoints failing
4. **Performance metrics not loading** - Gateway API not responding

## Root Cause

The **Gateway API service** (`gateway_api.py`) on port **7070** was not running. This service is required for:
- Performance metrics (`/api/performance/metrics`)
- Cache statistics (`/api/performance/cache`)
- Video feed proxy (`/video_feed`)
- Network summary (`/api/net/summary`)
- Latency endpoints
- WebSocket proxying to downstream services

## Solution Applied

### 1. Installed Required Dependencies

```bash
cd ~/Omega-Code/servers/robot_controller_backend
source venv/bin/activate
pip install fastapi uvicorn httpx psutil
```

### 2. Configured Downstream Services

Added to `.env`:
```bash
DS_MOVE_WS=ws://127.0.0.1:8081
DS_LIGHT_WS=ws://127.0.0.1:8082/lighting
DS_ULTRA_WS=ws://127.0.0.1:8080/ultrasonic
```

### 3. Started Gateway Service

```bash
cd ~/Omega-Code/servers/robot_controller_backend
./scripts/run_gateway.sh
```

Or manually:
```bash
source venv/bin/activate
uvicorn servers.gateway_api:app --host 0.0.0.0 --port 7070
```

## Verification

✅ Gateway is running on port 7070
✅ Health endpoint responding: `http://localhost:7070/health` → `ok`
✅ Performance metrics endpoint working: `http://localhost:7070/api/performance/metrics`
✅ All dependencies installed

## Diagnostic Tools Created

1. **`scripts/check_gateway.sh`** - Diagnostic script to verify gateway status
2. **`GATEWAY_SETUP.md`** - Comprehensive setup guide
3. **`CONNECTIVITY_FIXES.md`** - This document

## Running Gateway as a Service

To keep the gateway running after logout, use `nohup` or systemd:

```bash
# Using nohup
cd ~/Omega-Code/servers/robot_controller_backend
nohup ./scripts/run_gateway.sh > gateway.log 2>&1 &

# Check if running
pgrep -f gateway_api

# View logs
tail -f gateway.log
```

## Frontend Configuration

The frontend expects the gateway at:
- **Tailscale**: `http://100.93.225.61:7070`
- **LAN**: `http://192.168.6.164:7070`
- **Local**: `http://localhost:7070`

These are configured in `ui/robot-controller-ui/src/utils/unifiedNetworkManager.ts`.

## Next Steps

1. ✅ Gateway service started
2. ✅ Dependencies installed
3. ✅ Endpoints verified
4. 🔄 Test frontend - performance metrics should now load
5. 🔄 Verify video feed proxy works
6. 🔄 Check network summary endpoints

## Troubleshooting

If issues persist:

1. **Check gateway status:**
   ```bash
   ./scripts/check_gateway.sh
   ```

2. **Check gateway logs:**
   ```bash
   tail -f gateway.log
   ```

3. **Restart gateway:**
   ```bash
   pkill -f gateway_api
   ./scripts/run_gateway.sh
   ```

4. **Verify port is accessible:**
   ```bash
   curl http://localhost:7070/health
   curl http://100.93.225.61:7070/health  # Tailscale IP
   ```

