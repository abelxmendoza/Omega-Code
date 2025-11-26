# Gateway Lighting Setup Guide

## Overview
The gateway (port 7070) can proxy lighting WebSocket connections to the direct lighting server (port 8082). This provides a single entry point for all services.

## Backend Configuration (Raspberry Pi)

To enable gateway proxying for lighting, set this environment variable when starting the gateway:

```bash
# In servers/robot-controller-backend/.env or when running gateway:
DS_LIGHT_WS=ws://127.0.0.1:8082/lighting
```

### Option 1: Add to .env file
```bash
cd ~/Omega-Code/servers/robot-controller-backend
echo "DS_LIGHT_WS=ws://127.0.0.1:8082/lighting" >> .env
```

### Option 2: Set when starting gateway
```bash
DS_LIGHT_WS=ws://127.0.0.1:8082/lighting python3 -m uvicorn servers.gateway_api:app --port 7070
```

## Frontend Configuration (Already Done)

The UI `.env.local` is configured to connect directly to port 8082. If you want to use the gateway instead, update to:

```bash
# Gateway URLs (port 7070)
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE=ws://100.93.225.61:7070/ws/lighting
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN=ws://192.168.6.164:7070/ws/lighting
NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL=ws://192.168.6.164:7070/ws/lighting
```

## Current Setup (Direct Connection)

Currently configured for **direct connection** (bypasses gateway):
- ✅ Lower latency
- ✅ Simpler debugging
- ✅ Works independently of gateway

## Architecture

```
UI (Mac)
  ↓
  ├─→ Gateway (7070) → /ws/lighting → DS_LIGHT_WS → Lighting Server (8082)
  └─→ Direct Connection → Lighting Server (8082) [CURRENT]
```

## Testing

1. **Test direct connection** (current):
   ```bash
   # On Mac, test connection:
   wscat -c ws://192.168.6.164:8082/lighting
   ```

2. **Test gateway** (if configured):
   ```bash
   # On Mac, test gateway:
   wscat -c ws://192.168.6.164:7070/ws/lighting
   ```

## Benefits of Each Approach

### Direct Connection (Current)
- ✅ Lower latency (one less hop)
- ✅ Simpler architecture
- ✅ Works if gateway is down
- ❌ Requires managing multiple ports

### Gateway Proxy
- ✅ Single entry point (port 7070)
- ✅ Centralized management
- ✅ Easier firewall rules
- ❌ Slightly higher latency
- ❌ Requires gateway to be running

## Recommendation

**Use direct connection** (current setup) for now since:
1. Lighting server is already running standalone
2. Lower latency for real-time lighting control
3. Simpler debugging

You can always switch to gateway later if you want centralized management.

