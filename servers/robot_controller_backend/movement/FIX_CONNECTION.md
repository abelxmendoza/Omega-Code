# Fix Movement WebSocket Connection Issues

## Problem
The frontend cannot connect to the movement WebSocket server on port 8081. The server logs show `ORIGIN_ALLOW=(none)` but connections are still being rejected.

## Solution

### Step 1: Update `.env` file on Raspberry Pi

SSH into your Raspberry Pi and edit the `.env` file:

```bash
cd ~/Omega-Code/servers/robot_controller_backend
nano .env
```

Add or update these lines:

```bash
# Allow connections from frontend (add your frontend URLs)
ORIGIN_ALLOW=http://localhost:3000,http://omega1-1.hartley-ghost.ts.net:3000,http://192.168.1.107:3000

# Allow connections without Origin header (for development/testing)
ORIGIN_ALLOW_NO_HEADER=1
```

**OR** to allow ALL connections (less secure, but good for development):

```bash
# Leave ORIGIN_ALLOW empty or unset to allow all connections
# ORIGIN_ALLOW=

# Allow connections without Origin header
ORIGIN_ALLOW_NO_HEADER=1
```

### Step 2: Restart the Movement Server

After updating `.env`, restart the movement server:

```bash
# Stop the current server (Ctrl+C if running in terminal)
# Then restart:
cd ~/Omega-Code/servers/robot_controller_backend/movement
python3 movement_ws_server.py
```

### Step 3: Verify Server Configuration

Check the server startup logs. You should see:

```
listening on ws://0.0.0.0:8081
ORIGIN_ALLOW=(none) ALLOW_NO_ORIGIN=True SIM_MODE=False
```

If `ORIGIN_ALLOW=(none)` and `ALLOW_NO_ORIGIN=True`, the server should accept all connections.

### Step 4: Test Connection

From your local machine (where the frontend is running), test the connection:

```bash
# Test WebSocket connection
python3 -c "
import asyncio
import websockets

async def test():
    try:
        async with websockets.connect('ws://omega1-1.hartley-ghost.ts.net:8081/') as ws:
            await ws.send('{\"type\":\"ping\"}')
            response = await ws.recv()
            print(f'✅ Connected! Response: {response}')
    except Exception as e:
        print(f'❌ Connection failed: {e}')

asyncio.run(test())
"
```

### Step 5: Check Firewall

If connections still fail, check if port 8081 is open:

```bash
# On Raspberry Pi
sudo ufw status
sudo ufw allow 8081/tcp
```

### Step 6: Check Network Connectivity

Verify you can reach the server:

```bash
# From your local machine
ping omega1-1.hartley-ghost.ts.net
telnet omega1-1.hartley-ghost.ts.net 8081
```

## Troubleshooting

### If connections still fail:

1. **Check server logs** - Look for "reject origin" messages
2. **Verify .env is loaded** - The server should print `ORIGIN_ALLOW=...` on startup
3. **Check browser console** - Look for specific WebSocket error messages
4. **Test with curl** - Try: `curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" -H "Origin: http://localhost:3000" http://omega1-1.hartley-ghost.ts.net:8081/`

### Common Issues:

- **`.env` file not in correct location** - Must be in `servers/robot_controller_backend/.env`
- **Server not restarted** - Changes to `.env` require server restart
- **Firewall blocking port** - Check `ufw` or `iptables` rules
- **Wrong port** - Verify server is listening on port 8081

## Quick Fix Script

Run this on the Raspberry Pi to quickly fix the `.env` file:

```bash
cd ~/Omega-Code/servers/robot_controller_backend

# Backup current .env
cp .env .env.backup

# Add/update ORIGIN_ALLOW settings
if grep -q "^ORIGIN_ALLOW" .env; then
    sed -i 's|^ORIGIN_ALLOW=.*|ORIGIN_ALLOW=http://localhost:3000,http://omega1-1.hartley-ghost.ts.net:3000|' .env
else
    echo "ORIGIN_ALLOW=http://localhost:3000,http://omega1-1.hartley-ghost.ts.net:3000" >> .env
fi

# Add/update ORIGIN_ALLOW_NO_HEADER
if grep -q "^ORIGIN_ALLOW_NO_HEADER" .env; then
    sed -i 's|^ORIGIN_ALLOW_NO_HEADER=.*|ORIGIN_ALLOW_NO_HEADER=1|' .env
else
    echo "ORIGIN_ALLOW_NO_HEADER=1" >> .env
fi

echo "✅ .env file updated. Please restart the movement server."
```

