#!/bin/bash
# Quick connection test script for movement WebSocket server

echo "=== Movement WebSocket Server Connection Test ==="
echo ""

# Check if server is running
echo "1. Checking if server is listening on port 8081..."
if netstat -tuln 2>/dev/null | grep -q ":8081" || ss -tuln 2>/dev/null | grep -q ":8081"; then
    echo "   ✅ Port 8081 is listening"
else
    echo "   ❌ Port 8081 is NOT listening - server may not be running"
fi

echo ""
echo "2. Checking environment variables..."
if [ -f "../.env" ]; then
    echo "   ✅ .env file exists"
    echo ""
    echo "   Current ORIGIN_ALLOW setting:"
    grep "^ORIGIN_ALLOW" ../.env 2>/dev/null || echo "   (not set)"
    echo ""
    echo "   Current ORIGIN_ALLOW_NO_HEADER setting:"
    grep "^ORIGIN_ALLOW_NO_HEADER" ../.env 2>/dev/null || echo "   (not set)"
else
    echo "   ⚠️  .env file not found at ../.env"
fi

echo ""
echo "3. Testing WebSocket connection locally..."
python3 << 'EOF'
import asyncio
import websockets
import sys

async def test_connection():
    try:
        uri = "ws://localhost:8081/"
        print(f"   Attempting to connect to {uri}...")
        async with websockets.connect(uri) as ws:
            print("   ✅ Connection successful!")
            await ws.send('{"type":"ping"}')
            response = await ws.recv()
            print(f"   ✅ Received response: {response[:100]}")
    except Exception as e:
        print(f"   ❌ Connection failed: {e}")
        sys.exit(1)

asyncio.run(test_connection())
EOF

echo ""
echo "=== Test Complete ==="

