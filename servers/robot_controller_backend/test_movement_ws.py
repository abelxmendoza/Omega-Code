#!/usr/bin/env python3
"""
Quick test script to verify movement WebSocket server is working.
Tests basic movement commands: forward, backward, left, right, stop.
"""
import asyncio
import websockets
import json
import sys

MOVEMENT_WS_URL = "ws://localhost:8081"

async def test_movement():
    try:
        print(f"🔌 Connecting to {MOVEMENT_WS_URL}...")
        async with websockets.connect(MOVEMENT_WS_URL) as ws:
            print("✅ Connected!")
            
            # Wait for welcome message
            welcome = await ws.recv()
            print(f"📨 Welcome: {welcome[:100]}...")
            
            # Send ping
            ping_msg = {"type": "ping", "ts": 1234567890}
            await ws.send(json.dumps(ping_msg))
            print("📤 Sent ping")
            
            # Wait for pong
            pong = await ws.recv()
            print(f"📨 Pong: {pong}")
            
            # Test movement commands
            commands = [
                ("move-up", "Forward"),
                ("move-down", "Backward"),
                ("move-left", "Left"),
                ("move-right", "Right"),
                ("stop", "Stop"),
            ]
            
            print("\n🧪 Testing movement commands:")
            for cmd, desc in commands:
                print(f"  → {desc} ({cmd})")
                msg = {"command": cmd}
                await ws.send(json.dumps(msg))
                await asyncio.sleep(0.5)  # Small delay between commands
            
            print("\n✅ Test complete!")
            print("💡 Check the server logs to see if commands were received.")
            
    except ConnectionRefusedError:
        print(f"❌ Connection refused. Is the movement server running on {MOVEMENT_WS_URL}?")
        print("   Start it with: cd servers/robot-controller-backend && python3 movement/movement_ws_server.py")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(test_movement())


