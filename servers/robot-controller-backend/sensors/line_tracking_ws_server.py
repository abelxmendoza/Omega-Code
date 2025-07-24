"""
# File: /Omega-Code/servers/robot-controller-backend/sensors/line_tracking_ws_server.py
# Summary:
WebSocket server for streaming line tracking sensor states to the UI.
Uses 3 IR sensors on GPIO 14, 15, 23 (BCM).
"""

import asyncio
import websockets
import json
import RPi.GPIO as GPIO
import time

# Pin config (edit if needed)
SENSOR_PINS = {"left": 14, "center": 15, "right": 23}

# GPIO Setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
for pin in SENSOR_PINS.values():
    GPIO.setup(pin, GPIO.IN)

def read_sensors():
    """Read all line tracking sensors and return as a dict."""
    return {name: int(GPIO.input(pin)) for name, pin in SENSOR_PINS.items()}

async def handler(websocket):
    print(f"[CONNECTED] Client: {websocket.remote_address}")
    try:
        while True:
            state = read_sensors()
            payload = {
                "status": "success",
                "lineTracking": state,
                "timestamp": time.time()
            }
            await websocket.send(json.dumps(payload))
            await asyncio.sleep(0.1)  # Stream at 10Hz
    except websockets.exceptions.ConnectionClosed:
        print(f"[DISCONNECTED] {websocket.remote_address}")

if __name__ == "__main__":
    print("Starting Line Tracker WebSocket Server on ws://0.0.0.0:8090/line-tracker")
    async def main():
        async with websockets.serve(handler, "0.0.0.0", 8090):
            await asyncio.Future()  # run forever
    try:
        asyncio.run(main())
    finally:
        GPIO.cleanup()
