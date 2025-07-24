"""
# File: /Omega-Code/servers/robot-controller-backend/movement/movement_ws_server.py
# Summary:
WebSocket server for robot movement, speed, and buzzer control.
Routes commands from the frontend UI to the correct hardware controller modules.
"""

import asyncio
import websockets
import json
from minimal_motor_control import Motor
from controllers.buzzer import setup_buzzer, buzz_on, buzz_off

motor = Motor()
setup_buzzer()  # Only once at startup!

current_speed = 2000  # Default speed (adjust as needed)

async def handler(websocket):
    global current_speed
    print(f"[CONNECTED] Client: {websocket.remote_address}")
    try:
        async for message in websocket:
            print(f"[RECEIVED] {message}")
            try:
                data = json.loads(message)
                cmd = data.get("command")

                # Allow speed to be overridden by payload if provided
                speed = int(data.get("speed", current_speed))

                if cmd in ["forward", "move-up"]:
                    motor.forward(speed)
                    response = {"status": "ok", "action": "forward", "speed": speed}
                elif cmd in ["backward", "move-down"]:
                    motor.backward(speed)
                    response = {"status": "ok", "action": "backward", "speed": speed}
                elif cmd in ["stop", "move-stop"]:
                    motor.stop()
                    response = {"status": "ok", "action": "stop"}
                elif cmd in ["increase-speed"]:
                    current_speed = min(current_speed + 200, 4095)
                    response = {"status": "ok", "action": "increase-speed", "speed": current_speed}
                elif cmd in ["decrease-speed"]:
                    current_speed = max(current_speed - 200, 0)
                    response = {"status": "ok", "action": "decrease-speed", "speed": current_speed}
                elif cmd == "buzz":
                    buzz_on()
                    response = {"status": "ok", "action": "buzz"}
                elif cmd == "buzz-stop":
                    buzz_off()
                    response = {"status": "ok", "action": "buzz-stop"}
                else:
                    response = {"status": "error", "error": f"Unknown command: {cmd}"}
            except Exception as e:
                response = {"status": "error", "error": str(e)}

            try:
                await websocket.send(json.dumps(response))
            except Exception as e:
                print(f"[SEND ERROR] {e}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"[DISCONNECTED] {websocket.remote_address} - {e}")

if __name__ == "__main__":
    print("Starting Movement WebSocket Server on ws://0.0.0.0:8081")

    async def main():
        async with websockets.serve(handler, "0.0.0.0", 8081):
            await asyncio.Future()  # run forever

    asyncio.run(main())
