# movement_ws_server.py

import asyncio
import websockets
import json
from minimal_motor_control import Motor

motor = Motor()

async def handler(websocket, path):
    print(f"[CONNECTED] Client: {websocket.remote_address}")
    try:
        async for message in websocket:
            print(f"[RECEIVED] {message}")
            try:
                data = json.loads(message)
                cmd = data.get("command")
                speed = int(data.get("speed", 2000))  # Optional: set default speed

                if cmd == "forward":
                    motor.forward(speed)
                    response = {"status": "ok", "action": "forward", "speed": speed}
                elif cmd == "backward":
                    motor.backward(speed)
                    response = {"status": "ok", "action": "backward", "speed": speed}
                elif cmd == "stop":
                    motor.stop()
                    response = {"status": "ok", "action": "stop"}
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
    start_server = websockets.serve(handler, "0.0.0.0", 8081)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
