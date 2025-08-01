"""
# File: /Omega-Code/servers/robot-controller-backend/movement/movement_ws_server.py
# Summary:
WebSocket server for robot movement, speed, buzzer, and servo control.
Routes commands from the frontend UI to the correct hardware controller modules.
"""

import sys
import os
import asyncio
import websockets
import json

# Add parent directory to sys.path so we can import from controllers/
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from minimal_motor_control import Motor
from controllers.buzzer import setup_buzzer, buzz_on, buzz_off
from controllers.servo_control import Servo

motor = Motor()
setup_buzzer()  # Only once at startup
servo = Servo()  # Only once at startup

current_speed = 2000  # Default speed
current_horizontal_angle = 90  # Start centered
current_vertical_angle = 90

def clamp_angle(angle):
    return max(0, min(180, angle))

async def handler(websocket):
    global current_speed, current_horizontal_angle, current_vertical_angle
    print(f"[CONNECTED] Client: {websocket.remote_address}")
    try:
        async for message in websocket:
            print(f"[RECEIVED] {message}")
            try:
                data = json.loads(message)
                cmd = data.get("command")
                speed = int(data.get("speed", current_speed))

                # --- Movement commands ---
                if cmd in ["forward", "move-up"]:
                    motor.forward(speed)
                    response = {"status": "ok", "action": "forward", "speed": speed}

                elif cmd in ["backward", "move-down"]:
                    motor.backward(speed)
                    response = {"status": "ok", "action": "backward", "speed": speed}

                elif cmd in ["stop", "move-stop"]:
                    motor.stop()
                    response = {"status": "ok", "action": "stop"}

                # --- Speed commands ---
                elif cmd == "increase-speed":
                    current_speed = min(current_speed + 200, 4095)
                    response = {"status": "ok", "action": "increase-speed", "speed": current_speed}

                elif cmd == "decrease-speed":
                    current_speed = max(current_speed - 200, 0)
                    response = {"status": "ok", "action": "decrease-speed", "speed": current_speed}

                # --- Buzzer commands ---
                elif cmd == "buzz":
                    buzz_on()
                    response = {"status": "ok", "action": "buzz"}

                elif cmd == "buzz-stop":
                    buzz_off()
                    response = {"status": "ok", "action": "buzz-stop"}

                # --- Servo control commands ---
                elif cmd == "servo-horizontal":
                    delta = int(data.get("angle", 0))
                    current_horizontal_angle = clamp_angle(current_horizontal_angle + delta)
                    print(f"[SERVO] Horizontal new angle: {current_horizontal_angle}")
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    response = {"status": "ok", "action": "servo-horizontal", "angle": current_horizontal_angle}

                elif cmd == "servo-vertical":
                    delta = int(data.get("angle", 0))
                    current_vertical_angle = clamp_angle(current_vertical_angle + delta)
                    print(f"[SERVO] Vertical new angle: {current_vertical_angle}")
                    servo.setServoPwm("vertical", current_vertical_angle)
                    response = {"status": "ok", "action": "servo-vertical", "angle": current_vertical_angle}

                # --- Unknown command ---
                else:
                    response = {"status": "error", "error": f"Unknown command: {cmd}"}

            except Exception as e:
                print(f"[ERROR] Command processing failed: {e}")
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
