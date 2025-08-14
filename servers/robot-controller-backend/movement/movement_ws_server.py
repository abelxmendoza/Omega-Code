# File: /Omega-Code/servers/robot-controller-backend/movement/movement_ws_server.py
# Summary:
# WebSocket server for robot movement, speed, buzzer, and servo control.
# - JSON welcome on connect; replies {type:"pong"} to {type:"ping"}
# - Commands aligned with ui/src/control_definitions.ts
# - Clamped speed/angles, structured acks, safe stop on disconnect
# - Optional env: PORT_MOVEMENT, ORIGIN_ALLOW (comma-separated)

import sys
import os
import asyncio
import json
import time
from typing import Optional, Set

import websockets

# --- Import paths so controllers/utils resolve ---
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(ROOT)

# ---- Optional env helpers (works even if python-dotenv isn't installed) ----
def _env(key: str, default: Optional[str] = None) -> Optional[str]:
    return os.getenv(key, default)

def _env_int(key: str, default: int) -> int:
    try:
        return int(os.getenv(key, str(default)))
    except Exception:
        return default

# ---- Hardware modules (adjust imports if your files live elsewhere) ----
# minimal_motor_control can be under controllers/ or common/gpio in some repos.
try:
    from controllers.minimal_motor_control import Motor
except ImportError:
    try:
        from minimal_motor_control import Motor  # as in your original file
    except Exception as e:
        raise

from controllers.buzzer import setup_buzzer, buzz_on, buzz_off
from controllers.servo_control import Servo

# ---- Constants / state ----
SPEED_MIN, SPEED_MAX = 0, 4095
SERVO_MIN, SERVO_MAX = 0, 180
DEFAULT_SPEED_STEP = 200
DEFAULT_SERVO_STEP = 5

PORT = _env_int("PORT_MOVEMENT", 8081)

# Allowlist Origin check (optional). Example:
# export ORIGIN_ALLOW="http://localhost:3000,https://your-ui.example.com"
_ALLOWED_ORIGINS: Set[str] = set(
    [o.strip() for o in (_env("ORIGIN_ALLOW", "") or "").split(",") if o.strip()]
)
def _origin_ok(ws) -> bool:
    if not _ALLOWED_ORIGINS:
        return True  # disabled
    origin = ws.request_headers.get("Origin")
    return origin in _ALLOWED_ORIGINS

# ---- Devices (singletons) ----
motor = Motor()
setup_buzzer()
servo = Servo()

current_speed = 2000
current_horizontal_angle = 90
current_vertical_angle = 90

def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))

def ok(action: str, **extra):
    return {"type": "ack", "status": "ok", "action": action, **extra, "ts": int(time.time() * 1000)}

def err(message: str, **extra):
    return {"type": "ack", "status": "error", "error": message, **extra, "ts": int(time.time() * 1000)}

async def send_json(ws, payload: dict):
    try:
        await ws.send(json.dumps(payload))
    except Exception as e:
        print(f"[SEND ERROR] {e}")

async def handler(websocket: websockets.WebSocketServerProtocol):
    global current_speed, current_horizontal_angle, current_vertical_angle

    addr = getattr(websocket, "remote_address", None)
    print(f"[CONNECTED] {addr}")

    if not _origin_ok(websocket):
        print("[SECURITY] Rejecting connection due to Origin")
        await send_json(websocket, err("origin-not-allowed"))
        await websocket.close(code=4403, reason="Forbidden origin")
        return

    # Welcome envelope
    await send_json(websocket, {
        "type": "welcome",
        "status": "connected",
        "service": "movement",
        "ts": int(time.time() * 1000),
    })

    try:
        async for message in websocket:
            # Require JSON
            try:
                data = json.loads(message)
            except Exception:
                print(f"[WARN] Non-JSON ignored: {message!r}")
                continue

            # Heartbeat
            if data.get("type") == "ping":
                await send_json(websocket, {"type": "pong", "ts": data.get("ts", int(time.time() * 1000))})
                continue

            # ---- Command handling (aligned with control_definitions.ts) ----
            try:
                cmd = data.get("command")
                if not cmd:
                    await send_json(websocket, err("missing-command"))
                    continue

                # Speed parameter (clamped)
                speed = clamp(int(data.get("speed", current_speed)), SPEED_MIN, SPEED_MAX)

                # 1) Movement
                if cmd in ("forward", "move-up"):
                    current_speed = speed
                    motor.forward(current_speed)
                    await send_json(websocket, ok("forward", speed=current_speed))

                elif cmd in ("backward", "move-down"):
                    current_speed = speed
                    motor.backward(current_speed)
                    await send_json(websocket, ok("backward", speed=current_speed))

                elif cmd in ("move-left",):
                    # Some Motor classes expose left()/turn_left()
                    if hasattr(motor, "left"):
                        current_speed = speed
                        getattr(motor, "left")(current_speed)
                        await send_json(websocket, ok("left", speed=current_speed))
                    elif hasattr(motor, "turn_left"):
                        current_speed = speed
                        getattr(motor, "turn_left")(current_speed)
                        await send_json(websocket, ok("turn-left", speed=current_speed))
                    else:
                        await send_json(websocket, err("unsupported:move-left"))

                elif cmd in ("move-right",):
                    if hasattr(motor, "right"):
                        current_speed = speed
                        getattr(motor, "right")(current_speed)
                        await send_json(websocket, ok("right", speed=current_speed))
                    elif hasattr(motor, "turn_right"):
                        current_speed = speed
                        getattr(motor, "turn_right")(current_speed)
                        await send_json(websocket, ok("turn-right", speed=current_speed))
                    else:
                        await send_json(websocket, err("unsupported:move-right"))

                elif cmd in ("stop", "move-stop"):
                    motor.stop()
                    await send_json(websocket, ok("stop"))

                # 2) Speed
                elif cmd == "increase-speed":
                    current_speed = clamp(current_speed + DEFAULT_SPEED_STEP, SPEED_MIN, SPEED_MAX)
                    await send_json(websocket, ok("increase-speed", speed=current_speed))

                elif cmd == "decrease-speed":
                    current_speed = clamp(current_speed - DEFAULT_SPEED_STEP, SPEED_MIN, SPEED_MAX)
                    await send_json(websocket, ok("decrease-speed", speed=current_speed))

                # 3) Buzzer
                elif cmd == "buzz":
                    buzz_on()
                    await send_json(websocket, ok("buzz"))

                elif cmd == "buzz-stop":
                    buzz_off()
                    await send_json(websocket, ok("buzz-stop"))

                # 4) Servos (primary API)
                elif cmd == "servo-horizontal":
                    delta = int(data.get("angle", 0))
                    current_horizontal_angle = clamp(current_horizontal_angle + delta, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    await send_json(websocket, ok("servo-horizontal", angle=current_horizontal_angle))

                elif cmd == "servo-vertical":
                    delta = int(data.get("angle", 0))
                    current_vertical_angle = clamp(current_vertical_angle + delta, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(websocket, ok("servo-vertical", angle=current_vertical_angle))

                # 4b) Servo convenience: set explicit position(s)
                elif cmd == "set-servo-position":
                    # Accept horizontal/vertical fields; clamp both
                    h = data.get("horizontal")
                    v = data.get("vertical")
                    if h is not None:
                        current_horizontal_angle = clamp(int(h), SERVO_MIN, SERVO_MAX)
                        servo.setServoPwm("horizontal", current_horizontal_angle)
                    if v is not None:
                        current_vertical_angle = clamp(int(v), SERVO_MIN, SERVO_MAX)
                        servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(websocket, ok("set-servo-position",
                                                  horizontal=current_horizontal_angle,
                                                  vertical=current_vertical_angle))

                elif cmd == "reset-servo":
                    current_horizontal_angle = 90
                    current_vertical_angle = 90
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(websocket, ok("reset-servo",
                                                  horizontal=current_horizontal_angle,
                                                  vertical=current_vertical_angle))

                # 4c) Camera nudge aliases (map to servo deltas)
                elif cmd == "camera-servo-left":
                    current_horizontal_angle = clamp(current_horizontal_angle - DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    await send_json(websocket, ok("camera-servo-left", angle=current_horizontal_angle))

                elif cmd == "camera-servo-right":
                    current_horizontal_angle = clamp(current_horizontal_angle + DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    await send_json(websocket, ok("camera-servo-right", angle=current_horizontal_angle))

                elif cmd == "camera-servo-up":
                    current_vertical_angle = clamp(current_vertical_angle + DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(websocket, ok("camera-servo-up", angle=current_vertical_angle))

                elif cmd == "camera-servo-down":
                    current_vertical_angle = clamp(current_vertical_angle - DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(websocket, ok("camera-servo-down", angle=current_vertical_angle))

                # Unknown
                else:
                    await send_json(websocket, err(f"unknown-command:{cmd}"))

            except Exception as e:
                print(f"[ERROR] Command failure: {e}")
                await send_json(websocket, err("exception", detail=str(e)))

    except websockets.exceptions.ConnectionClosed as e:
        print(f"[DISCONNECTED] {addr} - {e}")
    finally:
        # Safety: stop motors & buzzer when client drops
        try:
            motor.stop()
            buzz_off()
        except Exception:
            pass

if __name__ == "__main__":
    print(f"Starting Movement WebSocket Server on ws://0.0.0.0:{PORT}")

    async def main():
        # Disable TCP ping so JSON ping/pong is the single heartbeat
        async with websockets.serve(
            handler,
            "0.0.0.0",
            PORT,
            ping_interval=None,
            max_size=64 * 1024,
            max_queue=32,
        ):
            await asyncio.Future()  # run forever

    asyncio.run(main())
