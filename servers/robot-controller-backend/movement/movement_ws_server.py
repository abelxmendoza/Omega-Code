# File: /Omega-Code/servers/robot-controller-backend/movement/movement_ws_server.py
# Summary:
#   WebSocket server for robot movement, speed, buzzer, and servo control.
#   - JSON welcome on connect; replies { "type":"pong" } to { "type":"ping" }
#   - Commands aligned with ui/src/control_definitions.ts (+ tolerant synonyms)
#   - Clamped speed/angles, structured ACKs, safe stop on disconnect
#   - Optional timed moves (durationMs) with guard so newer moves aren't stopped by older timers
#   - Single-writer motor lock to avoid concurrent GPIO writes
#   - Optional origin allowlist + optional simulation (no hardware)
#
#   Env:
#     PORT_MOVEMENT=8081
#     MOVEMENT_PATH="/"                # set to "/movement" if you want a path (UI then must use it)
#     ORIGIN_ALLOW="http://localhost:3000,https://your-ui"
#     ROBOT_SIM=0|1                    # 1 = no-op motor/buzzer/servo for dev
#
#   Start:
#     python3 movement_ws_server.py
#
#   Notes:
#     - If you serve the UI over HTTPS, make sure the UI keeps ws:// (NEXT_PUBLIC_WS_FORCE_INSECURE=1)
#       or terminate TLS in a reverse proxy.
#     - Compatible with websockets >= 12 (no path parameter) and older versions (with path parameter).

import os
import sys
import json
import time
import asyncio
from typing import Optional, Set, Tuple

import websockets
# Use legacy namespace for stable type hints across versions (avoids deprecation warning).
from websockets.legacy.server import WebSocketServerProtocol

# ---------- util helpers ----------

def _env(key: str, default: Optional[str] = None) -> Optional[str]:
    return os.getenv(key, default)

def _env_int(key: str, default: int) -> int:
    try:
        return int(os.getenv(key, str(default)))
    except Exception:
        return default

def _now_ms() -> int:
    return int(time.time() * 1000)

def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))

def log(*a):    print("[MOVE]", *a, file=sys.stdout, flush=True)
def warn(*a):   print("[MOVE][WARN]", *a, file=sys.stdout, flush=True)
def elog(*a):   print("[MOVE][ERROR]", *a, file=sys.stderr,  flush=True)

def ok(action: str, **extra) -> dict:
    return {"type": "ack", "status": "ok", "action": action, **extra, "ts": _now_ms()}

def err(msg: str, **extra) -> dict:
    return {"type": "ack", "status": "error", "error": msg, **extra, "ts": _now_ms()}

async def send_json(ws: WebSocketServerProtocol, payload: dict) -> None:
    try:
        await ws.send(json.dumps(payload))
    except Exception as e:
        # Don't raise; connection might be going away.
        elog("send failed:", repr(e))

# ---------- config ----------

PORT  = _env_int("PORT_MOVEMENT", 8081)
PATH  = (_env("MOVEMENT_PATH", "/") or "/").rstrip("/") or "/"

_ALLOWED_ORIGINS: Set[str] = set(
    o.strip() for o in (_env("ORIGIN_ALLOW", "") or "").split(",") if o.strip()
)
SIM_MODE = _env("ROBOT_SIM", "0") == "1"

# ---------- hardware (with simulation fallback) ----------

class _NoopMotor:
    def forward(self, *a, **k): log("NOOP motor.forward", a, k)
    def backward(self, *a, **k): log("NOOP motor.backward", a, k)
    def left(self, *a, **k): log("NOOP motor.left", a, k)
    def right(self, *a, **k): log("NOOP motor.right", a, k)
    def turn_left(self, *a, **k): log("NOOP motor.turn_left", a, k)
    def turn_right(self, *a, **k): log("NOOP motor.turn_right", a, k)
    def stop(self): log("NOOP motor.stop")

class _NoopServo:
    def setServoPwm(self, axis: str, angle: int): log(f"NOOP servo[{axis}] -> {angle}")

def _noop_buzz_on():  log("NOOP buzz_on")
def _noop_buzz_off(): log("NOOP buzz_off")

Motor = None
Servo = None
buzz_on = _noop_buzz_on
buzz_off = _noop_buzz_off

if SIM_MODE:
    log("SIM_MODE=1 → using NOOP devices")
    motor = _NoopMotor()
    servo = _NoopServo()
else:
    try:
        # Prefer controllers/… paths
        ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        if ROOT not in sys.path:
            sys.path.append(ROOT)

        try:
            from controllers.minimal_motor_control import Motor as _Motor
        except Exception:
            from minimal_motor_control import Motor as _Motor  # fallback

        from controllers.servo_control import Servo as _Servo
        from controllers.buzzer import setup_buzzer, buzz_on as _buzz_on, buzz_off as _buzz_off

        motor = _Motor()
        servo = _Servo()
        buzz_on = _buzz_on
        buzz_off = _buzz_off
        # init buzzer safely
        try:
            setup_buzzer()
        except Exception as e:
            warn("buzzer setup failed (continuing):", repr(e))
    except Exception as e:
        elog("hardware import failed → falling back to NOOP:", repr(e))
        motor = _NoopMotor()
        servo = _NoopServo()
        buzz_on = _noop_buzz_on
        buzz_off = _noop_buzz_off

# ---------- server state ----------

SPEED_MIN, SPEED_MAX = 0, 4095
SERVO_MIN, SERVO_MAX = 0, 180
DEFAULT_SPEED_STEP   = 200
DEFAULT_SERVO_STEP   = 5

current_speed             = 2000
current_horizontal_angle  = 90
current_vertical_angle    = 90

# Serialize motor operations
motor_lock = asyncio.Lock()

# When a timed move is issued, we record an op id; only that op may auto-stop itself.
_last_move_op_id = 0

# ---------- helpers ----------

def origin_ok(ws: WebSocketServerProtocol) -> bool:
    if not _ALLOWED_ORIGINS:
        return True
    origin = ws.request_headers.get("Origin")
    return bool(origin) and origin in _ALLOWED_ORIGINS

def path_ok(request_path: str) -> bool:
    if PATH == "/":
        return True  # root
    # tolerate trailing slash and ignore queries
    cleaned = (request_path or "/").split("?", 1)[0].rstrip("/") or "/"
    return cleaned == PATH

def parse_json(msg: str) -> Tuple[Optional[str], dict]:
    """
    Accept JSON commands (preferred). Returns (command, payload).
    Ignores non-JSON messages but keeps the server alive.
    """
    try:
        data = json.loads(msg)
        if isinstance(data, dict):
            cmd = data.get("command")
            return (cmd if isinstance(cmd, str) else None, data)
    except Exception:
        pass
    return (None, {})

async def do_stop():
    async with motor_lock:
        try:
            motor.stop()
            buzz_off()
        except Exception as e:
            elog("stop failed:", repr(e))

async def do_move(fn_name: str, speed: int):
    async with motor_lock:
        fn = getattr(motor, fn_name, None)
        if not callable(fn):
            # try turn_left/turn_right fallbacks
            if fn_name == "left":
                fn = getattr(motor, "turn_left", None)
            elif fn_name == "right":
                fn = getattr(motor, "turn_right", None)
        if not callable(fn):
            raise RuntimeError(f"motor missing method: {fn_name}")
        fn(speed)

def norm_cmd_name(raw: str) -> str:
    """
    Map tolerant synonyms from the UI into canonical actions.
      forward:  forward | move-up | move-forward | 'w'
      backward: backward | move-down | move-back  | 's'
      left:     left | move-left | turn-left      | 'a'
      right:    right| move-right| turn-right     | 'd'
      stop:     stop | move-stop | halt           | ' '
    """
    s = (raw or "").strip().lower()
    if s in {"forward", "move-up", "move-forward", "w"}: return "forward"
    if s in {"backward", "move-down", "move-back", "s"}: return "backward"
    if s in {"left", "move-left", "turn-left", "a"}:     return "left"
    if s in {"right", "move-right", "turn-right", "d"}:  return "right"
    if s in {"stop", "move-stop", "halt", ""}:           return "stop"
    return raw

def _extract_request_path(ws: WebSocketServerProtocol, request_path: Optional[str]) -> str:
    """
    Support websockets >=12 (no path parameter) and older (with path).
    Try, in order:
      - explicit request_path argument (old signature)
      - ws.path (newer versions)
      - ws.request.path (some internals)
      - fallback "/"
    """
    if isinstance(request_path, str) and request_path:
        return request_path
    rp = getattr(ws, "path", None)
    if isinstance(rp, str) and rp:
        return rp
    try:
        rp = ws.request.path  # type: ignore[attr-defined]
        if isinstance(rp, str) and rp:
            return rp
    except Exception:
        pass
    return "/"

# ---------- websocket handler ----------

async def handler(ws: WebSocketServerProtocol, request_path: Optional[str] = None):
    global current_speed, current_horizontal_angle, current_vertical_angle, _last_move_op_id

    request_path = _extract_request_path(ws, request_path)
    peer = getattr(ws, "remote_address", None)
    log("CONNECTED", peer, f"path={request_path!r}")

    if not path_ok(request_path):
        warn("reject path", request_path, "expected", PATH)
        await send_json(ws, err("invalid-path", expected=PATH))
        await ws.close(code=1008, reason="invalid path")
        return

    if not origin_ok(ws):
        warn("reject origin", ws.request_headers.get("Origin"))
        await send_json(ws, err("origin-not-allowed"))
        await ws.close(code=4403, reason="forbidden origin")
        return

    # Welcome
    await send_json(ws, {
        "type": "welcome",
        "status": "connected",
        "service": "movement",
        "ts": _now_ms(),
        "path": request_path or "/",
        "sim": SIM_MODE,
    })

    try:
        async for message in ws:
            # Heartbeat first (allow non-JSON ping envelope from UI)
            if isinstance(message, str) and message.startswith('{"type":"ping"'):
                try:
                    obj = json.loads(message)
                    await send_json(ws, {"type": "pong", "ts": obj.get("ts", _now_ms())})
                except Exception:
                    await send_json(ws, {"type": "pong", "ts": _now_ms()})
                continue

            cmd, data = parse_json(message)
            if not cmd:
                warn("non-JSON or missing command; ignored")
                continue

            cmd = norm_cmd_name(cmd)

            # quick reads
            speed = clamp(int(data.get("speed", current_speed)), SPEED_MIN, SPEED_MAX)
            duration_ms = int(data.get("durationMs", 0)) if str(data.get("durationMs", "")).isdigit() else 0

            try:
                # -------- MOVEMENT --------
                if cmd in {"forward", "backward", "left", "right"}:
                    current_speed = speed
                    await do_move(cmd, current_speed)
                    await send_json(ws, ok(cmd, speed=current_speed))

                    # Optional timed move
                    if duration_ms > 0:
                        _last_move_op_id += 1
                        my_id = _last_move_op_id
                        async def _auto_stop(delay: float, op_id: int):
                            try:
                                await asyncio.sleep(delay)
                                if op_id == _last_move_op_id:
                                    await do_stop()
                                    await send_json(ws, ok("auto-stop"))
                            except Exception as e:
                                elog("auto-stop failed:", repr(e))
                        asyncio.create_task(_auto_stop(duration_ms/1000.0, my_id))

                elif cmd == "stop":
                    await do_stop()
                    await send_json(ws, ok("stop"))

                # -------- SPEED --------
                elif cmd == "increase-speed":
                    current_speed = clamp(current_speed + DEFAULT_SPEED_STEP, SPEED_MIN, SPEED_MAX)
                    await send_json(ws, ok("increase-speed", speed=current_speed))

                elif cmd == "decrease-speed":
                    current_speed = clamp(current_speed - DEFAULT_SPEED_STEP, SPEED_MIN, SPEED_MAX)
                    await send_json(ws, ok("decrease-speed", speed=current_speed))

                elif cmd == "set-speed":
                    current_speed = clamp(int(data.get("value", current_speed)), SPEED_MIN, SPEED_MAX)
                    await send_json(ws, ok("set-speed", speed=current_speed))

                # -------- BUZZER --------
                elif cmd == "buzz":
                    try:
                        buzz_on()
                    finally:
                        await send_json(ws, ok("buzz"))

                elif cmd == "buzz-stop":
                    try:
                        buzz_off()
                    finally:
                        await send_json(ws, ok("buzz-stop"))

                # -------- SERVOS --------
                elif cmd == "servo-horizontal":
                    delta = int(data.get("angle", 0))
                    current_horizontal_angle = clamp(current_horizontal_angle + delta, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    await send_json(ws, ok("servo-horizontal", angle=current_horizontal_angle))

                elif cmd == "servo-vertical":
                    delta = int(data.get("angle", 0))
                    current_vertical_angle = clamp(current_vertical_angle + delta, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(ws, ok("servo-vertical", angle=current_vertical_angle))

                elif cmd == "set-servo-position":
                    h = data.get("horizontal", None)
                    v = data.get("vertical", None)
                    if h is None and v is None:
                        await send_json(ws, err("missing-servo-fields"))
                    else:
                        if h is not None:
                            current_horizontal_angle = clamp(int(h), SERVO_MIN, SERVO_MAX)
                            servo.setServoPwm("horizontal", current_horizontal_angle)
                        if v is not None:
                            current_vertical_angle = clamp(int(v), SERVO_MIN, SERVO_MAX)
                            servo.setServoPwm("vertical", current_vertical_angle)
                        await send_json(ws, ok("set-servo-position",
                                               horizontal=current_horizontal_angle,
                                               vertical=current_vertical_angle))

                elif cmd == "reset-servo":
                    current_horizontal_angle = 90
                    current_vertical_angle = 90
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(ws, ok("reset-servo",
                                           horizontal=current_horizontal_angle,
                                           vertical=current_vertical_angle))

                # camera nudge aliases
                elif cmd == "camera-servo-left":
                    current_horizontal_angle = clamp(current_horizontal_angle - DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    await send_json(ws, ok("camera-servo-left", angle=current_horizontal_angle))

                elif cmd == "camera-servo-right":
                    current_horizontal_angle = clamp(current_horizontal_angle + DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    await send_json(ws, ok("camera-servo-right", angle=current_horizontal_angle))

                elif cmd == "camera-servo-up":
                    current_vertical_angle = clamp(current_vertical_angle + DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(ws, ok("camera-servo-up", angle=current_vertical_angle))

                elif cmd == "camera-servo-down":
                    current_vertical_angle = clamp(current_vertical_angle - DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(ws, ok("camera-servo-down", angle=current_vertical_angle))

                # -------- STATUS --------
                elif cmd == "status":
                    await send_json(ws, {
                        "type": "status",
                        "speed": current_speed,
                        "servo": {
                            "horizontal": current_horizontal_angle,
                            "vertical": current_vertical_angle,
                        },
                        "ts": _now_ms(),
                        "sim": SIM_MODE,
                    })

                else:
                    await send_json(ws, err(f"unknown-command:{cmd}"))

            except Exception as ex:
                elog("command failed:", repr(ex))
                await send_json(ws, err("exception", detail=str(ex)))

    except websockets.exceptions.ConnectionClosed as e:
        log("DISCONNECTED", peer, e.code, e.reason or "")
    except Exception as e:
        elog("handler error:", repr(e))
    finally:
        # Safety: stop motors & buzzer when client drops
        await do_stop()

# ---------- serve ----------

async def main():
    log(f"listening on ws://0.0.0.0:{PORT}{'' if PATH=='/' else PATH}")
    # We disable TCP-level ping so JSON ping/pong from the UI is the single heartbeat.
    async with websockets.serve(
        handler,
        "0.0.0.0",
        PORT,
        ping_interval=None,
        max_size=64 * 1024,
        max_queue=64,
        # process_request could enforce PATH for HTTP GETs if you expose health here later
    ):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
