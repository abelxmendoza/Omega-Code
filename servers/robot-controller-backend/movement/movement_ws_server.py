# File: /Omega-Code/servers/robot-controller-backend/movement/movement_ws_server.py
# Summary:
#   WebSocket server for robot movement, speed, buzzer, and servo control.
#   - JSON welcome on connect; replies { "type":"pong" } to { "type":"ping" }
#   - Commands aligned with ui/src/control_definitions.ts (+ tolerant synonyms)
#   - Clamped speed/angles, structured ACKs
#   - Safe stop on disconnect (only when last client leaves)
#   - Optional timed moves (durationMs) with guard so newer moves aren't stopped by older timers
#   - Single-writer motor lock + buzzer lock to avoid concurrent device writes
#   - Optional origin allowlist + optional simulation (no hardware)
#   - Straight-drive assist trim helper with remote tuning commands
#
#   Env:
#     PORT_MOVEMENT=8081
#     MOVEMENT_PATH="/"                # set to "/movement" if you want a path (UI then must use it)
#     ORIGIN_ALLOW="http://localhost:3000,https://your-ui"
#     ORIGIN_ALLOW_NO_HEADER=0|1       # allow CLI tools that don't send Origin
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
import inspect
import traceback
import logging
from typing import Optional, Set, Tuple

# Add parent directory to path for autonomy module
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.append(ROOT)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('movement_ws_server.log')
    ]
)
logger = logging.getLogger(__name__)

# Error handling utilities
class MovementError(Exception):
    """Base exception for movement-related errors"""
    pass

class MotorError(MovementError):
    """Motor control specific errors"""
    pass

class ServoError(MovementError):
    """Servo control specific errors"""
    pass

class WebSocketError(MovementError):
    """WebSocket communication errors"""
    pass

def log_error(error: Exception, context: str = "", websocket=None):
    """Log error with context and optional WebSocket info"""
    error_info = {
        "error_type": type(error).__name__,
        "error_message": str(error),
        "context": context,
        "timestamp": time.time(),
        "traceback": traceback.format_exc()
    }
    
    if websocket:
        error_info["websocket_info"] = {
            "remote_address": websocket.remote_address if hasattr(websocket, 'remote_address') else None,
            "state": websocket.state.name if hasattr(websocket, 'state') else None
        }
    
    logger.error(f"Movement WebSocket Error: {json.dumps(error_info, indent=2)}")
    return error_info

def safe_json_send(websocket, data: dict, context: str = ""):
    """Safely send JSON data with error handling"""
    try:
        if websocket and websocket.state.name == "OPEN":
            asyncio.create_task(websocket.send(json.dumps(data)))
            return True
        else:
            logger.warning(f"Cannot send data - WebSocket not open. Context: {context}")
            return False
    except Exception as e:
        log_error(e, f"Failed to send JSON data: {context}")
        return False

from autonomy import AutonomyError, build_default_controller

try:
    from .straight_drive_assist import StraightDriveAssist
except Exception:  # pragma: no cover - fallback for script execution
    from straight_drive_assist import StraightDriveAssist

try:
    from .motor_telemetry import MotorTelemetryController
except Exception:  # pragma: no cover - fallback for script execution
    from motor_telemetry import MotorTelemetryController

# Import optimization utilities
try:
    from ..utils.optimization.websocket_optimizer import WebSocketOptimizer, ConnectionPool
    from ..utils.optimization.cache_manager import cached, cache_manager
    from ..utils.optimization.async_processor import task_processor, TaskPriority
    from ..utils.optimization.performance_monitor import performance_monitor, app_profiler
    OPTIMIZATION_AVAILABLE = True
except ImportError:
    logger.warning("Optimization utilities not available")
    OPTIMIZATION_AVAILABLE = False

# Optional .env loader (backend root)
try:
    from dotenv import load_dotenv
    _root_env = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".env"))
    if os.path.exists(_root_env):
        load_dotenv(_root_env)
except Exception:
    pass

import websockets
# Use legacy namespace for stable type hints across versions (avoids deprecation warnings).
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

def _as_bool(value, default: Optional[bool] = None) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        s = value.strip().lower()
        if s in {"1", "true", "yes", "on", "enable", "enabled"}:
            return True
        if s in {"0", "false", "no", "off", "disable", "disabled"}:
            return False
    return default

def _as_int(value, default: Optional[int] = None) -> Optional[int]:
    try:
        return int(value)  # type: ignore[arg-type]
    except Exception:
        return default

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
ALLOW_NO_ORIGIN = _env("ORIGIN_ALLOW_NO_HEADER", "0") == "1"
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
        # Prefer controllers/… paths (ROOT already added to sys.path above)

        try:
            from controllers.minimal_motor_control import Motor as _Motor
        except Exception:
            from minimal_motor_control import Motor as _Motor  # fallback

        from controllers.servo_control import Servo as _Servo
        from controllers.buzzer import setup_buzzer, buzz_on as _buzz_on, buzz_off as _buzz_off

        # Use MotorTelemetryController instead of basic Motor
        motor = MotorTelemetryController()
        servo = _Servo()
        buzz_on = _buzz_on
        buzz_off = _buzz_off
        # init buzzer safely
        try:
            setup_buzzer()
        except Exception as e:
            warn("buzzer setup failed (continuing):", repr(e))
        log(f"HW init ok: motor={type(motor).__name__}, servo={type(servo).__name__}")
    except Exception as e:
        elog("hardware import failed → falling back to NOOP:", repr(e))
        motor = _NoopMotor()
        servo = _NoopServo()
        buzz_on = _noop_buzz_on
        buzz_off = _noop_buzz_off

# Autonomy controller (modular command routing)
AUTONOMY = build_default_controller(context={
    "motor": motor,
    "servo": servo,
    "buzz_on": buzz_on,
    "buzz_off": buzz_off,
})

STRAIGHT_ASSIST = StraightDriveAssist(motor)
# Initialize motor telemetry with caching
if OPTIMIZATION_AVAILABLE:
    @cached(ttl=1, key_prefix="motor_telemetry:")  # Cache for 1 second
    def get_cached_motor_telemetry():
        return MOTOR_TELEMETRY.get_telemetry()
else:
    def get_cached_motor_telemetry():
        return MOTOR_TELEMETRY.get_telemetry()

# ---------- server state ----------

SPEED_MIN, SPEED_MAX = 0, 4095
SERVO_MIN, SERVO_MAX = 0, 180


def _servo_state_payload(horizontal: int, vertical: int) -> dict:
    return {
        "servo": {
            "horizontal": horizontal,
            "vertical": vertical,
            "min": SERVO_MIN,
            "max": SERVO_MAX,
        }
    }
DEFAULT_SPEED_STEP   = 200
DEFAULT_SERVO_STEP   = 5

current_speed             = 1200
current_horizontal_angle  = 90   # Updated to match current position
current_vertical_angle    = 90   # Updated to match current position

# Serialize motor operations
motor_lock = asyncio.Lock()

# Buzzer control
buzzer_lock = asyncio.Lock()
_buzz_task: Optional[asyncio.Task] = None
_last_buzz_op_id = 0
buzzer_on_state = False

# Timed move guard
_last_move_op_id = 0

# Initialize optimization systems
if OPTIMIZATION_AVAILABLE:
    websocket_optimizer = WebSocketOptimizer(batch_size=5, batch_timeout=0.02)
    connection_pool = ConnectionPool(max_connections=50)
    
    # Start async task processor
    asyncio.create_task(task_processor.start())
    
    # Start performance monitoring
    asyncio.create_task(performance_monitor.start_monitoring(interval=10.0))
    
    logger.info("Optimization systems initialized")

# Track connected clients to avoid stopping on non-last disconnects
CLIENTS: Set[WebSocketServerProtocol] = set()

# ---------- helpers ----------

def origin_ok(ws: WebSocketServerProtocol) -> bool:
    if not _ALLOWED_ORIGINS:
        return True
    origin = ws.request_headers.get("Origin")
    if origin is None and ALLOW_NO_ORIGIN:
        return True  # allow Python/CLI clients without Origin header
    return bool(origin) and origin in _ALLOWED_ORIGINS

def path_ok(request_path: str) -> bool:
    if PATH == "/":
        return True  # root
    cleaned = (request_path or "/").split("?", 1)[0].rstrip("/") or "/"
    return cleaned == PATH

def _parse_commandish_string(s: str) -> Tuple[Optional[str], dict]:
    t = (s or "").strip()
    if not t:
        return (None, {})
    parts = t.replace(":", " ").replace(",", " ").split()
    if not parts:
        return (None, {})
    cmd = parts[0]
    data: dict = {}
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            data[k] = v
        else:
            try:
                data.setdefault("value", int(p))
            except Exception:
                pass
    return (cmd, data)

def parse_json_or_string(msg: str) -> Tuple[Optional[str], dict]:
    try:
        data = json.loads(msg)
        if isinstance(data, dict):
            cmd = data.get("command")
            if isinstance(cmd, str) and cmd:
                return (cmd, data)
            cmd2 = data.get("cmd") or data.get("action")
            if isinstance(cmd2, str) and cmd2:
                return (cmd2, data)
    except Exception:
        pass
    if isinstance(msg, str):
        return _parse_commandish_string(msg)
    return (None, {})

async def buzz_on_safe():
    global buzzer_on_state
    async with buzzer_lock:
        try:
            buzz_on()
            buzzer_on_state = True
        except Exception as e:
            elog("buzzer on failed:", repr(e))

async def buzz_off_safe():
    global buzzer_on_state
    async with buzzer_lock:
        try:
            buzz_off()
            buzzer_on_state = False
        except Exception as e:
            elog("buzzer off failed:", repr(e))

def cancel_buzz_task():
    global _buzz_task
    if _buzz_task and not _buzz_task.done():
        _buzz_task.cancel()
    _buzz_task = None

async def do_stop():
    # Stop motors + buzzer
    async with motor_lock:
        try:
            motor.stop()
        except Exception as e:
            elog("motor stop failed:", repr(e))
    await buzz_off_safe()

def _call_motor(fn, speed: int):
    try:
        sig = inspect.signature(fn)
        if len(sig.parameters) == 0:
            fn()
        else:
            fn(speed)
    except (TypeError, ValueError):
        try:
            fn(speed)
        except Exception:
            fn()

async def do_move(fn_name: str, speed: int):
    async with motor_lock:
        fn = getattr(motor, fn_name, None)
        if not callable(fn):
            # Use gentle turns instead of sharp pivots
            if fn_name == "left":
                fn = getattr(motor, "left", None)
            elif fn_name == "right":
                fn = getattr(motor, "right", None)
        if not callable(fn):
            raise RuntimeError(f"motor missing method: {fn_name}")
        _call_motor(fn, speed)

def norm_cmd_name(raw: str) -> str:
    s = (raw or "").strip().lower()
    if s in {"forward", "move-up", "move-forward", "w"}: return "forward"
    if s in {"backward", "move-down", "move-back", "s"}: return "backward"
    if s in {"left", "move-left", "turn-left", "a"}:     return "left"
    if s in {"right", "move-right", "turn-right", "d"}:  return "right"
    if s in {"stop", "move-stop", "halt", ""}:           return "stop"
    return raw

def _extract_request_path(ws: WebSocketServerProtocol, request_path: Optional[str]) -> str:
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

# ---------- websocket handler (stop only on last disconnect) ----------

async def handler(ws: WebSocketServerProtocol, request_path: Optional[str] = None):
    global current_speed, current_horizontal_angle, current_vertical_angle, _last_move_op_id
    global _last_buzz_op_id, _buzz_task

    request_path = _extract_request_path(ws, request_path)
    peer = getattr(ws, "remote_address", None)

    CLIENTS.add(ws)
    log(f"CONNECTED {peer} path={request_path!r} clients={len(CLIENTS)}")

    try:
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

        await send_json(ws, {
            "type": "welcome",
            "status": "connected",
            "service": "movement",
            "ts": _now_ms(),
            "path": request_path or "/",
            "sim": SIM_MODE,
        })

        async for message in ws:
            try:
                # JSON heartbeat
                if isinstance(message, str) and message.startswith('{"type":"ping"'):
                    try:
                        obj = json.loads(message)
                        await send_json(ws, {"type": "pong", "ts": obj.get("ts", _now_ms())})
                    except Exception as e:
                        log_error(e, "Failed to process ping message", ws)
                        await send_json(ws, {"type": "pong", "ts": _now_ms()})
                    continue

                cmd, data = parse_json_or_string(message)
                if not cmd:
                    warn("non-JSON/unrecognized command; ignored")
                    continue

                cmd = norm_cmd_name(cmd)

                # reads
                try:
                    speed = int(data.get("speed", current_speed))
                except Exception as e:
                    log_error(e, f"Invalid speed parameter: {data.get('speed')}", ws)
                    speed = current_speed
                speed = clamp(speed, SPEED_MIN, SPEED_MAX)

                duration_raw = str(data.get("durationMs", "")).strip()
                try:
                    duration_ms = int(duration_raw) if duration_raw.isdigit() else 0
                except Exception as e:
                    log_error(e, f"Invalid duration parameter: {duration_raw}", ws)
                    duration_ms = 0

                try:
                    # -------- MOVEMENT --------
                    if cmd in {"forward", "backward", "left", "right"}:
                        current_speed = speed
                        await do_move(cmd, current_speed)
                        await send_json(ws, ok(cmd, speed=current_speed))

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
                                    log_error(e, "Auto-stop failed", ws)
                            asyncio.create_task(_auto_stop(duration_ms/1000.0, my_id))

                    elif cmd == "stop":
                        await do_stop()
                        await send_json(ws, ok("stop"))

                # -------- STRAIGHT ASSIST --------
                elif cmd in {"straight-assist", "straight-assist-config"}:
                    status_payload = STRAIGHT_ASSIST.status()

                    bool_enable = _as_bool(data.get("enable"), None)
                    if bool_enable is None:
                        bool_enable = _as_bool(data.get("enabled"), None)
                    if bool_enable is not None:
                        status_payload = STRAIGHT_ASSIST.set_enabled(bool_enable)

                    bool_disable = _as_bool(data.get("disable"), None)
                    if bool_disable is not None:
                        status_payload = STRAIGHT_ASSIST.set_enabled(not bool_disable)

                    step_val = _as_int(data.get("step"), None)
                    if step_val is not None:
                        status_payload = STRAIGHT_ASSIST.set_step(step_val)

                    max_val = _as_int(data.get("maxTrim", data.get("max_trim")), None)
                    if max_val is not None:
                        status_payload = STRAIGHT_ASSIST.set_max_trim(max_val)

                    left_val = data.get("leftTrim", data.get("left_trim"))
                    right_val = data.get("rightTrim", data.get("right_trim"))
                    trim_kwargs = {}
                    if left_val is not None:
                        parsed = _as_int(left_val, None)
                        if parsed is not None:
                            trim_kwargs["left"] = parsed
                    if right_val is not None:
                        parsed = _as_int(right_val, None)
                        if parsed is not None:
                            trim_kwargs["right"] = parsed
                    if trim_kwargs:
                        status_payload = STRAIGHT_ASSIST.set_trim(**trim_kwargs)

                    await send_json(ws, ok("straight-assist", straightAssist=status_payload))

                elif cmd == "straight-assist-nudge":
                    direction = data.get("direction") or data.get("drift") or data.get("dir")
                    amount = _as_int(data.get("amount", data.get("step", data.get("by"))), None)
                    if not direction:
                        await send_json(ws, err("invalid-straight-assist", detail="direction required"))
                    else:
                        try:
                            status_payload = STRAIGHT_ASSIST.nudge(direction, amount)
                        except ValueError as exc:
                            await send_json(ws, err("invalid-straight-assist", detail=str(exc)))
                        else:
                            await send_json(ws, ok("straight-assist-nudge", straightAssist=status_payload))

                elif cmd in {"straight-assist-reset", "straight-assist-clear"}:
                    status_payload = STRAIGHT_ASSIST.reset()
                    await send_json(ws, ok(cmd, straightAssist=status_payload))

                elif cmd == "straight-assist-status":
                    await send_json(ws, ok("straight-assist-status", straightAssist=STRAIGHT_ASSIST.status()))

                # -------- SPEED --------
                elif cmd == "increase-speed":
                    current_speed = clamp(current_speed + DEFAULT_SPEED_STEP, SPEED_MIN, SPEED_MAX)
                    await send_json(ws, ok("increase-speed", speed=current_speed))

                elif cmd == "decrease-speed":
                    current_speed = clamp(current_speed - DEFAULT_SPEED_STEP, SPEED_MIN, SPEED_MAX)
                    await send_json(ws, ok("decrease-speed", speed=current_speed))

                elif cmd == "set-speed":
                    try:
                        val = int(data.get("value", data.get("speed", current_speed)))
                    except Exception:
                        val = current_speed
                    current_speed = clamp(val, SPEED_MIN, SPEED_MAX)
                    await send_json(ws, ok("set-speed", speed=current_speed))

                # -------- BUZZER --------
                elif cmd == "buzz":
                    cancel_buzz_task()
                    await buzz_on_safe()
                    await send_json(ws, ok("buzz"))

                elif cmd == "buzz-stop":
                    cancel_buzz_task()
                    await buzz_off_safe()
                    await send_json(ws, ok("buzz-stop"))

                elif cmd == "buzz-for":
                    # one-shot beep for durationMs (10..10000 ms)
                    try:
                        dur = int(data.get("durationMs", 0))
                    except Exception:
                        dur = 0
                    dur = clamp(dur, 10, 10000)
                    if dur <= 0:
                        await send_json(ws, err("bad-duration"))
                    else:
                        cancel_buzz_task()
                        _last_buzz_op_id += 1
                        my_id = _last_buzz_op_id

                        async def _beep_once(ms: int, op_id: int):
                            try:
                                await buzz_on_safe()
                                await asyncio.sleep(ms / 1000.0)
                            except asyncio.CancelledError:
                                pass
                            finally:
                                # Only the latest op is allowed to turn off
                                if op_id == _last_buzz_op_id:
                                    await buzz_off_safe()

                        _buzz_task = asyncio.create_task(_beep_once(dur, my_id))
                        await send_json(ws, ok("buzz-for", durationMs=dur))

                elif cmd == "buzz-pulse":
                    # pulse pattern: onMs, offMs, repeat (defaults: 150,120,3)
                    def _ival(x, d): 
                        try:
                            return int(x)
                        except Exception:
                            return d
                    on_ms  = clamp(_ival(data.get("onMs"), 150), 5, 2000)
                    off_ms = clamp(_ival(data.get("offMs"), 120), 5, 3000)
                    repeat = clamp(_ival(data.get("repeat"), 3), 1, 50)

                    cancel_buzz_task()
                    _last_buzz_op_id += 1
                    my_id = _last_buzz_op_id

                    async def _pulse(on_ms: int, off_ms: int, n: int, op_id: int):
                        try:
                            for _ in range(n):
                                if op_id != _last_buzz_op_id:
                                    break
                                await buzz_on_safe()
                                await asyncio.sleep(on_ms / 1000.0)
                                if op_id != _last_buzz_op_id:
                                    break
                                await buzz_off_safe()
                                await asyncio.sleep(off_ms / 1000.0)
                        except asyncio.CancelledError:
                            pass
                        finally:
                            if op_id == _last_buzz_op_id:
                                await buzz_off_safe()

                    _buzz_task = asyncio.create_task(_pulse(on_ms, off_ms, repeat, my_id))
                    await send_json(ws, ok("buzz-pulse", onMs=on_ms, offMs=off_ms, repeat=repeat))

                # -------- AUTONOMY --------
                elif cmd == "autonomy-start":
                    mode = data.get("mode")
                    try:
                        status = await AUTONOMY.start(mode, data.get("params") or {})
                    except AutonomyError as exc:
                        await send_json(ws, err("autonomy-error", detail=str(exc)))
                    else:
                        await send_json(ws, ok("autonomy-start", autonomy=status))

                elif cmd == "autonomy-stop":
                    try:
                        status = await AUTONOMY.stop()
                    except AutonomyError as exc:
                        await send_json(ws, err("autonomy-error", detail=str(exc)))
                    else:
                        await send_json(ws, ok("autonomy-stop", autonomy=status))

                elif cmd == "autonomy-update":
                    try:
                        status = await AUTONOMY.update(data.get("params") or {})
                    except AutonomyError as exc:
                        await send_json(ws, err("autonomy-error", detail=str(exc)))
                    else:
                        await send_json(ws, ok("autonomy-update", autonomy=status))

                elif cmd == "autonomy-dock":
                    try:
                        status = await AUTONOMY.dock()
                    except AutonomyError as exc:
                        await send_json(ws, err("autonomy-error", detail=str(exc)))
                    else:
                        await send_json(ws, ok("autonomy-dock", autonomy=status))

                elif cmd == "autonomy-set_waypoint":
                    try:
                        status = await AUTONOMY.set_waypoint(
                            data.get("label", ""),
                            data.get("lat"),
                            data.get("lon"),
                        )
                    except AutonomyError as exc:
                        await send_json(ws, err("autonomy-error", detail=str(exc)))
                    else:
                        await send_json(ws, ok("autonomy-set_waypoint", autonomy=status))

                # -------- SERVOS --------
                elif cmd == "servo-horizontal":
                    delta = int(data.get("angle", 0))
                    current_horizontal_angle = clamp(current_horizontal_angle + delta, SERVO_MIN, SERVO_MAX)
                    try:
                        servo.setServoPwm("horizontal", current_horizontal_angle)
                    except Exception as e:
                        elog("servo H failed:", repr(e))
                        raise
                    await send_json(ws, ok(
                        "servo-horizontal",
                        angle=current_horizontal_angle,
                        **_servo_state_payload(current_horizontal_angle, current_vertical_angle),
                    ))

                elif cmd == "servo-vertical":
                    delta = int(data.get("angle", 0))
                    current_vertical_angle = clamp(current_vertical_angle + delta, SERVO_MIN, SERVO_MAX)
                    try:
                        servo.setServoPwm("vertical", current_vertical_angle)
                    except Exception as e:
                        elog("servo V failed:", repr(e))
                        raise
                    await send_json(ws, ok(
                        "servo-vertical",
                        angle=current_vertical_angle,
                        **_servo_state_payload(current_horizontal_angle, current_vertical_angle),
                    ))

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
                        await send_json(ws, ok(
                            "set-servo-position",
                            horizontal=current_horizontal_angle,
                            vertical=current_vertical_angle,
                            **_servo_state_payload(current_horizontal_angle, current_vertical_angle),
                        ))

                elif cmd == "reset-servo":
                    # Use current positions as the new "center" positions
                    # Horizontal: 90°, Vertical: 90°
                    current_horizontal_angle = 90
                    current_vertical_angle = 90
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(ws, ok(
                        "reset-servo",
                        horizontal=current_horizontal_angle,
                        vertical=current_vertical_angle,
                        **_servo_state_payload(current_horizontal_angle, current_vertical_angle),
                    ))

                # camera nudge aliases
                elif cmd == "camera-servo-left":
                    current_horizontal_angle = clamp(current_horizontal_angle - DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    await send_json(ws, ok(
                        "camera-servo-left",
                        angle=current_horizontal_angle,
                        **_servo_state_payload(current_horizontal_angle, current_vertical_angle),
                    ))

                elif cmd == "camera-servo-right":
                    current_horizontal_angle = clamp(current_horizontal_angle + DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("horizontal", current_horizontal_angle)
                    await send_json(ws, ok(
                        "camera-servo-right",
                        angle=current_horizontal_angle,
                        **_servo_state_payload(current_horizontal_angle, current_vertical_angle),
                    ))

                elif cmd == "camera-servo-up":
                    current_vertical_angle = clamp(current_vertical_angle + DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(ws, ok(
                        "camera-servo-up",
                        angle=current_vertical_angle,
                        **_servo_state_payload(current_horizontal_angle, current_vertical_angle),
                    ))

                elif cmd == "camera-servo-down":
                    current_vertical_angle = clamp(current_vertical_angle - DEFAULT_SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    servo.setServoPwm("vertical", current_vertical_angle)
                    await send_json(ws, ok(
                        "camera-servo-down",
                        angle=current_vertical_angle,
                        **_servo_state_payload(current_horizontal_angle, current_vertical_angle),
                    ))

                # -------- STATUS --------
                elif cmd == "status":
                    await send_json(ws, {
                        "type": "status",
                        "speed": current_speed,
                        "motors": get_cached_motor_telemetry(),
                        "servo": {
                            "horizontal": current_horizontal_angle,
                            "vertical": current_vertical_angle,
                            "min": SERVO_MIN,
                            "max": SERVO_MAX,
                        },
                        "buzzer": buzzer_on_state,
                        "autonomy": AUTONOMY.status(),
                        "straightAssist": STRAIGHT_ASSIST.status(),
                        "ts": _now_ms(),
                        "sim": SIM_MODE,
                    })

                else:
                    await send_json(ws, err(f"unknown-command:{cmd}"))

            except Exception as ex:
                log_error(ex, f"Command '{cmd}' failed", ws)
                await send_json(ws, err("exception", detail=str(ex)))

    except websockets.exceptions.ConnectionClosed as e:
        log(f"DISCONNECTED {peer} code={e.code} reason={e.reason or ''}")
    except Exception as e:
        log_error(e, "Handler error", ws)
    finally:
        CLIENTS.discard(ws)
        remaining = len(CLIENTS)
        if remaining == 0:
            # Cancel any buzzer pattern and stop for safety
            cancel_buzz_task()
            await do_stop()
            log("All clients disconnected -> STOP for safety")
        else:
            log(f"Client left; {remaining} client(s) still connected (motors unchanged)")

# ---------- serve ----------

async def main():
    log(f"listening on ws://0.0.0.0:{PORT}{'' if PATH=='/' else PATH}")
    log(f"ORIGIN_ALLOW={','.join(sorted(_ALLOWED_ORIGINS)) or '(none)'} "
        f"ALLOW_NO_ORIGIN={ALLOW_NO_ORIGIN} SIM_MODE={SIM_MODE}")
    async with websockets.serve(
        handler,
        "0.0.0.0",
        PORT,
        ping_interval=None,
        max_size=64 * 1024,
        max_queue=64,
    ):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
