# File: /Omega-Code/servers/robot_controller_backend/movement/movement_ws_server.py
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
import subprocess
from typing import Optional, Set, Tuple

# Add parent directory to path for autonomy module and proper package imports
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.append(ROOT)
# Add project root to path for absolute imports
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

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

# Global exception handler for asyncio event loop
def handle_global_exception(loop, context):
    """Handle unhandled exceptions in the event loop"""
    err = context.get("exception")
    msg = context.get("message", "Unknown error")
    print("🔥 [GLOBAL ERROR] Unhandled exception in event loop:")
    print(f"   message: {msg}")
    if err:
        print(f"   exception: {repr(err)}")
        print(f"   type: {type(err).__name__}")
        if hasattr(err, '__traceback__') and err.__traceback__:
            tb = err.__traceback__
            print(f"   location: {tb.tb_frame.f_code.co_filename}:{tb.tb_lineno}")
    logger.error(f"Global exception: {msg}", exc_info=err)

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

try:
    from servers.robot_controller_backend.autonomy import AutonomyError, build_default_controller
except ImportError:
    try:
        from ..autonomy import AutonomyError, build_default_controller
    except ImportError:
        from autonomy import AutonomyError, build_default_controller

try:
    from .straight_drive_assist import StraightDriveAssist
except Exception:  # pragma: no cover - fallback for script execution
    from straight_drive_assist import StraightDriveAssist

# Import MotorTelemetryController - this will try to load real PCA9685 first
MotorTelemetryController = None
try:
    from .motor_telemetry import MotorTelemetryController
    logger.info("MotorTelemetryController imported successfully")
except Exception as e:  # pragma: no cover - fallback for script execution
    try:
        from servers.robot_controller_backend.movement.motor_telemetry import MotorTelemetryController
        logger.info("MotorTelemetryController imported successfully (absolute)")
    except Exception:
        try:
            from motor_telemetry import MotorTelemetryController
        except Exception:
            logger.warning(f"MotorTelemetryController not available - telemetry disabled: {repr(e)}")
            MotorTelemetryController = None

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

# Import PID controllers for Movement V2
try:
    from .movement_pid import SpeedPID, PIDTuning
    PID_AVAILABLE = True
except ImportError:
    try:
        from servers.robot_controller_backend.movement.movement_pid import SpeedPID, PIDTuning
        PID_AVAILABLE = True
    except ImportError:
        logger.warning("PID controllers not available - PID features disabled")
        PID_AVAILABLE = False
        SpeedPID = None
        PIDTuning = None

# Import Movement V2 modules
try:
    from .thermal_safety import ThermalSafety, ThermalLimits, SafetyState
    from .movement_ramp import MovementRamp, RampType
    from .movement_profiles import ProfileManager, ProfileType
    from .movement_watchdog import MovementWatchdog, WatchdogState
    from .movement_config import load_config, MovementV2Config
    MOVEMENT_V2_AVAILABLE = True
except ImportError:
    try:
        from servers.robot_controller_backend.movement.thermal_safety import ThermalSafety, ThermalLimits, SafetyState
        from servers.robot_controller_backend.movement.movement_ramp import MovementRamp, RampType
        from servers.robot_controller_backend.movement.movement_profiles import ProfileManager, ProfileType
        from servers.robot_controller_backend.movement.movement_watchdog import MovementWatchdog, WatchdogState
        from servers.robot_controller_backend.movement.movement_config import load_config, MovementV2Config
        MOVEMENT_V2_AVAILABLE = True
    except ImportError as e:
        logger.warning(f"Movement V2 modules not available: {e}")
        MOVEMENT_V2_AVAILABLE = False
        ThermalSafety = None
        MovementRamp = None
        ProfileManager = None
        MovementWatchdog = None

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
# Use optimized hardware flags module for cached SIM_MODE check
try:
    from servers.robot_controller_backend.hardware.hardware_flags import is_sim
    SIM_MODE = is_sim()
except ImportError:
    # Fallback to direct check if hardware_flags not available
    SIM_MODE = _env("ROBOT_SIM", "0") == "1" or _env("SIM_MODE", "0") == "1"

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

# Hardware initialization with proper logging and fallback order
telemetry_enabled = False
motor_driver_type = None

if SIM_MODE:
    log("[MOVE][INIT] SIM_MODE=1 → using NOOP devices")
    motor = _NoopMotor()
    servo = _NoopServo()
    motor_driver_type = "NOOP (SIM_MODE=True)"
    log("[MOVE][INIT] Motor driver: NOOP")
    log("[MOVE][INIT] Telemetry: DISABLED")
else:
    # Hardware initialization order:
    # (1) Try MotorTelemetryController (with real PCA9685 + Motor)
    # (2) Fallback to basic Motor (with real PCA9685)
    # (3) Fallback to NOOP (only if hardware completely unavailable)
    
    try:
        # Import servo and buzzer first (these are less critical)
        try:
            from servers.robot_controller_backend.controllers.servo_control import Servo as _Servo
        except ImportError:
            try:
                from ..controllers.servo_control import Servo as _Servo
            except ImportError:
                from controllers.servo_control import Servo as _Servo
        
        try:
            from servers.robot_controller_backend.controllers.buzzer import setup_buzzer, buzz_on as _buzz_on, buzz_off as _buzz_off
        except ImportError:
            try:
                from ..controllers.buzzer import setup_buzzer, buzz_on as _buzz_on, buzz_off as _buzz_off
            except ImportError:
                from controllers.buzzer import setup_buzzer, buzz_on as _buzz_on, buzz_off as _buzz_off
        
        servo = _Servo()
        buzz_on = _buzz_on
        buzz_off = _buzz_off
        
        # Try to initialize buzzer safely
        try:
            setup_buzzer()
        except Exception as e:
            warn("buzzer setup failed (continuing):", repr(e))
        
        # Now try motor initialization in priority order
        # Priority 1: MotorTelemetryController (with telemetry)
        try:
            if MotorTelemetryController:
                motor = MotorTelemetryController()
                telemetry_enabled = True
                motor_driver_type = "MotorTelemetryController"
                log("[MOVE][INIT] Motor driver: PCA9685")
                log("[MOVE][INIT] Controller: MotorTelemetryController")
                log("[MOVE][INIT] Telemetry: ENABLED")
            else:
                raise ImportError("MotorTelemetryController not available")
        except Exception as e:
            # Priority 2: Basic Motor (without telemetry)
            warn(f"MotorTelemetryController failed: {repr(e)}, trying basic Motor...")
            try:
                from servers.robot_controller_backend.movement.minimal_motor_control import Motor as _Motor
            except ImportError:
                try:
                    from .minimal_motor_control import Motor as _Motor
                except ImportError:
                    from minimal_motor_control import Motor as _Motor
            
            motor = _Motor()
            telemetry_enabled = False
            motor_driver_type = "Motor (basic)"
            log("[MOVE][INIT] Motor driver: PCA9685")
            log("[MOVE][INIT] Controller: Motor (basic)")
            log("[MOVE][INIT] Telemetry: DISABLED")
        
        log(f"[MOVE][INIT] HW init ok: motor={motor_driver_type}, servo={type(servo).__name__}")
        
    except Exception as e:
        # Priority 3: NOOP fallback (only if hardware completely unavailable)
        elog(f"[MOVE][INIT] Hardware import failed → falling back to NOOP: {repr(e)}")
        motor = _NoopMotor()
        servo = _NoopServo()
        motor_driver_type = "NOOP (fallback)"
        log("[MOVE][INIT] Motor driver: NOOP (fallback)")
        log("[MOVE][INIT] Telemetry: DISABLED")
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
if motor and hasattr(motor, 'get_telemetry'):
    MOTOR_TELEMETRY = motor
    if OPTIMIZATION_AVAILABLE:
        @cached(ttl=1, key_prefix="motor_telemetry:")  # Cache for 1 second
        def get_cached_motor_telemetry():
            return MOTOR_TELEMETRY.get_telemetry()
    else:
        def get_cached_motor_telemetry():
            return MOTOR_TELEMETRY.get_telemetry()
else:
    MOTOR_TELEMETRY = None
    def get_cached_motor_telemetry():
        return {}  # Return empty dict if telemetry not available

# Initialize Movement V2 system
MOVEMENT_V2_ENABLED = _env("MOVEMENT_V2_ENABLED", "1") == "1" and MOVEMENT_V2_AVAILABLE

# Load Movement V2 configuration
MOVEMENT_V2_CONFIG = None
if MOVEMENT_V2_ENABLED:
    try:
        MOVEMENT_V2_CONFIG = load_config()
        log("[MOVE][V2] Movement V2 configuration loaded")
    except Exception as e:
        logger.warning(f"Failed to load Movement V2 config: {e}")
        MOVEMENT_V2_CONFIG = None

# Initialize PID controller for speed regulation
SPEED_PID = None
PID_ENABLED = _env("MOVEMENT_PID_ENABLED", "1") == "1"
if PID_AVAILABLE and PID_ENABLED and MOTOR_TELEMETRY:
    try:
        # Get PID tuning from config or environment
        if MOVEMENT_V2_CONFIG:
            pid_kp = MOVEMENT_V2_CONFIG.pid_kp
            pid_ki = MOVEMENT_V2_CONFIG.pid_ki
            pid_kd = MOVEMENT_V2_CONFIG.pid_kd
            pid_kf = MOVEMENT_V2_CONFIG.pid_kf
        else:
            pid_kp = float(_env("MOVEMENT_PID_KP", "0.3"))
            pid_ki = float(_env("MOVEMENT_PID_KI", "0.05"))
            pid_kd = float(_env("MOVEMENT_PID_KD", "0.01"))
            pid_kf = float(_env("MOVEMENT_PID_KF", "0.0"))
        
        SPEED_PID = SpeedPID(
            tuning=PIDTuning(kp=pid_kp, ki=pid_ki, kd=pid_kd, kf=pid_kf),
            max_rpm=300.0,  # Maximum expected RPM
            max_pwm_correction=500.0  # Maximum PWM correction (±500)
        )
        SPEED_PID.enable()
        log(f"[MOVE][PID] Speed PID initialized: kp={pid_kp}, ki={pid_ki}, kd={pid_kd}")
    except Exception as e:
        logger.warning(f"Failed to initialize PID controller: {e}")
        SPEED_PID = None
else:
    if not PID_AVAILABLE:
        log("[MOVE][PID] PID not available - speed regulation disabled")
    elif not PID_ENABLED:
        log("[MOVE][PID] PID disabled via MOVEMENT_PID_ENABLED=0")
    elif not MOTOR_TELEMETRY:
        log("[MOVE][PID] PID disabled - motor telemetry not available")

# Initialize Movement V2 components
THERMAL_SAFETY = None
MOVEMENT_RAMP = None
PROFILE_MANAGER = None
MOVEMENT_WATCHDOG = None

if MOVEMENT_V2_ENABLED and MOVEMENT_V2_CONFIG:
    try:
        # Initialize thermal safety
        if MOVEMENT_V2_CONFIG.thermal_enabled:
            thermal_limits = ThermalLimits(
                max_temp=MOVEMENT_V2_CONFIG.thermal_max_temp,
                warning_temp=MOVEMENT_V2_CONFIG.thermal_warning_temp,
                max_current=MOVEMENT_V2_CONFIG.thermal_max_current,
                warning_current=MOVEMENT_V2_CONFIG.thermal_warning_current,
                cooldown_temp=MOVEMENT_V2_CONFIG.thermal_cooldown_temp,
                throttle_factor=MOVEMENT_V2_CONFIG.thermal_throttle_factor
            )
            THERMAL_SAFETY = ThermalSafety(limits=thermal_limits, enabled=True)
            log(f"[MOVE][V2] Thermal safety initialized: max_temp={thermal_limits.max_temp}°C")
        
        # Initialize movement ramping
        ramp_type_map = {
            "linear": RampType.LINEAR,
            "exponential": RampType.EXPONENTIAL,
            "s_curve": RampType.S_CURVE
        }
        ramp_type = ramp_type_map.get(MOVEMENT_V2_CONFIG.ramp_type, RampType.LINEAR)
        MOVEMENT_RAMP = MovementRamp(
            accel_rate=MOVEMENT_V2_CONFIG.accel_rate,
            decel_rate=MOVEMENT_V2_CONFIG.decel_rate,
            ramp_type=ramp_type
        )
        log(f"[MOVE][V2] Movement ramping initialized: {MOVEMENT_V2_CONFIG.ramp_type}")
        
        # Initialize profile manager
        profile_map = {
            "smooth": ProfileType.SMOOTH,
            "aggressive": ProfileType.AGGRESSIVE,
            "precision": ProfileType.PRECISION
        }
        default_profile = profile_map.get(MOVEMENT_V2_CONFIG.default_profile, ProfileType.SMOOTH)
        PROFILE_MANAGER = ProfileManager(default_profile=default_profile)
        log(f"[MOVE][V2] Profile manager initialized: default={MOVEMENT_V2_CONFIG.default_profile}")
        
        # Initialize watchdog
        if MOVEMENT_V2_CONFIG.watchdog_enabled:
            # Watchdog callback is synchronous - background task handles async stop
            MOVEMENT_WATCHDOG = MovementWatchdog(
                timeout_sec=MOVEMENT_V2_CONFIG.watchdog_timeout_sec,
                stop_callback=None,  # Background task handles stopping
                enabled=True
            )
            log(f"[MOVE][V2] Watchdog initialized: timeout={MOVEMENT_V2_CONFIG.watchdog_timeout_sec}s")
        
        log("[MOVE][V2] Movement V2 system initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize Movement V2 components: {e}")
        import traceback
        traceback.print_exc()
        MOVEMENT_V2_ENABLED = False
else:
    if not MOVEMENT_V2_AVAILABLE:
        log("[MOVE][V2] Movement V2 modules not available")
    elif not MOVEMENT_V2_ENABLED:
        log("[MOVE][V2] Movement V2 disabled via MOVEMENT_V2_ENABLED=0")

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
    
    # Note: Task processor and performance monitor will be started in main()
    # after the event loop is running (Python 3.11+ requirement)
    
    logger.info("Optimization systems initialized")

# Track connected clients to avoid stopping on non-last disconnects
CLIENTS: Set[WebSocketServerProtocol] = set()

# Connection limits and cleanup
MAX_CLIENTS = 10  # Limit concurrent connections
CONNECTION_TIMEOUT = 300  # 5 minutes idle timeout

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
    global current_speed
    async with motor_lock:
        try:
            motor.stop()
        except Exception as e:
            elog("motor stop failed:", repr(e))
    await buzz_off_safe()
    # Reset speed to 0 on stop for UI sync
    current_speed = 0

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
        log(f"[MOVE] do_move({fn_name}, {speed})")
        
        # Reset watchdog on any movement command
        if MOVEMENT_WATCHDOG:
            MOVEMENT_WATCHDOG.kick()
        
        fn = getattr(motor, fn_name, None)
        actual_method = fn_name
        if not callable(fn):
            # Map left/right to pivot turns (A/D keys use pivot turns)
            if fn_name == "left":
                fn = getattr(motor, "pivot_left", None)
                actual_method = "pivot_left"
            elif fn_name == "right":
                fn = getattr(motor, "pivot_right", None)
                actual_method = "pivot_right"
        if not callable(fn):
            elog(f"[MOVE] motor missing method: {fn_name}")
            raise RuntimeError(f"motor missing method: {fn_name}")
        
        # Movement V2 Pipeline:
        # 1. Profile transformation (if enabled)
        profiled_speed = speed
        if PROFILE_MANAGER:
            try:
                current_profile = PROFILE_MANAGER.get_current_profile()
                profiled_speed = int(current_profile.transform_speed(speed))
                # Update ramp rates from profile
                if MOVEMENT_RAMP:
                    MOVEMENT_RAMP.accel_rate = current_profile.get_accel_rate()
                    MOVEMENT_RAMP.decel_rate = current_profile.get_decel_rate()
            except Exception as e:
                logger.warning(f"Profile transformation failed: {e}")
        
        # 2. Movement ramping (if enabled)
        ramped_speed = profiled_speed
        if MOVEMENT_RAMP:
            try:
                MOVEMENT_RAMP.set_target(float(profiled_speed))
                dt = 1.0 / MOVEMENT_V2_CONFIG.ramp_update_rate_hz if MOVEMENT_V2_CONFIG else 0.02
                ramped_speed = int(MOVEMENT_RAMP.update(dt))
                # Check for ramp stall (no progress)
                if MOVEMENT_RAMP._is_ramping and abs(MOVEMENT_RAMP.current_pwm - MOVEMENT_RAMP.target_pwm) > 10:
                    elapsed = time.monotonic() - MOVEMENT_RAMP._ramp_start_time
                    if elapsed > 5.0:  # Stalled for more than 5 seconds
                        print(f"⚠️ [RAMPING] Ramp stalled — command blocked or hardware busy")
                        print(f"   Current: {MOVEMENT_RAMP.current_pwm:.0f}, Target: {MOVEMENT_RAMP.target_pwm:.0f}")
                        print(f"   Elapsed: {elapsed:.1f}s")
            except Exception as e:
                logger.warning(f"Ramping failed: {e}, using profiled speed")
                ramped_speed = profiled_speed
        
        # 3. Thermal safety check (if enabled)
        safe_speed = ramped_speed
        if THERMAL_SAFETY and MOTOR_TELEMETRY:
            try:
                telemetry = get_cached_motor_telemetry()
                safety_state = THERMAL_SAFETY.check(telemetry)
                
                if safety_state == SafetyState.KILL:
                    print("🔥 [THERMAL][KILL] Motor kill triggered - stopping motors")
                    log("[MOVE][THERMAL] Motor kill triggered - stopping motors")
                    motor.stop()
                    return
                
                # Apply thermal throttling
                safe_speed = int(THERMAL_SAFETY.apply_throttle(ramped_speed))
                
                if safety_state == SafetyState.THROTTLE:
                    print(f"⚠️ [THERMAL][THROTTLE] Throttling applied: {ramped_speed} -> {safe_speed}")
                    log(f"[MOVE][THERMAL] Throttling applied: {ramped_speed} -> {safe_speed}")
            except Exception as e:
                logger.warning(f"Thermal safety check failed: {e}, using ramped speed")
                safe_speed = ramped_speed
        
        # 4. PID correction (if enabled and telemetry available)
        final_speed = safe_speed
        if SPEED_PID and SPEED_PID.enabled and MOTOR_TELEMETRY:
            try:
                # Get current telemetry
                telemetry = get_cached_motor_telemetry()
                
                # Calculate average RPM from telemetry
                motor_keys = ['frontLeft', 'frontRight', 'rearLeft', 'rearRight']
                rpm_values = [
                    telemetry.get(m, {}).get('speed', 0.0) 
                    for m in motor_keys 
                    if m in telemetry
                ]
                current_rpm = sum(rpm_values) / len(rpm_values) if rpm_values else 0.0
                
                # Estimate target RPM from PWM (rough conversion: PWM 0-4095 -> RPM 0-300)
                target_rpm = (abs(safe_speed) / 4095.0) * 300.0
                SPEED_PID.set_target_rpm(target_rpm)
                
                # Compute PID correction (dt = 0.1s for 10Hz update rate)
                dt = 0.1
                correction = SPEED_PID.compute(current_rpm=current_rpm, dt=dt)
                
                # Apply correction to speed (maintain direction)
                if safe_speed >= 0:
                    final_speed = max(0, min(4095, int(safe_speed + correction)))
                else:
                    final_speed = max(-4095, min(0, int(safe_speed - correction)))
                
                if abs(correction) > 1.0:  # Only log significant corrections
                    log(f"[MOVE][PID] Correction: {correction:.1f} PWM (target: {target_rpm:.1f} RPM, current: {current_rpm:.1f} RPM)")
            except Exception as e:
                logger.warning(f"PID correction failed: {e}, using safe speed")
                final_speed = safe_speed
        
        log(f"[MOVE] Calling motor.{actual_method}({final_speed}) [V2 pipeline: {speed} -> {profiled_speed} -> {ramped_speed} -> {safe_speed} -> {final_speed}]")
        _call_motor(fn, final_speed)

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

    # Check connection limit
    if len(CLIENTS) >= MAX_CLIENTS:
        await send_json(ws, err("server-busy", message="Too many connections"))
        await ws.close(code=1013, reason="Server at capacity")
        log(f"REJECTED {peer} - server at capacity ({len(CLIENTS)}/{MAX_CLIENTS})")
        return

    # Track connection start time and last activity
    ws._connection_start = time.time()
    ws._last_activity = time.time()

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
            # Update last activity timestamp
            ws._last_activity = time.time()
            
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
                log(f"[CMD] Received: {cmd!r} (normalized from {parse_json_or_string(message)[0]!r})")

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

                # -------- MOVEMENT --------
                if cmd in {"forward", "backward", "left", "right"}:
                    log(f"[CMD] Executing movement: {cmd} at speed {speed}")
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
                    log(f"[CMD] Executing stop")
                    await do_stop()
                    await send_json(ws, ok("stop", speed=current_speed))

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
                            await send_json(ws, ok("straight-assist-nudge", straightAssist=status_payload))
                        except ValueError as exc:
                            await send_json(ws, err("invalid-straight-assist", detail=str(exc)))

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

                # -------- MOVEMENT V2 PROFILE --------
                elif cmd == "set-profile":
                    if PROFILE_MANAGER:
                        profile_name = data.get("profile", "").lower()
                        profile_map = {
                            "smooth": ProfileType.SMOOTH,
                            "aggressive": ProfileType.AGGRESSIVE,
                            "precision": ProfileType.PRECISION
                        }
                        if profile_name in profile_map:
                            PROFILE_MANAGER.set_profile(profile_map[profile_name])
                            await send_json(ws, ok("set-profile", profile=profile_name, profileInfo=PROFILE_MANAGER.get_current_profile().get_config()))
                        else:
                            await send_json(ws, err("invalid-profile", valid=["smooth", "aggressive", "precision"]))
                    else:
                        await send_json(ws, err("profiles-not-available"))

                elif cmd == "get-profile":
                    if PROFILE_MANAGER:
                        current_profile = PROFILE_MANAGER.get_current_profile()
                        await send_json(ws, ok("get-profile", profile=current_profile.name, profileInfo=current_profile.get_config()))
                    else:
                        await send_json(ws, err("profiles-not-available"))

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
                    status_payload = {
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
                    }
                    
                    # Add PID status if available
                    if SPEED_PID:
                        status_payload["pid"] = {
                            "enabled": SPEED_PID.enabled,
                            "target_rpm": SPEED_PID.target_rpm,
                            "tuning": {
                                "kp": SPEED_PID.tuning.kp,
                                "ki": SPEED_PID.tuning.ki,
                                "kd": SPEED_PID.tuning.kd,
                                "kf": SPEED_PID.tuning.kf,
                            }
                        }
                    else:
                        status_payload["pid"] = {
                            "enabled": False,
                            "available": PID_AVAILABLE
                        }
                    
                    # Add Movement V2 status
                    if MOVEMENT_V2_ENABLED:
                        v2_status = {
                            "enabled": True,
                        }
                        
                        if PROFILE_MANAGER:
                            current_profile = PROFILE_MANAGER.get_current_profile()
                            v2_status["profile"] = {
                                "name": current_profile.name,
                                "config": current_profile.get_config()
                            }
                        
                        if MOVEMENT_RAMP:
                            v2_status["ramping"] = {
                                "current_pwm": MOVEMENT_RAMP.current_pwm,
                                "target_pwm": MOVEMENT_RAMP.target_pwm,
                                "is_ramping": MOVEMENT_RAMP._is_ramping,
                                "accel_rate": MOVEMENT_RAMP.accel_rate,
                                "decel_rate": MOVEMENT_RAMP.decel_rate,
                                "ramp_type": MOVEMENT_RAMP.ramp_type.value if hasattr(MOVEMENT_RAMP.ramp_type, 'value') else str(MOVEMENT_RAMP.ramp_type)
                            }
                        
                        if THERMAL_SAFETY:
                            v2_status["thermal"] = THERMAL_SAFETY.get_status()
                        
                        if MOVEMENT_WATCHDOG:
                            v2_status["watchdog"] = MOVEMENT_WATCHDOG.get_status()
                        
                        status_payload["movementV2"] = v2_status
                    else:
                        status_payload["movementV2"] = {
                            "enabled": False,
                            "available": MOVEMENT_V2_AVAILABLE
                        }
                    
                    await send_json(ws, status_payload)

                else:
                        await send_json(ws, err(f"unknown-command:{cmd}"))

            except Exception as e:
                log_error(e, "Error processing message", ws)
                await send_json(ws, err(f"error: {str(e)}"))

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

async def cleanup_stale_connections():
    """Periodically clean up stale/idle connections"""
    while True:
        await asyncio.sleep(60)  # Check every minute
        current_time = time.time()
        stale_clients = []
        
        for client in CLIENTS.copy():
            last_activity = getattr(client, '_last_activity', current_time)
            if current_time - last_activity > CONNECTION_TIMEOUT:
                stale_clients.append(client)
        
        for client in stale_clients:
            try:
                await client.close(code=1008, reason="Idle timeout")
                log(f"Closed stale connection: {getattr(client, 'remote_address', 'unknown')}")
            except Exception as e:
                log_error(e, "Error closing stale connection", client)
            finally:
                CLIENTS.discard(client)
        
        if stale_clients:
            log(f"Cleaned up {len(stale_clients)} stale connection(s)")

async def watchdog_background_task():
    """Background task to check watchdog timer"""
    if not MOVEMENT_WATCHDOG or not MOVEMENT_WATCHDOG.enabled:
        return
    
    while True:
        await asyncio.sleep(0.1)  # Check every 100ms
        if MOVEMENT_WATCHDOG.should_stop():
            # Watchdog triggered - stop motors
            async with motor_lock:
                try:
                    print("⏱️ [WATCHDOG] Watchdog triggered - stopping motors")
                    motor.stop()
                    log("[MOVE][WATCHDOG] Watchdog triggered - motors stopped")
                except Exception as e:
                    print(f"🔥 [WATCHDOG][ERROR] Failed to stop motors: {repr(e)}")
                    logger.error(f"Watchdog stop failed: {e}")

async def main():
    # Set global exception handler for this event loop
    loop = asyncio.get_event_loop()
    loop.set_exception_handler(handle_global_exception)
    
    log(f"listening on ws://0.0.0.0:{PORT}{'' if PATH=='/' else PATH}")
    log(f"ORIGIN_ALLOW={','.join(sorted(_ALLOWED_ORIGINS)) or '(none)'} "
        f"ALLOW_NO_ORIGIN={ALLOW_NO_ORIGIN} SIM_MODE={SIM_MODE}")
    log(f"Connection limits: MAX_CLIENTS={MAX_CLIENTS}, CONNECTION_TIMEOUT={CONNECTION_TIMEOUT}s")
    
    if MOVEMENT_V2_ENABLED:
        log(f"[MOVE][V2] Movement V2 enabled: Ramping={MOVEMENT_RAMP is not None}, "
            f"Thermal={THERMAL_SAFETY is not None}, Watchdog={MOVEMENT_WATCHDOG is not None}, "
            f"Profiles={PROFILE_MANAGER is not None}")
    
    # Start optimization systems (must be done after event loop is running)
    if OPTIMIZATION_AVAILABLE:
        # Start async task processor
        asyncio.create_task(task_processor.start())
        
        # Start performance monitoring
        asyncio.create_task(performance_monitor.start_monitoring(interval=10.0))
        
        logger.info("Optimization systems started")
    
    # Start Movement V2 watchdog background task
    if MOVEMENT_WATCHDOG and MOVEMENT_WATCHDOG.enabled:
        asyncio.create_task(watchdog_background_task())
        log("[MOVE][V2] Watchdog background task started")
    
    # Start cleanup task
    asyncio.create_task(cleanup_stale_connections())
    
    async with websockets.serve(
        handler,
        "0.0.0.0",
        PORT,
        ping_interval=20,      # Send ping every 20 seconds
        ping_timeout=10,       # Close if no pong within 10 seconds
        close_timeout=10,      # Wait 10s for graceful close
        max_size=32 * 1024,    # Reduce from 64KB to 32KB
        max_queue=32,          # Reduce from 64 to 32
    ):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        # Set global exception handler before running
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.set_exception_handler(handle_global_exception)
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n⚠️ [SHUTDOWN] Keyboard interrupt received - shutting down gracefully")
        pass
    except Exception as e:
        print("🔥 [FATAL] movement_ws_server crashed")
        print(f"   Type: {type(e).__name__}")
        print(f"   Error: {str(e)}")
        if hasattr(e, '__traceback__') and e.__traceback__:
            tb = e.__traceback__
            print(f"   Location: {tb.tb_frame.f_code.co_filename}:{tb.tb_lineno}")
            print(f"   Function: {tb.tb_frame.f_code.co_name}")
        print("\n   Troubleshooting:")
        print("   - If PCA9685 or smbus2 error: verify I2C address, permissions, and driver load")
        print("   - Check hardware connections (SDA/SCL)")
        print("   - Verify environment variables are set correctly")
        print("   - Check logs above for specific hardware initialization errors")
        raise
