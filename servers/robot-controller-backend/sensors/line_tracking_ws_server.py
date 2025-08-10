# File: /Omega-Code/servers/robot-controller-backend/sensors/line_tracking_ws_server.py
# Summary:
# WebSocket server for streaming line-tracking sensor states (3x IR) to the UI.
#
# Improvements:
# - Tolerant path check (allows /line-tracker, /line-tracker/, query strings)
# - Resilient GPIO init with clear logs + FORCE_SIM override
# - Falls back to simulation instead of crashing if GPIO fails
# - Live rate changes still apply (period recomputed each loop)
# - Welcome envelope + JSON heartbeat (ping/pong)

import os
import asyncio
import json
import time
import urllib.parse
import sys

# ---------- Config ----------
PIN_LEFT   = int(os.getenv("PIN_LEFT", "14"))
PIN_CENTER = int(os.getenv("PIN_CENTER", "15"))
PIN_RIGHT  = int(os.getenv("PIN_RIGHT", "23"))
SENSOR_PINS = {"left": PIN_LEFT, "center": PIN_CENTER, "right": PIN_RIGHT}

INVERT = {
    "left":   os.getenv("INVERT_LEFT", "0") == "1",
    "center": os.getenv("INVERT_CENTER", "0") == "1",
    "right":  os.getenv("INVERT_RIGHT", "0") == "1",
}

RATE_HZ = float(os.getenv("RATE_HZ", "10"))
KEEPALIVE_SEC = float(os.getenv("KEEPALIVE_SEC", "2.0"))

HOST = os.getenv("LINE_TRACKER_HOST", "0.0.0.0")
PORT = int(os.getenv("LINE_TRACKER_PORT", "8090"))
PATH = os.getenv("LINE_TRACKER_PATH", "/line-tracker").rstrip("/") or "/line-tracker"

FORCE_SIM = os.getenv("FORCE_SIM", "0") == "1"

# ---------- GPIO backends ----------
ON_PI = False
USE_LGPIO = False
USE_RPIGPIO = False
_lgpio_h = None
RGPIO = None  # set when imported

def _log(*args):
    print("[LineTracker]", *args, file=sys.stdout, flush=True)

def _try_import_gpio():
    global ON_PI, USE_LGPIO, USE_RPIGPIO, RGPIO
    if FORCE_SIM:
        _log("FORCE_SIM=1 → running in simulation mode (no GPIO).")
        ON_PI = False
        return
    try:
        import lgpio  # type: ignore
        globals()['lgpio'] = lgpio
        USE_LGPIO = True
        ON_PI = True
        _log("Using lgpio backend.")
    except Exception as e:
        _log(f"lgpio import failed: {e!r} → trying RPi.GPIO")
        try:
            import RPi.GPIO as _RGPIO  # type: ignore
            RGPIO = _RGPIO
            USE_RPIGPIO = True
            ON_PI = True
            _log("Using RPi.GPIO backend.")
        except Exception as e2:
            _log(f"RPi.GPIO import failed: {e2!r} → simulation mode.")
            ON_PI = False

def _setup_gpio():
    """Attempt to initialize GPIO. If it fails, fall back to simulation."""
    global _lgpio_h
    if FORCE_SIM:
        return
    try:
        if USE_LGPIO:
            _lgpio_h = lgpio.gpiochip_open(0)
            for pin in SENSOR_PINS.values():
                lgpio.gpio_claim_input(_lgpio_h, pin)
            _log("lgpio initialized.")
        elif USE_RPIGPIO:
            RGPIO.setwarnings(False)
            RGPIO.setmode(RGPIO.BCM)
            for pin in SENSOR_PINS.values():
                RGPIO.setup(pin, RGPIO.IN)
            _log("RPi.GPIO initialized.")
        else:
            _log("No GPIO backend available → simulation mode.")
    except Exception as e:
        _log(f"GPIO setup failed: {e!r} → simulation mode.")
        # Ensure we don't partial-open
        try:
            if USE_LGPIO and _lgpio_h is not None:
                lgpio.gpiochip_close(_lgpio_h)
        except Exception:
            pass
        _lgpio_h = None

def _cleanup_gpio():
    global _lgpio_h
    try:
        if USE_LGPIO and _lgpio_h is not None:
            lgpio.gpiochip_close(_lgpio_h)
            _lgpio_h = None
            _log("lgpio cleaned up.")
        elif USE_RPIGPIO:
            RGPIO.cleanup()
            _log("RPi.GPIO cleaned up.")
    except Exception as e:
        _log(f"GPIO cleanup error (ignored): {e!r}")

def _gpio_read(pin: int) -> int:
    if USE_LGPIO and _lgpio_h is not None:
        try:
            return 1 if lgpio.gpio_read(_lgpio_h, pin) else 0
        except Exception:
            return 0
    if USE_RPIGPIO:
        try:
            return int(RGPIO.input(pin))
        except Exception:
            return 0
    # Simulation for local dev: deterministic blinking
    t = int(time.time() * 2)  # 2 Hz flip
    return 1 if (pin + t) % 2 == 0 else 0

def read_sensors() -> dict:
    raw = {name: _gpio_read(pin) for name, pin in SENSOR_PINS.items()}
    # Apply inversion (some IR boards output 0 on black line)
    inv = {k: (1 - v) if INVERT[k] else v for k, v in raw.items()}
    return inv

import websockets
from websockets.server import WebSocketServerProtocol

def _path_ok(request_path: str) -> bool:
    # tolerate trailing slash and query string
    parsed = urllib.parse.urlparse(request_path or "")
    req = parsed.path.rstrip("/")
    return (req or "/") == PATH

# ---------- WS tasks ----------
async def producer(websocket: WebSocketServerProtocol):
    """Send samples at RATE_HZ, change-only, with periodic keepalives."""
    last_state = None
    last_sent = 0.0

    while True:
        period = 1.0 / max(RATE_HZ, 0.1)  # live-applied rate
        state = read_sensors()
        now = time.time()
        changed = (state != last_state)
        due_keepalive = (now - last_sent) >= KEEPALIVE_SEC

        if changed or due_keepalive:
            payload = {
                "type": "sample",
                "status": "success",
                "service": "line-tracker",
                "lineTracking": state,
                "pins": SENSOR_PINS,
                "invert": INVERT,
                "timestamp": now,
            }
            await websocket.send(json.dumps(payload))
            last_state = state
            last_sent = now

        await asyncio.sleep(period)

async def consumer(websocket: WebSocketServerProtocol):
    """Handle inbound heartbeat and optional runtime settings."""
    async for message in websocket:
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            continue

        # JSON heartbeat
        if data.get("type") == "ping":
            await websocket.send(json.dumps({"type": "pong", "ts": data.get("ts")}))
            continue

        # Optional runtime settings
        if data.get("type") == "set":
            global RATE_HZ
            if "rateHz" in data:
                try:
                    RATE_HZ = float(data["rateHz"])
                    _log(f"rateHz updated → {RATE_HZ}")
                except Exception:
                    pass
            if "invert" in data and isinstance(data["invert"], dict):
                for k in ("left", "center", "right"):
                    if k in data["invert"]:
                        INVERT[k] = bool(data["invert"][k])
                _log(f"invert updated → {INVERT}")

async def handler(websocket: WebSocketServerProtocol):
    req_path = getattr(websocket, "path", "") or ""
    if not _path_ok(req_path):
        _log(f"Rejecting path: {req_path!r} (expected {PATH})")
        await websocket.close(code=1008, reason="Invalid path")
        return

    remote = getattr(websocket, "remote_address", None)
    _log(f"[CONNECTED] client: {remote} path={req_path!r}")

    # Welcome so UI can flip to Connected immediately
    welcome = {
        "status": "connected",
        "service": "line-tracker",
        "message": "Line Tracker WebSocket connection established",
        "pins": SENSOR_PINS,
        "invert": INVERT,
        "rateHz": RATE_HZ,
        "timestamp": time.time(),
    }
    try:
        await websocket.send(json.dumps(welcome))
    except Exception as e:
        _log(f"Send welcome failed: {e!r}")
        await websocket.close()
        return

    prod = asyncio.create_task(producer(websocket))
    cons = asyncio.create_task(consumer(websocket))
    try:
        await asyncio.gather(prod, cons)
    except asyncio.CancelledError:
        pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        for t in (prod, cons):
            if not t.done():
                t.cancel()
        _log(f"[DISCONNECTED] client: {remote}")

async def main():
    _try_import_gpio()
    _setup_gpio()
    _log(f"Starting Line Tracker WebSocket Server on ws://{HOST}:{PORT}{PATH}")
    async with websockets.serve(
        handler, HOST, PORT, ping_interval=20, ping_timeout=10
    ):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        _cleanup_gpio()
