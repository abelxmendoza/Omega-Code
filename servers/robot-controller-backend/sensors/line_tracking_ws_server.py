# File: /Omega-Code/servers/robot-controller-backend/sensors/line_tracking_ws_server.py
# Summary:
# WebSocket server for streaming line-tracking sensor states (3x IR) to the UI.
#
# Updates in this version:
# - Sends a "connected" welcome envelope so the UI can flip status immediately.
# - Supports JSON heartbeat: {"type":"ping","ts"} -> {"type":"pong","ts"}.
# - Streams at RATE_HZ (default 10 Hz), change-only with periodic keepalives.
# - Path-aware: serves on ws://<host>:<port>/line-tracker.
# - GPIO backend prefers lgpio (Pi 5 compatible), falls back to RPi.GPIO; simulates on non-Pi.
# - Pins + inversion + rate configurable via env vars, rate changes apply live.
#
# Env (examples):
#   LINE_TRACKER_HOST=0.0.0.0
#   LINE_TRACKER_PORT=8090
#   LINE_TRACKER_PATH=/line-tracker
#   PIN_LEFT=14 PIN_CENTER=15 PIN_RIGHT=23
#   INVERT_LEFT=1 INVERT_CENTER=1 INVERT_RIGHT=1
#   RATE_HZ=15 KEEPALIVE_SEC=2.0
#
# Test:
#   wscat -c ws://<pi-ip>:8090/line-tracker
#   > {"type":"ping","ts":123}
#   < {"type":"pong","ts":123}

import os
import asyncio
import json
import time

# ---------- GPIO backends ----------
ON_PI = False
USE_LGPIO = False
USE_RPIGPIO = False

try:
    import lgpio  # works on Pi 5
    USE_LGPIO = True
    ON_PI = True
except Exception:
    try:
        import RPi.GPIO as RGPIO  # common on Pi 3/4
        USE_RPIGPIO = True
        ON_PI = True
    except Exception:
        ON_PI = False

import websockets
from websockets.server import WebSocketServerProtocol

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
PATH = os.getenv("LINE_TRACKER_PATH", "/line-tracker")

# ---------- GPIO setup / read ----------
_lgpio_h = None
def _setup_gpio():
    global _lgpio_h
    if USE_LGPIO:
        _lgpio_h = lgpio.gpiochip_open(0)
        for pin in SENSOR_PINS.values():
            lgpio.gpio_claim_input(_lgpio_h, pin)
    elif USE_RPIGPIO:
        RGPIO.setwarnings(False)
        RGPIO.setmode(RGPIO.BCM)
        for pin in SENSOR_PINS.values():
            RGPIO.setup(pin, RGPIO.IN)

def _cleanup_gpio():
    global _lgpio_h
    try:
        if USE_LGPIO and _lgpio_h is not None:
            lgpio.gpiochip_close(_lgpio_h)
            _lgpio_h = None
        elif USE_RPIGPIO:
            RGPIO.cleanup()
    except Exception:
        pass

def _gpio_read(pin: int) -> int:
    if USE_LGPIO and _lgpio_h is not None:
        return 1 if lgpio.gpio_read(_lgpio_h, pin) else 0
    if USE_RPIGPIO:
        return int(RGPIO.input(pin))
    # Simulation for local dev
    t = int(time.time() * 2)  # 2 Hz flip
    return 1 if (pin + t) % 2 == 0 else 0

def read_sensors() -> dict:
    raw = {name: _gpio_read(pin) for name, pin in SENSOR_PINS.items()}
    # Apply inversion (some IR boards output 0 on black line)
    inv = {k: (1 - v) if INVERT[k] else v for k, v in raw.items()}
    return inv

# ---------- WS tasks ----------
async def producer(websocket: WebSocketServerProtocol):
    """Send samples at RATE_HZ, change-only, with periodic keepalives."""
    last_state = None
    last_sent = 0.0

    while True:
        # Recompute period each loop so runtime rate changes apply immediately
        period = 1.0 / max(RATE_HZ, 0.1)

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
                # "sensors": state,  # optional alias
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
                except Exception:
                    pass
            if "invert" in data and isinstance(data["invert"], dict):
                for k in ("left", "center", "right"):
                    if k in data["invert"]:
                        INVERT[k] = bool(data["invert"][k])

async def handler(websocket: WebSocketServerProtocol):
    # Path check (works with websockets>=10 via websocket.path)
    if getattr(websocket, "path", PATH) != PATH:
        await websocket.close(code=1008, reason="Invalid path")
        return

    remote = getattr(websocket, "remote_address", None)
    print(f"[CONNECTED] LineTracker client: {remote}")

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
    except Exception:
        # If client closed instantly, bail out
        await websocket.close()
        return

    prod = asyncio.create_task(producer(websocket))
    cons = asyncio.create_task(consumer(websocket))
    try:
        await asyncio.gather(prod, cons)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        for t in (prod, cons):
            if not t.done():
                t.cancel()
        print(f"[DISCONNECTED] LineTracker client: {remote}")

async def main():
    _setup_gpio()
    print(f"Starting Line Tracker WebSocket Server on ws://{HOST}:{PORT}{PATH}")
    # Protocol ping/pong (transport-level) in addition to JSON heartbeat
    async with websockets.serve(handler, HOST, PORT, ping_interval=20, ping_timeout=10):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        _cleanup_gpio()
