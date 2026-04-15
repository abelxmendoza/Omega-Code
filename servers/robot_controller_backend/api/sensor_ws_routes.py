"""
Sensor & Lighting WebSocket endpoints
=======================================
Exposes the three remaining service WebSocket endpoints the GUI Header
health-checks, all on the same FastAPI server (port 8000):

  /ws/ultrasonic  -- streams /omega/ultrasonic ROS topic; any message = alive
  /ws/line        -- streams /omega/line_tracking/state; ping/pong latency
  /ws/lighting    -- accepts lighting commands; publishes via ros_bridge; ping/pong

Protocol (matches what useWsStatus in the Next.js UI expects):
  Server → Client on connect:  { type: 'welcome', service: '<name>', status: 'connected' }
  Client → Server heartbeat:   { type: 'ping', ts: <epoch_ms> }
  Server → Client heartbeat:   { type: 'pong', ts: <echo> }
  Server → Client sensor data: { type: 'ultrasonic'|'line_tracking', ... }
  Client → Server command:     { pattern: '<pattern>', ... }   (lighting only)
  Server → Client ack:         { type: 'ack', status: 'ok'|'error', ... }

Thread model:
  OmegaSensorBridge (daemon thread) puts data into asyncio.Queue.
  Each WS handler races asyncio.wait([receive_task, queue_task]) so it
  can respond to pings immediately while also streaming sensor data.
"""

from __future__ import annotations

import asyncio
import threading
import time
import logging
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

# LED hardware (optional — degrades gracefully on non-Pi hosts)
try:
    from api.lighting_routes import led_controller as _led_controller
    from controllers.lighting.dispatcher import apply_lighting_mode as _apply_lighting_mode
    _LED_AVAILABLE = _led_controller is not None
except Exception:
    _led_controller = None
    _apply_lighting_mode = None
    _LED_AVAILABLE = False

# --- Cancellable LED pattern worker ---
# Each new command gets its own stop_event; we signal the previous one before starting.
_led_stop: threading.Event | None = None
_led_thread: threading.Thread | None = None


def _start_led_pattern(data: dict) -> None:
    """Stop any running pattern and start a new one in a daemon thread."""
    global _led_stop, _led_thread

    # Signal previous pattern to stop
    old_stop = _led_stop
    if old_stop:
        old_stop.set()

    # Give the old thread a brief window to exit (handles short sleep patterns)
    old_thread = _led_thread
    if old_thread and old_thread.is_alive():
        old_thread.join(timeout=0.15)

    # Fresh stop event for the new pattern
    stop_ev = threading.Event()
    _led_stop = stop_ev

    def _run() -> None:
        try:
            _apply_lighting_mode(data, _led_controller, stop_event=stop_ev)
        except Exception as exc:
            log.warning("LED pattern thread error: %s", exc)

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    _led_thread = t


def _stop_led_now() -> None:
    """Signal the running LED pattern to stop immediately."""
    global _led_stop
    if _led_stop:
        _led_stop.set()

log = logging.getLogger(__name__)
router = APIRouter()

# Maximum sensor frame rate sent to any WebSocket client.
# Readings that arrive faster than this are silently dropped (latest-wins).
_SEND_INTERVAL = 0.05  # 20 FPS


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

async def _ws_race(websocket: WebSocket, q: asyncio.Queue) -> tuple[dict | None, dict | None]:
    """
    Race a receive_json against a queue.get.

    Returns (received_msg, queued_data) where exactly one is non-None.
    Cancels the loser task.
    Raises WebSocketDisconnect if the receive task raises it.
    """
    recv = asyncio.create_task(websocket.receive_json())
    deq  = asyncio.create_task(q.get())

    done, pending = await asyncio.wait(
        [recv, deq],
        return_when=asyncio.FIRST_COMPLETED,
    )

    for t in pending:
        t.cancel()

    for t in done:
        if t is recv:
            return t.result(), None
        else:
            return None, t.result()

    return None, None   # unreachable


# ---------------------------------------------------------------------------
# /ws/ultrasonic
# ---------------------------------------------------------------------------

@router.websocket('/ws/ultrasonic')
async def ws_ultrasonic(websocket: WebSocket):
    """
    Streams /omega/ultrasonic (sensor_msgs/Range) to the UI.
    Header uses treatAnyMessageAsAlive=true -- every sensor push keeps the
    pill green.  Still responds to pings so latency display works too.
    """
    await websocket.accept()
    sensor_bridge = getattr(websocket.app.state, 'sensor_bridge', None)
    log.info('ws_ultrasonic: client connected, bridge active=%s',
             sensor_bridge.is_active if sensor_bridge else False)

    await websocket.send_json({
        'type': 'welcome', 'service': 'ultrasonic', 'status': 'connected',
    })

    q: asyncio.Queue = asyncio.Queue(maxsize=5)
    if sensor_bridge:
        sensor_bridge.add_ultrasonic_queue(q)

    last_send = 0.0
    try:
        while True:
            received, queued = await _ws_race(websocket, q)

            if received is not None:
                if received.get('type') == 'ping':
                    await websocket.send_json({'type': 'pong', 'ts': received.get('ts')})

            if queued is not None:
                now = time.monotonic()
                if now - last_send >= _SEND_INTERVAL:
                    await websocket.send_json(queued)
                    last_send = now
                # else: drop sample — client already has a fresh one in-flight

    except WebSocketDisconnect:
        log.info('ws_ultrasonic: client disconnected')
    except Exception as exc:
        log.debug('ws_ultrasonic: %s', exc)
    finally:
        if sensor_bridge:
            sensor_bridge.remove_ultrasonic_queue(q)


# ---------------------------------------------------------------------------
# /ws/line
# ---------------------------------------------------------------------------

@router.websocket('/ws/line')
async def ws_line(websocket: WebSocket):
    """
    Streams /omega/line_tracking/state (JSON String) to the UI.
    Responds to ping with pong for latency measurement.
    """
    await websocket.accept()
    sensor_bridge = getattr(websocket.app.state, 'sensor_bridge', None)
    log.info('ws_line: client connected, bridge active=%s',
             sensor_bridge.is_active if sensor_bridge else False)

    await websocket.send_json({
        'type': 'welcome', 'service': 'line', 'status': 'connected',
    })

    q: asyncio.Queue = asyncio.Queue(maxsize=5)
    if sensor_bridge:
        sensor_bridge.add_line_queue(q)

    last_send = 0.0
    try:
        while True:
            received, queued = await _ws_race(websocket, q)

            if received is not None:
                if received.get('type') == 'ping':
                    await websocket.send_json({'type': 'pong', 'ts': received.get('ts')})

            if queued is not None:
                now = time.monotonic()
                if now - last_send >= _SEND_INTERVAL:
                    await websocket.send_json(queued)
                    last_send = now

    except WebSocketDisconnect:
        log.info('ws_line: client disconnected')
    except Exception as exc:
        log.debug('ws_line: %s', exc)
    finally:
        if sensor_bridge:
            sensor_bridge.remove_line_queue(q)


# ---------------------------------------------------------------------------
# /ws/lighting
# ---------------------------------------------------------------------------

@router.websocket('/ws/lighting')
async def ws_lighting(websocket: WebSocket):
    """
    Accepts lighting commands from the UI and publishes them via the ROS bridge.
    Also responds to ping/pong so the Header pill shows live latency.

    Command format (UI → server):
      { pattern: 'solid' | 'pulse' | 'rainbow' | 'off' | ...,
        r: 0-255, g: 0-255, b: 0-255,    # optional colour
        brightness: 0.0-1.0 }             # optional
    """
    await websocket.accept()
    ros_bridge = getattr(websocket.app.state, 'ros_bridge', None)
    log.info('ws_lighting: client connected, bridge active=%s',
             ros_bridge.is_active if ros_bridge else False)

    await websocket.send_json({
        'type': 'welcome', 'service': 'lighting', 'status': 'connected',
    })

    try:
        while True:
            data = await websocket.receive_json()

            # Heartbeat
            if data.get('type') == 'ping':
                await websocket.send_json({'type': 'pong', 'ts': data.get('ts')})
                continue

            # Lighting command
            pattern = str(data.get('pattern') or data.get('command') or '').strip()
            if not pattern:
                await websocket.send_json({
                    'type': 'lighting_result', 'ok': False,
                    'error': 'missing pattern field',
                    'ts': int(time.time() * 1000),
                })
                continue

            led_ok = False
            led_error = None

            # Direct hardware path (Pi only)
            if _LED_AVAILABLE:
                try:
                    if pattern == 'off':
                        # Stop any running pattern, then clear the strip
                        _stop_led_now()
                        await asyncio.to_thread(_led_controller.clear_strip)
                    else:
                        # Fire-and-forget in a background thread so the WS
                        # handler can immediately process the next command.
                        _start_led_pattern(data)
                    led_ok = True
                except Exception as exc:
                    led_error = str(exc)
                    log.warning('ws_lighting: LED hardware error: %s', exc)

            # Also forward to ROS bridge (future ROS-native lighting node)
            ros_sent = False
            if ros_bridge:
                extras = {
                    k: v for k, v in data.items()
                    if k not in ('type', 'pattern', 'command')
                }
                ros_sent = ros_bridge.send_lighting_cmd(pattern, **extras)

            await websocket.send_json({
                'type': 'lighting_result',
                'ok': led_ok or ros_sent,
                'pattern': pattern,
                'ros_sent': ros_sent,
                'error': led_error,
                'ts': int(time.time() * 1000),
            })

    except WebSocketDisconnect:
        log.info('ws_lighting: client disconnected')
    except Exception as exc:
        log.debug('ws_lighting: %s', exc)


# ---------------------------------------------------------------------------
# /ws/radar  — ultrasonic radar sweep (servo pan + HC-SR04 distance)
# ---------------------------------------------------------------------------

# Hardware classes loaded lazily so the server still starts on non-Pi hosts.
_radar_servo_cls = None   # controllers.servo_control.Servo
_radar_sensor_cls = None  # sensors.ultrasonic_sensor.Ultrasonic


def _try_load_radar_hw() -> bool:
    """Return True if both Servo and Ultrasonic classes are importable."""
    global _radar_servo_cls, _radar_sensor_cls
    if _radar_servo_cls is None:
        try:
            from controllers.servo_control import Servo
            _radar_servo_cls = Servo
        except Exception:
            pass
    if _radar_sensor_cls is None:
        try:
            from sensors.ultrasonic_sensor import Ultrasonic
            _radar_sensor_cls = Ultrasonic
        except Exception:
            pass
    return _radar_servo_cls is not None and _radar_sensor_cls is not None


async def _do_radar_sweep(
    websocket: WebSocket,
    step_deg: int,
    dwell_ms: float,
    sweep_min: int,
    sweep_max: int,
    stop_ev: asyncio.Event,
    servo,
    sensor,
) -> None:
    """
    Sweep the pan servo back and forth between sweep_min and sweep_max,
    reading ultrasonic distance at each step and streaming radar_scan frames.
    Exits when stop_ev is set or the WebSocket closes.
    """
    angle = sweep_min
    direction = 1

    while not stop_ev.is_set():
        # Position servo (blocking I²C writes, offload to thread pool)
        try:
            await asyncio.to_thread(servo.setServoPwm, '0', angle)
        except Exception as exc:
            log.warning('radar: servo error at %d°: %s', angle, exc)

        # Dwell — servo settle time; exit early if stopped
        try:
            await asyncio.wait_for(stop_ev.wait(), timeout=dwell_ms / 1000.0)
            break  # stop_ev fired during dwell
        except asyncio.TimeoutError:
            pass

        if stop_ev.is_set():
            break

        # Read ultrasonic distance (blocking)
        try:
            dist = await asyncio.to_thread(sensor.get_distance)
        except Exception:
            dist = -1

        # Stream frame to UI
        frame = {
            'type': 'radar_scan',
            'angle_deg': angle,
            'distance_cm': dist if dist >= 2 else None,
            'ts': int(time.time() * 1000),
        }
        try:
            await websocket.send_json(frame)
        except Exception:
            return  # client gone

        # Advance angle (bounce off limits)
        angle += direction * step_deg
        if angle >= sweep_max:
            angle = sweep_max
            direction = -1
        elif angle <= sweep_min:
            angle = sweep_min
            direction = 1

    # Return servo to center
    try:
        await asyncio.to_thread(servo.setServoPwm, '0', 90)
    except Exception:
        pass


@router.websocket('/ws/radar')
async def ws_radar(websocket: WebSocket):
    """
    Radar sweep control WebSocket.

    Client → Server:
      { command: 'start-sweep', step_deg?: int, dwell_ms?: int,
        sweep_min?: int, sweep_max?: int }
      { command: 'stop-sweep' }
      { type: 'ping', ts: int }

    Server → Client:
      { type: 'welcome', service: 'radar', status: 'connected', hardware: bool }
      { type: 'radar_scan', angle_deg: int, distance_cm: float|null, ts: int }
      { type: 'sweep_status', sweeping: bool, error?: str }
      { type: 'pong', ts: int }
    """
    await websocket.accept()
    log.info('ws_radar: client connected')

    hw = _try_load_radar_hw()
    await websocket.send_json({
        'type': 'welcome',
        'service': 'radar',
        'status': 'connected',
        'hardware': hw,
    })

    servo = None
    sensor = None
    sweep_task: asyncio.Task | None = None
    stop_ev = asyncio.Event()

    try:
        while True:
            data = await websocket.receive_json()

            if data.get('type') == 'ping':
                await websocket.send_json({'type': 'pong', 'ts': data.get('ts')})
                continue

            cmd = data.get('command', '')

            if cmd == 'start-sweep':
                # Cancel any running sweep cleanly
                if sweep_task and not sweep_task.done():
                    stop_ev.set()
                    try:
                        await asyncio.wait_for(asyncio.shield(sweep_task), timeout=0.6)
                    except (asyncio.TimeoutError, asyncio.CancelledError):
                        sweep_task.cancel()

                if not hw:
                    await websocket.send_json({
                        'type': 'sweep_status',
                        'sweeping': False,
                        'error': 'Hardware unavailable — run on Pi with servo + ultrasonic connected',
                    })
                    continue

                # Lazy-init hardware (blocking constructors in thread pool)
                try:
                    if servo is None:
                        servo = await asyncio.to_thread(_radar_servo_cls)
                    if sensor is None:
                        sensor = await asyncio.to_thread(_radar_sensor_cls)
                except Exception as exc:
                    await websocket.send_json({
                        'type': 'sweep_status',
                        'sweeping': False,
                        'error': f'Hardware init failed: {exc}',
                    })
                    continue

                step_deg  = max(1,   min(30,   int(data.get('step_deg',  5))))
                dwell_ms  = max(50,  min(1000, int(data.get('dwell_ms', 150))))
                sweep_min = max(30,  min(89,   int(data.get('sweep_min', 30))))
                sweep_max = max(91,  min(150,  int(data.get('sweep_max', 150))))

                stop_ev = asyncio.Event()
                sweep_task = asyncio.create_task(_do_radar_sweep(
                    websocket, step_deg, dwell_ms, sweep_min, sweep_max,
                    stop_ev, servo, sensor,
                ))
                await websocket.send_json({'type': 'sweep_status', 'sweeping': True})

            elif cmd == 'stop-sweep':
                if sweep_task and not sweep_task.done():
                    stop_ev.set()
                await websocket.send_json({'type': 'sweep_status', 'sweeping': False})

    except WebSocketDisconnect:
        log.info('ws_radar: client disconnected')
    except Exception as exc:
        log.debug('ws_radar: %s', exc)
    finally:
        stop_ev.set()
        if sweep_task and not sweep_task.done():
            sweep_task.cancel()
        # Return servo to center on disconnect
        if servo:
            try:
                await asyncio.to_thread(servo.setServoPwm, '0', 90)
            except Exception:
                pass
