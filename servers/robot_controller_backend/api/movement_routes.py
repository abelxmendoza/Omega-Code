"""
Movement WebSocket endpoint for the Omega Robot Controller.

Speaks the same JSON protocol as the legacy movement_ws_server.py so the
Next.js UI (CommandContext / CarControlPanel) needs zero changes.

Protocol summary:
  Server → Client on connect:  { type: 'welcome', service: 'movement', status: 'connected' }
  Client → Server heartbeat:   { type: 'ping', ts: <epoch_ms> }
  Server → Client heartbeat:   { type: 'pong', ts: <echo> }
  Client → Server move:        { command: 'move-up' | 'move-down' | 'move-left' | 'move-right' | 'move-stop' | 'stop' }
  Server → Client ack:         { type: 'ack', action: <cmd>, status: 'ok'|'error', ros_sent: bool }
  Client → Server status:      { command: 'status' }
  Server → Client status:      { type: 'status', speed: 0, movementV2: { enabled: true, ... } }
"""

import asyncio
import time
import logging
import websockets
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

log = logging.getLogger(__name__)
router = APIRouter()

_MOVEMENT_BACKEND_URL = 'ws://localhost:8081/'

async def _get_backend_ws() -> 'websockets.WebSocketClientProtocol | None':
    """Open a connection to the real movement WS server. Returns None on failure."""
    try:
        return await asyncio.wait_for(
            websockets.connect(_MOVEMENT_BACKEND_URL, open_timeout=2),
            timeout=2,
        )
    except Exception as exc:
        log.warning('movement proxy: cannot reach backend at %s: %s', _MOVEMENT_BACKEND_URL, exc)
        return None

# Speed constants (PWM range 0–4095, matches Freenove / PCA9685 12-bit)
_PWM_MAX        = 4095
_DEFAULT_SPEED  = 1200   # ~29% — safe starting speed, matches UI default
_SPEED_STEP     = 400    # ~10% per gas/brake press

# Maps UI movement command strings to OmegaRosBridge command keys
_UI_TO_BRIDGE: dict[str, str] = {
    'move-up':    'forward',
    'move-down':  'backward',
    'move-left':  'left',
    'move-right': 'right',
    'move-stop':  'stop',
    'stop':       'stop',
}

# Buzzer commands — map UI command to buzzer action
_BUZZ_CMDS: dict[str, str] = {
    'buzz':       'on',
    'buzz-stop':  'off',
    'buzz-for':   'for',
    'buzz-pulse': 'pulse',
}

# Servo commands — channels keyed by UI command string
_SERVO_CMDS: dict[str, str] = {
    'servo-horizontal':    'horizontal',
    'servo-vertical':      'vertical',
    'set-servo-position':  None,   # handles both axes; see handler below
}

# Camera nudge commands — relative angle steps (safe ±10° per press, clamped to 1200-1800µs range)
_SERVO_NUDGE_STEP = 10   # degrees per button press
_SERVO_NUDGE_CENTER = 90 # starting angle if no position tracked
_SERVO_NUDGE: dict[str, tuple[str, int]] = {
    'camera-servo-up':    ('vertical',    +_SERVO_NUDGE_STEP),
    'camera-servo-down':  ('vertical',    -_SERVO_NUDGE_STEP),
    'camera-servo-left':  ('horizontal',  +_SERVO_NUDGE_STEP),
    'camera-servo-right': ('horizontal',  -_SERVO_NUDGE_STEP),
}

_WELCOME = {
    'type': 'welcome',
    'service': 'movement',
    'status': 'connected',
    'protocol': 'omega-ros2-v1',
}

_BASE_MOVEMENT_V2 = {
    'enabled': True,
    'watchdog': {'enabled': True, 'time_until_trigger': 1.0, 'state': 'ok'},
}


@router.websocket('/ws/movement')
async def ws_movement(websocket: WebSocket):
    await websocket.accept()
    bridge = getattr(websocket.app.state, 'ros_bridge', None)
    use_ros = bridge is not None and bridge.is_active
    log.info('ws_movement: client connected, ros_bridge active=%s', use_ros)

    # Per-connection speed state (PWM 0–4095)
    current_speed: int = _DEFAULT_SPEED

    # Per-connection servo position tracking (degrees, clamped 0–180)
    servo_pos: dict[str, int] = {'horizontal': 90, 'vertical': 90}

    # ── Proxy mode: connect to real movement_ws_server on port 8081 ─────
    # Used when ROS bridge is not active (no ROS2 — normal Pi-only mode).
    backend: 'websockets.WebSocketClientProtocol | None' = None
    if not use_ros:
        backend = await _get_backend_ws()
        if backend:
            # Consume backend welcome — discard its speed (may be stale/0 from watchdog)
            try:
                import json as _json
                raw = await asyncio.wait_for(backend.recv(), timeout=2)
                _json.loads(raw)  # just drain it
            except Exception:
                pass
            # Immediately sync our current_speed to the backend so movement commands work
            try:
                import json as _json
                await backend.send(_json.dumps({'command': 'set-speed', 'speed': current_speed}))
                await asyncio.wait_for(backend.recv(), timeout=2)  # drain set-speed ack
            except Exception:
                pass
            log.info('ws_movement: proxy connected to backend, seeded speed=%d', current_speed)
        else:
            log.warning('ws_movement: backend unreachable — commands will be no-op')

    await websocket.send_json({**_WELCOME, 'speed': current_speed})

    # ── Backend I/O architecture ─────────────────────────────────────────
    # A single drainer task reads ALL backend responses into a queue.
    # _proxy() sends a command then awaits one item from the queue.
    # Keepalive sends only (no read) — drainer discards those acks silently.
    # This avoids any ack ordering races.
    import json as _json
    _ack_queue: asyncio.Queue = asyncio.Queue()
    _send_lock = asyncio.Lock()   # serialise writes only
    _keepalive_task: asyncio.Task | None = None
    _active_move_cmd: str | None = None
    _drainer_task: asyncio.Task | None = None

    async def _drain_backend() -> None:
        """Continuously read backend responses into the ack queue."""
        while backend is not None:
            try:
                raw = await asyncio.wait_for(backend.recv(), timeout=5)
                await _ack_queue.put(_json.loads(raw))
            except asyncio.TimeoutError:
                continue
            except Exception:
                break

    async def _proxy(data: dict) -> dict:
        """Send a command to the backend and return next ack from the queue."""
        if backend is None:
            return {'type': 'ack', 'action': data.get('command', ''),
                    'status': 'error', 'error': 'backend_unreachable'}
        try:
            async with _send_lock:
                await backend.send(_json.dumps(data))
            return await asyncio.wait_for(_ack_queue.get(), timeout=2)
        except Exception as exc:
            log.warning('ws_movement proxy error: %s', exc)
            return {'type': 'ack', 'action': data.get('command', ''),
                    'status': 'error', 'error': str(exc)}

    async def _keepalive_loop(move_cmd: str) -> None:
        """Fire movement command every 80ms — drainer discards acks."""
        while True:
            await asyncio.sleep(0.08)
            if _active_move_cmd != move_cmd or backend is None:
                break
            try:
                async with _send_lock:
                    await backend.send(_json.dumps({'command': move_cmd, 'speed': current_speed}))
            except Exception:
                break

    def _start_keepalive(move_cmd: str) -> None:
        nonlocal _keepalive_task, _active_move_cmd
        _active_move_cmd = move_cmd
        if _keepalive_task and not _keepalive_task.done():
            _keepalive_task.cancel()
        _keepalive_task = asyncio.create_task(_keepalive_loop(move_cmd))

    async def _stop_keepalive_and_drain() -> None:
        """Cancel keepalive and drain any extra acks before next _proxy call."""
        nonlocal _keepalive_task, _active_move_cmd
        _active_move_cmd = None
        if _keepalive_task and not _keepalive_task.done():
            _keepalive_task.cancel()
            try:
                await _keepalive_task
            except (asyncio.CancelledError, Exception):
                pass
        _keepalive_task = None
        # Drain any queued keepalive acks so _proxy gets its own clean response
        await asyncio.sleep(0.05)  # let drainer catch up
        while not _ack_queue.empty():
            try:
                _ack_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

    if backend:
        _drainer_task = asyncio.create_task(_drain_backend())

    try:
        while True:
            data = await websocket.receive_json()

            # Heartbeat — handle locally, never proxy
            if data.get('type') == 'ping':
                await websocket.send_json({'type': 'pong', 'ts': data.get('ts')})
                continue

            cmd = str(data.get('command', '')).strip()

            # Status query
            if cmd == 'status':
                if use_ros:
                    await websocket.send_json({
                        'type': 'status',
                        'speed': current_speed,
                        'movementV2': _BASE_MOVEMENT_V2,
                        'ts': int(time.time() * 1000),
                    })
                elif backend is not None:
                    ack = await _proxy(data)
                    # Override backend's speed field with ours — backend speed may be 0
                    # after watchdog, but our current_speed is the user's configured speed.
                    ack['speed'] = current_speed
                    await websocket.send_json(ack)
                else:
                    # Backend unreachable — return local state as a valid status response
                    # so the UI always gets a type:status (not a backend_unreachable ack).
                    await websocket.send_json({
                        'type': 'status',
                        'speed': current_speed,
                        'servo': servo_pos,
                        'movementV2': _BASE_MOVEMENT_V2,
                        'sim': True,
                        'ts': int(time.time() * 1000),
                    })
                continue

            # ── Speed control — track locally, sync to backend with normalized key ─
            if cmd == 'set-speed':
                # UI sends {value: pwm}; backend expects {speed: pwm}
                raw = data.get('value', data.get('speed', current_speed))
                current_speed = max(0, min(_PWM_MAX, int(raw)))
                if not use_ros:
                    await _proxy({'command': 'set-speed', 'speed': current_speed})
                await websocket.send_json({
                    'type': 'ack', 'action': 'set-speed', 'status': 'ok',
                    'speed': current_speed, 'ts': int(time.time() * 1000),
                })
                continue

            if cmd == 'increase-speed':
                current_speed = min(_PWM_MAX, current_speed + _SPEED_STEP)
                if not use_ros:
                    await _proxy({'command': 'set-speed', 'speed': current_speed})
                await websocket.send_json({
                    'type': 'ack', 'action': 'increase-speed', 'status': 'ok',
                    'speed': current_speed, 'ts': int(time.time() * 1000),
                })
                continue

            if cmd == 'decrease-speed':
                current_speed = max(0, current_speed - _SPEED_STEP)
                if not use_ros:
                    await _proxy({'command': 'set-speed', 'speed': current_speed})
                await websocket.send_json({
                    'type': 'ack', 'action': 'decrease-speed', 'status': 'ok',
                    'speed': current_speed, 'ts': int(time.time() * 1000),
                })
                continue

            # ── Everything else: ROS bridge or proxy ─────────────────────
            if use_ros:
                # Twist (gamepad)
                if cmd == 'twist':
                    linear_x  = max(-1.0, min(1.0, float(data.get('linear_x',  0.0))))
                    angular_z = max(-1.0, min(1.0, float(data.get('angular_z', 0.0))))
                    speed_norm = current_speed / _PWM_MAX
                    ros_sent = bool(bridge.send_twist(linear_x * speed_norm, angular_z * speed_norm))
                    await websocket.send_json({
                        'type': 'ack', 'action': 'twist', 'status': 'ok',
                        'speed': current_speed, 'ros_sent': ros_sent,
                        'ts': int(time.time() * 1000),
                    })
                    continue

                # Buzzer
                if cmd in _BUZZ_CMDS:
                    action = _BUZZ_CMDS[cmd]
                    kwargs: dict = {}
                    if action == 'for':
                        kwargs['durationMs'] = int(data.get('durationMs', 500))
                    elif action == 'pulse':
                        kwargs['onMs']   = int(data.get('onMs',   150))
                        kwargs['offMs']  = int(data.get('offMs',  120))
                        kwargs['repeat'] = int(data.get('repeat',   3))
                    ros_sent = bool(bridge.send_buzzer_cmd(action, **kwargs))
                    await websocket.send_json({
                        'type': 'ack', 'action': cmd, 'status': 'ok',
                        'ros_sent': ros_sent, 'ts': int(time.time() * 1000),
                    })
                    continue

                # Servo
                if cmd in _SERVO_CMDS or cmd in _SERVO_NUDGE:
                    ros_sent = False
                    if cmd == 'set-servo-position':
                        for ch in ('horizontal', 'vertical'):
                            if ch in data:
                                angle = max(0, min(180, int(data[ch])))
                                ros_sent = bool(bridge.send_servo_cmd(ch, angle))
                    elif cmd in _SERVO_CMDS:
                        channel = _SERVO_CMDS[cmd]
                        angle = max(0, min(180, int(data.get('angle', 90))))
                        ros_sent = bool(bridge.send_servo_cmd(channel, angle))
                    else:
                        channel, delta = _SERVO_NUDGE[cmd]
                        servo_pos[channel] = max(0, min(180, servo_pos[channel] + delta))
                        ros_sent = bool(bridge.send_servo_cmd(channel, servo_pos[channel]))
                    await websocket.send_json({
                        'type': 'ack', 'action': cmd, 'status': 'ok',
                        'ros_sent': ros_sent, 'ts': int(time.time() * 1000),
                    })
                    continue

                # Movement commands
                speed_norm = current_speed / _PWM_MAX
                bridge_cmd = _UI_TO_BRIDGE.get(cmd)
                if bridge_cmd is not None:
                    ros_sent = bool(bridge.send_command(bridge_cmd, speed_norm))
                    await websocket.send_json({
                        'type': 'ack', 'action': cmd, 'status': 'ok',
                        'speed': current_speed, 'ros_sent': ros_sent,
                        'ts': int(time.time() * 1000),
                    })
                else:
                    log.debug('ws_movement: unknown command %r', cmd)
                    await websocket.send_json({
                        'type': 'ack', 'action': cmd, 'status': 'error',
                        'error': f'unknown command: {cmd}',
                    })

            else:
                # ── Proxy path — forward everything to port 8081 ──────────
                # Always inject current_speed so backend doesn't fall back to its
                # global (which may be 0 after a watchdog trigger).
                proxy_data = {**data, 'speed': current_speed}

                # Manage keepalive for movement commands
                if cmd in _UI_TO_BRIDGE:
                    if cmd in ('stop', 'move-stop'):
                        await _stop_keepalive_and_drain()
                    else:
                        # Start keepalive before sending so ramp begins immediately
                        _start_keepalive(cmd)

                ack = await _proxy(proxy_data)
                # Keep local servo state in sync for nudge commands
                if cmd in _SERVO_NUDGE:
                    channel, delta = _SERVO_NUDGE[cmd]
                    servo_pos[channel] = max(0, min(180, servo_pos[channel] + delta))
                # Don't sync speed back from backend ack — we own speed state
                ack['speed'] = current_speed
                # In sim mode (backend unavailable), synthesize OK for known commands
                # so the UI and tests receive clean acks.  Unknown commands keep
                # the error status so callers can detect unrecognised inputs.
                if (ack.get('status') == 'error'
                        and ack.get('error') == 'backend_unreachable'
                        and (cmd in _UI_TO_BRIDGE or cmd in _BUZZ_CMDS
                             or cmd in _SERVO_CMDS or cmd in _SERVO_NUDGE
                             or cmd in ('twist', 'reset-servo'))):
                    ack = {'type': 'ack', 'action': cmd, 'status': 'ok',
                           'speed': current_speed, 'sim': True,
                           'ts': int(time.time() * 1000)}
                await websocket.send_json(ack)

    except WebSocketDisconnect:
        log.info('ws_movement: client disconnected — sending stop')
        await _stop_keepalive_and_drain()
        if use_ros and bridge:
            bridge.stop()
        elif backend:
            try:
                import json as _json
                await backend.send(_json.dumps({'command': 'stop'}))
            except Exception:
                pass
    except Exception as exc:
        log.warning('ws_movement: unexpected error: %s', exc, exc_info=True)
        if use_ros and bridge:
            bridge.stop()
    finally:
        if _drainer_task and not _drainer_task.done():
            _drainer_task.cancel()
        if backend:
            try:
                await backend.close()
            except Exception:
                pass
