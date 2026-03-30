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

import time
import logging
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

log = logging.getLogger(__name__)
router = APIRouter()

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
    log.info('ws_movement: client connected, bridge active=%s',
             bridge.is_active if bridge else False)

    await websocket.send_json(_WELCOME)

    try:
        while True:
            data = await websocket.receive_json()

            # Heartbeat
            if data.get('type') == 'ping':
                await websocket.send_json({'type': 'pong', 'ts': data.get('ts')})
                continue

            cmd = str(data.get('command', '')).strip()

            # Status query — return a minimal snapshot the UI can parse
            if cmd == 'status':
                await websocket.send_json({
                    'type': 'status',
                    'speed': 0,
                    'movementV2': _BASE_MOVEMENT_V2,
                    'ts': int(time.time() * 1000),
                })
                continue

            # Raw twist from gamepad (proportional velocity)
            if cmd == 'twist':
                linear_x  = max(-1.0, min(1.0, float(data.get('linear_x',  0.0))))
                angular_z = max(-1.0, min(1.0, float(data.get('angular_z', 0.0))))
                ros_sent = bool(bridge and bridge.send_twist(linear_x, angular_z))
                await websocket.send_json({
                    'type': 'ack',
                    'action': 'twist',
                    'status': 'ok',
                    'ros_sent': ros_sent,
                    'ts': int(time.time() * 1000),
                })
                continue

            # ── Buzzer commands ──────────────────────────────────────────
            if cmd in _BUZZ_CMDS:
                action = _BUZZ_CMDS[cmd]
                kwargs: dict = {}
                if action == 'for':
                    kwargs['durationMs'] = int(data.get('durationMs', 500))
                elif action == 'pulse':
                    kwargs['onMs']   = int(data.get('onMs',   150))
                    kwargs['offMs']  = int(data.get('offMs',  120))
                    kwargs['repeat'] = int(data.get('repeat',   3))
                ros_sent = bool(bridge and bridge.send_buzzer_cmd(action, **kwargs))
                await websocket.send_json({
                    'type': 'ack', 'action': cmd, 'status': 'ok',
                    'ros_sent': ros_sent, 'ts': int(time.time() * 1000),
                })
                continue

            # ── Servo commands ───────────────────────────────────────────
            if cmd in _SERVO_CMDS:
                ros_sent = False
                if cmd == 'set-servo-position':
                    # { command, horizontal: 0-180, vertical: 0-180 }
                    for ch in ('horizontal', 'vertical'):
                        if ch in data:
                            angle = max(0, min(180, int(data[ch])))
                            ros_sent = bool(bridge and bridge.send_servo_cmd(ch, angle))
                else:
                    channel = _SERVO_CMDS[cmd]
                    angle = max(0, min(180, int(data.get('angle', 90))))
                    ros_sent = bool(bridge and bridge.send_servo_cmd(channel, angle))
                await websocket.send_json({
                    'type': 'ack', 'action': cmd, 'status': 'ok',
                    'ros_sent': ros_sent, 'ts': int(time.time() * 1000),
                })
                continue

            # ── Discrete movement / stop commands (buttons, keyboard) ────
            speed = max(0.0, min(1.0, float(data.get('speed', 0.5))))
            bridge_cmd = _UI_TO_BRIDGE.get(cmd)
            if bridge_cmd is not None:
                ros_sent = bool(bridge and bridge.send_command(bridge_cmd, speed))
                await websocket.send_json({
                    'type': 'ack',
                    'action': cmd,
                    'status': 'ok',
                    'ros_sent': ros_sent,
                    'ts': int(time.time() * 1000),
                })
            else:
                log.debug('ws_movement: unknown command %r', cmd)
                await websocket.send_json({
                    'type': 'ack',
                    'action': cmd,
                    'status': 'error',
                    'error': f'unknown command: {cmd}',
                })

    except WebSocketDisconnect:
        log.info('ws_movement: client disconnected — publishing stop')
        if bridge:
            bridge.stop()
    except Exception as exc:
        log.warning('ws_movement: unexpected error: %s', exc, exc_info=True)
        if bridge:
            bridge.stop()
