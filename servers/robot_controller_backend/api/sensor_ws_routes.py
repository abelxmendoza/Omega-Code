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
import time
import logging
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

log = logging.getLogger(__name__)
router = APIRouter()


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

    q: asyncio.Queue = asyncio.Queue(maxsize=20)
    if sensor_bridge:
        sensor_bridge.add_ultrasonic_queue(q)

    try:
        while True:
            received, queued = await _ws_race(websocket, q)

            if received is not None:
                if received.get('type') == 'ping':
                    await websocket.send_json({'type': 'pong', 'ts': received.get('ts')})

            if queued is not None:
                await websocket.send_json(queued)

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

    q: asyncio.Queue = asyncio.Queue(maxsize=20)
    if sensor_bridge:
        sensor_bridge.add_line_queue(q)

    try:
        while True:
            received, queued = await _ws_race(websocket, q)

            if received is not None:
                if received.get('type') == 'ping':
                    await websocket.send_json({'type': 'pong', 'ts': received.get('ts')})

            if queued is not None:
                await websocket.send_json(queued)

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
                    'type': 'ack', 'status': 'error',
                    'error': 'missing pattern field',
                    'ts': int(time.time() * 1000),
                })
                continue

            ros_sent = False
            if ros_bridge:
                extras = {
                    k: v for k, v in data.items()
                    if k not in ('type', 'pattern', 'command')
                }
                ros_sent = ros_bridge.send_lighting_cmd(pattern, **extras)

            await websocket.send_json({
                'type': 'ack',
                'pattern': pattern,
                'status': 'ok',
                'ros_sent': ros_sent,
                'ts': int(time.time() * 1000),
            })

    except WebSocketDisconnect:
        log.info('ws_lighting: client disconnected')
    except Exception as exc:
        log.debug('ws_lighting: %s', exc)
