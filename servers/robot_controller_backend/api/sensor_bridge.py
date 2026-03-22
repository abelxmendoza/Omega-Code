"""
Omega Sensor Bridge
====================
Subscribes to ROS2 sensor topics and fans data out to asyncio queues
that WebSocket handlers drain.

Topics subscribed:
  /omega/ultrasonic           sensor_msgs/Range      (~10 Hz)
  /omega/line_tracking/state  std_msgs/String (JSON) (~20 Hz)

Thread model:
  rclpy spin runs in a daemon thread (SingleThreadedExecutor).
  Callbacks run in that thread and push data into asyncio queues via
  loop.call_soon_threadsafe() so the FastAPI event loop is never blocked.

Usage (in main_api.py lifespan):
  loop = asyncio.get_event_loop()
  bridge = OmegaSensorBridge.create(loop)
  app.state.sensor_bridge = bridge
  ...
  bridge.shutdown()

Usage (in a WebSocket handler):
  q: asyncio.Queue = asyncio.Queue(maxsize=10)
  sensor_bridge.add_ultrasonic_queue(q)
  try:
      data = await q.get()
      await websocket.send_json(data)
  finally:
      sensor_bridge.remove_ultrasonic_queue(q)
"""

from __future__ import annotations

import asyncio
import json
import math
import threading
import time
import logging
from typing import Set, Optional

log = logging.getLogger(__name__)

_rclpy_ok = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (
        QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
    )
    from sensor_msgs.msg import Range
    from std_msgs.msg import String
    _rclpy_ok = True
except ImportError as _ie:
    log.info('rclpy not available (%s) -- sensor bridge will be a no-op', _ie)


def _best_effort_qos(depth: int = 5) -> 'QoSProfile':
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class OmegaSensorBridge:
    """
    Thread-safe fan-out from ROS2 sensor topics to asyncio WebSocket queues.
    All public methods are safe to call from asyncio context.
    """

    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        node: Optional['Node'],
        executor_thread: Optional[threading.Thread],
    ) -> None:
        self._loop = loop
        self._node = node
        self._thread = executor_thread
        self._enabled = node is not None

        self._lock = threading.Lock()
        self._us_queues: Set[asyncio.Queue] = set()
        self._line_queues: Set[asyncio.Queue] = set()

    # ------------------------------------------------------------------
    # Factory
    # ------------------------------------------------------------------

    @classmethod
    def create(cls, loop: asyncio.AbstractEventLoop) -> 'OmegaSensorBridge':
        """
        Create and start the sensor bridge.
        Returns a no-op instance if rclpy is unavailable.
        """
        if not _rclpy_ok:
            log.info('Sensor bridge: rclpy absent -- running as no-op')
            return cls(loop, None, None)

        try:
            if not rclpy.ok():
                rclpy.init()

            node = rclpy.create_node('omega_sensor_ws_bridge')
            bridge = cls(loop, node, None)

            qos = _best_effort_qos()
            node.create_subscription(
                Range,
                '/omega/ultrasonic',
                bridge._on_ultrasonic,
                qos,
            )
            node.create_subscription(
                String,
                '/omega/line_tracking/state',
                bridge._on_line_state,
                qos,
            )

            executor = SingleThreadedExecutor()
            executor.add_node(node)

            thread = threading.Thread(
                target=executor.spin,
                name='omega_sensor_ws_bridge_spin',
                daemon=True,
            )
            thread.start()
            bridge._thread = thread
            bridge._enabled = True

            log.info('OmegaSensorBridge started (thread=%s)', thread.name)
            return bridge

        except Exception as exc:
            log.warning('OmegaSensorBridge failed to start: %s -- no-op', exc)
            return cls(loop, None, None)

    # ------------------------------------------------------------------
    # Ultrasonic subscriptions
    # ------------------------------------------------------------------

    def add_ultrasonic_queue(self, q: asyncio.Queue) -> None:
        with self._lock:
            self._us_queues.add(q)

    def remove_ultrasonic_queue(self, q: asyncio.Queue) -> None:
        with self._lock:
            self._us_queues.discard(q)

    def _on_ultrasonic(self, msg: 'Range') -> None:
        r = msg.range
        dist_m = None if math.isinf(r) or math.isnan(r) else round(r, 3)
        data = {
            'type': 'ultrasonic',
            'distance_m': dist_m,
            'distance_cm': round(dist_m * 100, 1) if dist_m is not None else None,
            'distance_inch': round(dist_m * 39.3701, 1) if dist_m is not None else None,
            'ts': int(time.time() * 1000),
        }
        self._fan_out(self._us_queues, data)

    # ------------------------------------------------------------------
    # Line tracking subscriptions
    # ------------------------------------------------------------------

    def add_line_queue(self, q: asyncio.Queue) -> None:
        with self._lock:
            self._line_queues.add(q)

    def remove_line_queue(self, q: asyncio.Queue) -> None:
        with self._lock:
            self._line_queues.discard(q)

    def _on_line_state(self, msg: 'String') -> None:
        try:
            parsed = json.loads(msg.data)
        except Exception:
            parsed = {'raw': msg.data}
        parsed['type'] = 'line_tracking'
        self._fan_out(self._line_queues, parsed)

    # ------------------------------------------------------------------
    # Internal fan-out (called from rclpy thread)
    # ------------------------------------------------------------------

    def _fan_out(self, queues: Set[asyncio.Queue], data: dict) -> None:
        with self._lock:
            snapshot = set(queues)
        for q in snapshot:
            try:
                self._loop.call_soon_threadsafe(q.put_nowait, data)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @property
    def is_active(self) -> bool:
        return self._enabled

    def shutdown(self) -> None:
        """Best-effort clean shutdown; the spin thread is a daemon so it
        dies with the process regardless."""
        if not self._enabled:
            return
        try:
            if self._node:
                self._node.destroy_node()
        except Exception as exc:
            log.debug('OmegaSensorBridge shutdown error (non-fatal): %s', exc)
        log.info('OmegaSensorBridge shut down')
