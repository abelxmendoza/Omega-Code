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
from collections import deque
from typing import Set, Optional

log = logging.getLogger(__name__)

# Latest ultrasonic reading shared with obstacle avoidance (avoids opening sensor twice).
# _bridge_latest_ultra_ts is the monotonic time of the last VALID reading; 0.0 = never.
_bridge_latest_distance_cm: float | None = None
_bridge_latest_ultra_ts: float = 0.0

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


def _safe_put(q: asyncio.Queue, data: dict) -> None:
    """Drop the oldest item if the queue is full, then enqueue data.

    Called via loop.call_soon_threadsafe(), so it always runs inside the
    asyncio event loop — no locking needed.
    """
    if q.full():
        try:
            q.get_nowait()  # drop oldest sample
        except asyncio.QueueEmpty:
            pass
    try:
        q.put_nowait(data)
    except asyncio.QueueFull:
        pass  # extremely unlikely; discard silently


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

        # Streaming gate: disabled by default so the UI shows 0 until the user
        # (or some control path) explicitly turns the sensor on.
        self._streaming_enabled: bool = False

        self._lock = threading.Lock()
        self._us_queues: Set[asyncio.Queue] = set()
        self._line_queues: Set[asyncio.Queue] = set()

        # Rolling average buffer for ultrasonic smoothing (5-sample window)
        self._ultra_buffer: deque = deque(maxlen=5)

        # Spike rejection: last accepted raw reading (cm)
        self._last_valid_cm: float | None = None

        # Health tracking: monotonic timestamps of last valid reading (0 = never)
        self._last_valid_ultra_ts: float = 0.0
        self._last_valid_line_ts: float = 0.0

    # ------------------------------------------------------------------
    # Factory
    # ------------------------------------------------------------------

    @classmethod
    def create(cls, loop: asyncio.AbstractEventLoop) -> 'OmegaSensorBridge':
        """
        Create and start the sensor bridge.
        Falls back to direct hardware polling when rclpy is unavailable.
        """
        if not _rclpy_ok:
            log.info('Sensor bridge: rclpy absent -- using direct hardware polling')
            bridge = cls(loop, None, None)
            bridge._start_hardware_polling()
            return bridge

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
    # Sensor health (monotonic seconds; 0.0 = never received)
    # ------------------------------------------------------------------

    @property
    def last_valid_ultra_ts(self) -> float:
        return self._last_valid_ultra_ts

    @property
    def last_valid_line_ts(self) -> float:
        return self._last_valid_line_ts

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
        if math.isinf(r) or math.isnan(r):
            return  # reject invalid hardware readings
        raw_cm = r * 100.0

        # Range validation: reject out-of-spec readings
        if raw_cm <= 0 or raw_cm > 400:
            return

        # Spike rejection: discard implausible jumps (> 100 cm in one step)
        if self._last_valid_cm is not None and abs(raw_cm - self._last_valid_cm) > 100:
            log.debug('Ultrasonic spike rejected: %.1f → %.1f cm', self._last_valid_cm, raw_cm)
            return

        self._last_valid_cm = raw_cm
        self._last_valid_ultra_ts = time.monotonic()

        smoothed_cm = self._smooth_ultrasonic(raw_cm)
        dist_m = round(smoothed_cm / 100.0, 3)
        data = {
            'type': 'ultrasonic',
            'distance_cm': round(smoothed_cm, 1),
            'distance_m': dist_m,
            'distance_inch': round(smoothed_cm * 0.393701, 1),
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
            parsed = {}

        # Normalize to blueprint payload regardless of source shape
        lt = parsed.get('lineTracking') or parsed.get('sensors') or parsed
        left   = bool(lt.get('left',   lt.get('Left',   lt.get('IR01', 0))))
        center = bool(lt.get('center', lt.get('Center', lt.get('IR02', 0))))
        right  = bool(lt.get('right',  lt.get('Right',  lt.get('IR03', 0))))

        self._last_valid_line_ts = time.monotonic()
        data = {
            'type': 'line_tracking',
            'left': left,
            'center': center,
            'right': right,
            'on_line': left or center or right,
            'ts': int(time.time() * 1000),
        }
        self._fan_out(self._line_queues, data)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _smooth_ultrasonic(self, dist_cm: float) -> float:
        """5-sample rolling average to reduce ultrasonic noise."""
        self._ultra_buffer.append(dist_cm)
        return sum(self._ultra_buffer) / len(self._ultra_buffer)

    # ------------------------------------------------------------------
    # Streaming gate (power on/off for the data fan-out)
    # ------------------------------------------------------------------

    def enable_streaming(self) -> None:
        """Allow sensor data to flow to WebSocket clients."""
        self._streaming_enabled = True
        log.info('Sensor streaming enabled')

    def disable_streaming(self) -> None:
        """Stop sending sensor data to WebSocket clients (readings still polled)."""
        self._streaming_enabled = False
        log.info('Sensor streaming disabled')

    def is_streaming(self) -> bool:
        return self._streaming_enabled

    def _fan_out(self, queues: Set[asyncio.Queue], data: dict) -> None:
        """Fan data out to all registered queues (called from sensor thread).

        Blocked by the streaming gate — no data leaves when disabled.
        Uses _safe_put (scheduled into the event loop) so a full queue
        drops the oldest sample instead of raising QueueFull.
        """
        if not self._streaming_enabled:
            return
        with self._lock:
            snapshot = set(queues)
        for q in snapshot:
            try:
                self._loop.call_soon_threadsafe(_safe_put, q, data)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Direct hardware polling (no ROS2)
    # ------------------------------------------------------------------

    def _start_hardware_polling(self) -> None:
        """Poll ultrasonic sensor directly at ~10 Hz when ROS2 is unavailable."""
        try:
            from sensors.ultrasonic_sensor import Ultrasonic
            sensor = Ultrasonic()
            self._enabled = True
            log.info('Sensor bridge: direct hardware polling started (ultrasonic)')

            def _poll():
                global _bridge_latest_distance_cm, _bridge_latest_ultra_ts
                try:
                    while True:
                        raw_cm = sensor.get_distance()

                        # Validate range
                        if raw_cm <= 0 or raw_cm > 400:
                            time.sleep(0.1)
                            continue

                        # Spike rejection
                        if (self._last_valid_cm is not None
                                and abs(raw_cm - self._last_valid_cm) > 100):
                            log.debug('Poll spike rejected: %.1f cm', raw_cm)
                            time.sleep(0.1)
                            continue

                        self._last_valid_cm = raw_cm
                        _bridge_latest_distance_cm = raw_cm
                        _bridge_latest_ultra_ts = time.monotonic()
                        self._last_valid_ultra_ts = _bridge_latest_ultra_ts

                        smoothed_cm = self._smooth_ultrasonic(raw_cm)
                        dist_m = round(smoothed_cm / 100.0, 3)
                        data = {
                            'type': 'ultrasonic',
                            'distance_cm': round(smoothed_cm, 1),
                            'distance_m': dist_m,
                            'distance_inch': round(smoothed_cm * 0.393701, 1),
                            'ts': int(time.time() * 1000),
                        }
                        self._fan_out(self._us_queues, data)
                        time.sleep(0.1)  # 10 Hz
                except Exception as exc:
                    log.warning('Sensor hardware poll error: %s', exc)
                finally:
                    try:
                        sensor.close()
                    except Exception:
                        pass

            thread = threading.Thread(target=_poll, daemon=True, name='sensor_hw_poll')
            thread.start()

        except Exception as exc:
            log.warning('Direct hardware polling unavailable: %s', exc)

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
