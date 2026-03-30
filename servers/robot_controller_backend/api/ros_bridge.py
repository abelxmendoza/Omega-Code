"""
Omega ROS Bridge -- FastAPI integration
========================================
Embeds a rclpy node inside the FastAPI process so that existing WebSocket
and REST handlers can publish to ROS topics without changing the UI protocol.

Architecture
------------
  [Next.js UI]
      |  WebSocket / REST  (unchanged)
      v
  [FastAPI  (this process)]
      |  rclpy publish     (this module)
      v
  [ROS2 DDS -- /cmd_vel, /omega/lighting_cmd, ...]
      |  DDS
      v
  [motor_controller_node, sensor_node, ...]  (separate processes / machines)

Usage (in FastAPI app startup)
-------------------------------
  from api.ros_bridge import OmegaRosBridge

  bridge: OmegaRosBridge | None = None

  @app.on_event('startup')
  async def startup():
      global bridge
      bridge = OmegaRosBridge.create()   # no-op if rclpy unavailable

  @app.on_event('shutdown')
  async def shutdown():
      if bridge:
          bridge.shutdown()

Usage (in a WebSocket handler)
-------------------------------
  async def ws_movement(websocket: WebSocket):
      data = await websocket.receive_json()
      if bridge:
          bridge.send_command(data.get('command'), data.get('speed', 0.5))
      ...

Usage (via REST)
-----------------
  @router.post('/move')
  async def move(cmd: MoveCommand):
      if bridge:
          bridge.send_twist(cmd.linear_x, cmd.angular_z)
      ...

Thread safety
-------------
rclpy.spin() blocks the calling thread.  We run it in a daemon thread so
FastAPI's asyncio event loop is never blocked.  The only cross-thread
operations are `publisher.publish()` calls, which are thread-safe in rclpy.

Environment variables
---------------------
  ROS_BRIDGE_ENABLED   '1' (default '1')  -- set '0' to disable entirely
  ROS_DOMAIN_ID        integer            -- inherited by rclpy automatically
"""

from __future__ import annotations

import os
import threading
import logging
import time
from typing import Optional

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Optional rclpy import -- the bridge degrades gracefully when ROS is absent
# ---------------------------------------------------------------------------
_rclpy_ok = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (
        QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
    )
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, Bool
    from sensor_msgs.msg import BatteryState
    _rclpy_ok = True
except ImportError as _ie:
    log.info('rclpy not available (%s) -- ROS bridge will be a no-op', _ie)


def _reliable_qos(depth: int = 10) -> 'QoSProfile':
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _best_effort_qos(depth: int = 5) -> 'QoSProfile':
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ---------------------------------------------------------------------------
# Command translation table
# Maps legacy UI command strings to (linear_x, angular_z) normalised [-1, 1].
# These match the keys used by the existing movement_ws_server protocol.
# ---------------------------------------------------------------------------
_CMD_TO_TWIST: dict[str, tuple[float, float]] = {
    'forward':    ( 1.0,  0.0),
    'backward':   (-1.0,  0.0),
    'left':       ( 0.0,  1.0),   # CCW = left in ROS convention
    'right':      ( 0.0, -1.0),   # CW  = right
    'pivot_left': ( 0.0,  1.0),
    'pivot_right':( 0.0, -1.0),
    'stop':       ( 0.0,  0.0),
    # diagonal helpers
    'forward_left':  ( 0.7,  0.5),
    'forward_right': ( 0.7, -0.5),
    'backward_left': (-0.7,  0.5),
    'backward_right':(-0.7, -0.5),
}


# ---------------------------------------------------------------------------
# The bridge node -- only instantiated when rclpy is available
# ---------------------------------------------------------------------------

if _rclpy_ok:

    class _BridgeNode(Node):
        """
        Lightweight ROS node embedded inside the FastAPI process.

        Publishes only; all subscriptions live in the dedicated ROS nodes
        (motor_controller_node, sensor_node, etc.).
        """

        def __init__(self) -> None:
            super().__init__('omega_web_bridge')

            # /cmd_vel_in -- consumed by ultrasonic_avoidance_node, which
            # filters and republishes to /cmd_vel for motor_controller_node.
            # When the avoidance node is not running this topic is unused;
            # remap the motor_controller subscriber or launch avoidance node.
            self._cmd_vel_pub = self.create_publisher(
                Twist, '/cmd_vel_in', _reliable_qos(depth=10)
            )

            # /omega/lighting_cmd -- consumed by a future lighting ROS node
            # (Go server still owns actual LED hardware; this is for future use)
            self._lighting_pub = self.create_publisher(
                String, '/omega/lighting_cmd', _best_effort_qos()
            )

            # /omega/system_cmd -- generic system-level commands
            self._syscmd_pub = self.create_publisher(
                String, '/omega/system_cmd', _best_effort_qos()
            )

            # /omega/buzzer_cmd -- consumed by buzzer_node (GPIO 17)
            self._buzzer_pub = self.create_publisher(
                String, '/omega/buzzer_cmd', _best_effort_qos()
            )

            # /omega/servo_cmd -- consumed by servo_node (PCA9685 ch 8-15)
            self._servo_pub = self.create_publisher(
                String, '/omega/servo_cmd', _best_effort_qos()
            )

            self.get_logger().info(
                'omega_web_bridge ready -- publishing to /cmd_vel_in, '
                '/omega/lighting_cmd, /omega/system_cmd, '
                '/omega/buzzer_cmd, /omega/servo_cmd'
            )

        # --- velocity helpers ------------------------------------------

        def publish_twist(self, linear_x: float, angular_z: float) -> None:
            """
            Publish a geometry_msgs/Twist directly.

            linear_x  and  angular_z  should be in [-1.0, 1.0] (normalised).
            motor_controller_node scales them to PWM internally.
            """
            msg = Twist()
            msg.linear.x  = max(-1.0, min(1.0, float(linear_x)))
            msg.angular.z = max(-1.0, min(1.0, float(angular_z)))
            self._cmd_vel_pub.publish(msg)

        def publish_stop(self) -> None:
            """Publish a zero-velocity Twist (safe stop)."""
            self.publish_twist(0.0, 0.0)

        # --- lighting helpers ------------------------------------------

        def publish_lighting_cmd(self, pattern: str, **kwargs) -> None:
            """
            Publish a lighting command as JSON String.
            The Go lighting server remains the hardware owner; this topic is
            for future ROS-native lighting integration.
            """
            import json
            msg = String()
            msg.data = json.dumps({'pattern': pattern, **kwargs})
            self._lighting_pub.publish(msg)

        # --- system helpers -------------------------------------------

        def publish_system_cmd(self, command: str, **kwargs) -> None:
            import json
            msg = String()
            msg.data = json.dumps({'command': command, **kwargs,
                                   'ts_ms': int(time.time() * 1000)})
            self._syscmd_pub.publish(msg)

        # --- buzzer helpers -------------------------------------------

        def publish_buzzer_cmd(self, action: str, **kwargs) -> None:
            """
            Publish a buzzer command to /omega/buzzer_cmd.

            action: 'on' | 'off' | 'for' | 'pulse'
            kwargs: durationMs, onMs, offMs, repeat
            """
            import json
            msg = String()
            msg.data = json.dumps({'action': action, **kwargs,
                                   'ts_ms': int(time.time() * 1000)})
            self._buzzer_pub.publish(msg)

        # --- servo helpers --------------------------------------------

        def publish_servo_cmd(self, channel: str, angle: int, error: int = 10) -> None:
            """
            Publish a servo command to /omega/servo_cmd.

            channel: '0'–'7' (maps to PCA9685 ch 8–15)
                     'horizontal' → '0', 'vertical' → '1'
            angle:   0–180 degrees
            error:   trim offset (default 10 per Freenove spec)
            """
            import json
            ch = '0' if channel in ('horizontal', '0') else \
                 '1' if channel in ('vertical', '1') else channel
            msg = String()
            msg.data = json.dumps({'channel': ch, 'angle': int(angle),
                                   'error': error,
                                   'ts_ms': int(time.time() * 1000)})
            self._servo_pub.publish(msg)


# ---------------------------------------------------------------------------
# Public facade -- always safe to call even when ROS is absent
# ---------------------------------------------------------------------------

class OmegaRosBridge:
    """
    Thread-safe facade over the embedded ROS bridge node.

    All public methods are safe to call from FastAPI async handlers AND
    from sync code.  When rclpy is unavailable the instance is created as
    a no-op stub -- no exceptions, no log spam.
    """

    def __init__(self, node: Optional['_BridgeNode'], executor_thread: Optional[threading.Thread]) -> None:
        self._node = node
        self._thread = executor_thread
        self._enabled = node is not None

    # ------------------------------------------------------------------
    @classmethod
    def create(cls, node_name: str = 'omega_web_bridge') -> 'OmegaRosBridge':
        """
        Factory.  Call once at FastAPI startup.

        Returns a functional bridge if rclpy is available, otherwise a no-op
        instance.  Never raises.
        """
        if not _rclpy_ok:
            log.info('ROS bridge: rclpy absent -- running as no-op')
            return cls(None, None)

        if os.environ.get('ROS_BRIDGE_ENABLED', '1') == '0':
            log.info('ROS bridge: disabled via ROS_BRIDGE_ENABLED=0')
            return cls(None, None)

        try:
            if not rclpy.ok():
                rclpy.init()

            node = _BridgeNode()
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            thread = threading.Thread(
                target=_spin_executor,
                args=(executor,),
                name='omega_ros_bridge_spin',
                daemon=True,
            )
            thread.start()

            log.info('ROS bridge started (thread=%s)', thread.name)
            return cls(node, thread)

        except Exception as exc:
            log.warning('ROS bridge failed to start: %s -- running as no-op', exc)
            return cls(None, None)

    # ------------------------------------------------------------------
    # Movement commands
    # ------------------------------------------------------------------

    def send_command(self, command: str, speed: float = 0.5) -> bool:
        """
        Translate a legacy UI command string into a /cmd_vel Twist.

        Args:
            command: One of the keys in _CMD_TO_TWIST  (e.g. 'forward')
            speed:   Normalised speed scalar [0.0, 1.0] applied to the
                     linear and angular components.

        Returns:
            True if published, False if bridge is disabled or command unknown.

        Example:
            bridge.send_command('forward', speed=0.7)
            bridge.send_command('left',    speed=0.4)
            bridge.send_command('stop')
        """
        if not self._enabled:
            return False

        cmd = command.lower().strip()
        if cmd not in _CMD_TO_TWIST:
            log.warning('ROS bridge: unknown command %r', command)
            return False

        lin, ang = _CMD_TO_TWIST[cmd]
        speed = max(0.0, min(1.0, float(speed)))

        self._node.publish_twist(lin * speed, ang * speed)
        return True

    def send_twist(self, linear_x: float, angular_z: float) -> bool:
        """
        Publish a raw Twist.  Use this when the caller already has velocity
        values (e.g. from a joystick axis or the Xbox controller bridge).
        """
        if not self._enabled:
            return False
        self._node.publish_twist(linear_x, angular_z)
        return True

    def stop(self) -> bool:
        """Publish a zero Twist (immediate stop)."""
        if not self._enabled:
            return False
        self._node.publish_stop()
        return True

    # ------------------------------------------------------------------
    # Lighting / system
    # ------------------------------------------------------------------

    def send_lighting_cmd(self, pattern: str, **kwargs) -> bool:
        if not self._enabled:
            return False
        self._node.publish_lighting_cmd(pattern, **kwargs)
        return True

    def send_system_cmd(self, command: str, **kwargs) -> bool:
        if not self._enabled:
            return False
        self._node.publish_system_cmd(command, **kwargs)
        return True

    def send_buzzer_cmd(self, action: str, **kwargs) -> bool:
        """
        Publish a buzzer command.

        action: 'on' | 'off' | 'for' | 'pulse'
        kwargs: durationMs, onMs, offMs, repeat
        """
        if not self._enabled:
            return False
        self._node.publish_buzzer_cmd(action, **kwargs)
        return True

    def send_servo_cmd(self, channel: str, angle: int, error: int = 10) -> bool:
        """
        Publish a servo position command.

        channel: 'horizontal'|'0' (ultrasonic pan, inverted, PCA9685 ch8)
                 'vertical'|'1'   (tilt, PCA9685 ch9)
                 '2'–'7'          (aux servos, PCA9685 ch10–15)
        angle:   0–180 degrees
        error:   trim offset (default 10)
        """
        if not self._enabled:
            return False
        self._node.publish_servo_cmd(channel, angle, error)
        return True

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @property
    def is_active(self) -> bool:
        return self._enabled

    def shutdown(self) -> None:
        """
        Clean shutdown.  Call from FastAPI shutdown event.
        The spin thread is a daemon so it dies with the process anyway,
        but explicit shutdown gives cleaner ROS node lifecycle logs.
        """
        if not self._enabled:
            return
        try:
            self._node.publish_stop()   # safety: stop motors on shutdown
            self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            log.info('ROS bridge shut down cleanly')
        except Exception as exc:
            log.warning('ROS bridge shutdown error (non-fatal): %s', exc)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _spin_executor(executor: 'SingleThreadedExecutor') -> None:
    """Target for the bridge spin thread.  Exits on shutdown or exception."""
    try:
        executor.spin()
    except Exception as exc:
        log.info('ROS bridge spin thread exiting: %s', exc)


# ---------------------------------------------------------------------------
# FastAPI lifespan helpers (convenience wrappers for use with lifespan context)
# ---------------------------------------------------------------------------

def create_bridge() -> OmegaRosBridge:
    """Shortcut: OmegaRosBridge.create()."""
    return OmegaRosBridge.create()


# ---------------------------------------------------------------------------
# Example WebSocket handler (for reference -- paste into your router)
# ---------------------------------------------------------------------------
#
# from fastapi import WebSocket, WebSocketDisconnect
# from api.ros_bridge import OmegaRosBridge
#
# _bridge: OmegaRosBridge | None = None  # set at app startup
#
# @router.websocket('/ws/movement')
# async def ws_movement(websocket: WebSocket):
#     await websocket.accept()
#     try:
#         while True:
#             data = await websocket.receive_json()
#             cmd   = data.get('command', '')
#             speed = float(data.get('speed', 0.5))
#
#             if _bridge:
#                 published = _bridge.send_command(cmd, speed)
#             else:
#                 published = False
#
#             await websocket.send_json({
#                 'type':      'ack',
#                 'command':   cmd,
#                 'ros_sent':  published,
#                 'ts':        int(time.time() * 1000),
#             })
#     except WebSocketDisconnect:
#         if _bridge:
#             _bridge.stop()   # safety stop on disconnect
