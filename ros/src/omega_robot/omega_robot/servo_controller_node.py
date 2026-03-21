#!/usr/bin/env python3
"""
Omega Servo Controller Node
============================
Incremental pan-tilt servo control for a 2-axis camera gimbal.

Published to /omega/servo_increment from either the Xbox teleop node
(D-pad / right stick) or the web GUI (button clicks).

Subscribed topics:
  /omega/servo_increment   geometry_msgs/msg/Vector3   (BEST_EFFORT)
    x  → yaw   delta  (positive = right, negative = left)
    y  → pitch delta  (positive = up,    negative = down)
    z  → ignored

Published topics:
  /omega/servo_state       std_msgs/msg/String   JSON   (BEST_EFFORT, 1 Hz)

Parameters:
  sim_mode         bool    False   suppress all hardware writes
  yaw_ch           int     8       PCA9685 channel for yaw   servo
  pitch_ch         int     9       PCA9685 channel for pitch servo
  yaw_center_us    int     1500    center pulse width (µs) for yaw
  pitch_center_us  int     1500    center pulse width (µs) for pitch
  yaw_min_us       int     1000    hard lower limit for yaw
  yaw_max_us       int     2000    hard upper limit for yaw
  pitch_min_us     int     1300    hard lower limit for pitch (tight — protects loose hardware)
  pitch_max_us     int     1700    hard upper limit for pitch
  scale_us         float   15.0    µs moved per unit of incoming increment
  rate_hz          float   20.0    maximum servo write rate (Hz)
  deadzone         float   0.10    increments below this magnitude are ignored

Design
------
Each /omega/servo_increment message carries a normalised delta value
(-1.0 … 1.0 per axis).  The callback multiplies by scale_us, adds the
result to the running µs position, clamps to the configured limits, and
writes the result via set_servo_pulse().

Rate-limiting is handled in the callback using a monotonic timer so
high-frequency sources (joystick axis spam) don't hammer I2C.

A 1 Hz state publisher emits JSON for monitoring/debugging.
"""

from __future__ import annotations

import json
import time
import logging
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# Hardware import (guarded — works without PCA9685 in sim mode)
# ---------------------------------------------------------------------------
_hw_available = False
try:
    from servers.robot_controller_backend.movement.hardware.pca9685_real import PCA9685
    _hw_available = True
except ImportError:
    logging.getLogger(__name__).warning(
        'PCA9685 driver unavailable -- servo node will run in sim mode'
    )


# ---------------------------------------------------------------------------
# QoS
# ---------------------------------------------------------------------------

def _be_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ServoControllerNode(Node):

    def __init__(self) -> None:
        super().__init__('omega_servo_controller')

        # ---- parameters --------------------------------------------------
        self.declare_parameter('sim_mode',        False)
        self.declare_parameter('yaw_ch',          8)
        self.declare_parameter('pitch_ch',        9)
        self.declare_parameter('yaw_center_us',   1500)
        self.declare_parameter('pitch_center_us', 1500)
        self.declare_parameter('yaw_min_us',      1000)
        self.declare_parameter('yaw_max_us',      2000)
        self.declare_parameter('pitch_min_us',    1300)   # tight — loose hardware
        self.declare_parameter('pitch_max_us',    1700)
        self.declare_parameter('scale_us',        15.0)
        self.declare_parameter('rate_hz',         20.0)
        self.declare_parameter('deadzone',        0.10)

        p = self.get_parameter
        self._sim: bool         = p('sim_mode').value or not _hw_available
        self._yaw_ch: int       = p('yaw_ch').value
        self._pitch_ch: int     = p('pitch_ch').value
        self._yaw_min: int      = p('yaw_min_us').value
        self._yaw_max: int      = p('yaw_max_us').value
        self._pitch_min: int    = p('pitch_min_us').value
        self._pitch_max: int    = p('pitch_max_us').value
        self._scale: float      = p('scale_us').value
        self._deadzone: float   = p('deadzone').value
        self._min_interval: float = 1.0 / max(1.0, p('rate_hz').value)

        # ---- servo position (floats for sub-integer accumulation) --------
        self._yaw_us: float   = float(p('yaw_center_us').value)
        self._pitch_us: float = float(p('pitch_center_us').value)
        self._last_apply_t: float = 0.0   # monotonic time of last hardware write

        # ---- hardware ----------------------------------------------------
        self._pwm: Optional[PCA9685] = None
        if not self._sim:
            try:
                self._pwm = PCA9685()
                self._pwm.set_pwm_freq(50)
                # Move both servos to center on startup (safe, not a jump)
                self._write_hardware()
                self.get_logger().info(
                    f'Servo hardware ready '
                    f'[yaw ch{self._yaw_ch} @ {int(self._yaw_us)}µs, '
                    f'pitch ch{self._pitch_ch} @ {int(self._pitch_us)}µs]'
                )
            except Exception as exc:
                self.get_logger().warning(
                    f'PCA9685 init failed: {exc} -- running in sim mode'
                )
                self._sim = True
                self._pwm = None

        # ---- subscriber --------------------------------------------------
        self._inc_sub = self.create_subscription(
            Vector3,
            '/omega/servo_increment',
            self._on_increment,
            _be_qos(depth=5),
        )

        # ---- state publisher (1 Hz diagnostics) -------------------------
        self._state_pub = self.create_publisher(
            String, '/omega/servo_state', _be_qos(depth=1)
        )
        self._state_timer = self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f'ServoControllerNode ready '
            f'[sim={self._sim} '
            f'yaw={self._yaw_min}-{self._yaw_max}µs '
            f'pitch={self._pitch_min}-{self._pitch_max}µs '
            f'scale={self._scale}µs/unit '
            f'rate={1.0/self._min_interval:.0f}Hz]'
        )

    # ------------------------------------------------------------------
    # Increment callback
    # ------------------------------------------------------------------

    def _on_increment(self, msg: Vector3) -> None:
        """
        Apply one servo increment.

        msg.x → yaw   delta (normalised, positive = right)
        msg.y → pitch delta (normalised, positive = up)

        Rate-limited to at most rate_hz writes per second.
        Values inside the deadzone are discarded.
        """
        # --- rate limit -------------------------------------------------
        now = time.monotonic()
        if (now - self._last_apply_t) < self._min_interval:
            return
        self._last_apply_t = now

        # --- deadzone ---------------------------------------------------
        x = float(msg.x)
        y = float(msg.y)
        x = x if abs(x) > self._deadzone else 0.0
        y = y if abs(y) > self._deadzone else 0.0

        if x == 0.0 and y == 0.0:
            return

        # --- apply increment + clamp ------------------------------------
        self._yaw_us   = max(self._yaw_min,
                             min(self._yaw_max,
                                 self._yaw_us + x * self._scale))
        self._pitch_us = max(self._pitch_min,
                             min(self._pitch_max,
                                 self._pitch_us + y * self._scale))

        # --- write to hardware ------------------------------------------
        self._write_hardware()

        self.get_logger().debug(
            f'servo: yaw={int(self._yaw_us)}µs  pitch={int(self._pitch_us)}µs'
        )

    # ------------------------------------------------------------------
    # Hardware write
    # ------------------------------------------------------------------

    def _write_hardware(self) -> None:
        """Send current yaw/pitch µs values to the PCA9685."""
        if self._sim or self._pwm is None:
            return
        try:
            self._pwm.set_servo_pulse(self._yaw_ch,   int(self._yaw_us))
            self._pwm.set_servo_pulse(self._pitch_ch, int(self._pitch_us))
        except Exception as exc:
            self.get_logger().warning(f'Servo write error: {exc}')

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def _publish_state(self) -> None:
        msg = String()
        msg.data = json.dumps({
            'yaw_us':     int(self._yaw_us),
            'pitch_us':   int(self._pitch_us),
            'yaw_pct':    round((self._yaw_us   - self._yaw_min)
                                / (self._yaw_max   - self._yaw_min) * 100, 1),
            'pitch_pct':  round((self._pitch_us  - self._pitch_min)
                                / (self._pitch_max - self._pitch_min) * 100, 1),
            'yaw_ch':     self._yaw_ch,
            'pitch_ch':   self._pitch_ch,
            'sim_mode':   self._sim,
            'ts_ms':      int(time.time() * 1000),
        })
        self._state_pub.publish(msg)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self.get_logger().info('ServoControllerNode shutting down -- centering servos')
        # Return to center on shutdown so the gimbal rests safely
        self._yaw_us   = float((self._yaw_min   + self._yaw_max)   // 2)
        self._pitch_us = float((self._pitch_min  + self._pitch_max) // 2)
        self._write_hardware()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
