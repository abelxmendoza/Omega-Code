#!/usr/bin/env python3
"""
Ultrasonic Obstacle Avoidance Node
====================================
Acts as a /cmd_vel multiplexer: forwards operator commands through to the
motor controller unless the ultrasonic sensor detects an obstacle, in which
case it suppresses forward motion and executes an in-place pivot turn.

When avoidance is disabled the node is purely a transparent passthrough and
adds zero latency penalty to the drive path.

Topics
------
  Subscribed:
    /cmd_vel_in                        geometry_msgs/Twist  RELIABLE
        Operator / teleop input (bridge, Xbox teleop node)
    /omega/ultrasonic                  sensor_msgs/Range    BEST_EFFORT
        Distance reading from HC-SR04 via sensor_node
    /omega/obstacle_avoidance/enable   std_msgs/Bool        RELIABLE
        GUI toggle — True to activate avoidance logic

  Published:
    /cmd_vel                           geometry_msgs/Twist  RELIABLE
        Final velocity command to motor_controller_node
    /omega/obstacle_avoidance/state    std_msgs/String      BEST_EFFORT
        JSON status snapshot for GUI / debugging

State machine (when enabled)
-----------------------------
  MOVING   — path clear (dist > warn_m), pass operator commands through
  WARN     — approaching obstacle (stop_m < dist <= warn_m), scale linear.x
  PIVOTING — too close (dist <= stop_m), spin in-place until clear

Pivot behaviour
---------------
  On entering PIVOTING the node publishes (0, ±pivot_speed) for pivot_duration_s.
  After each pivot it re-checks distance:
    • Still blocked → flip direction, pivot again
    • Clear         → return to MOVING

Parameters
----------
  warn_distance_m    float  0.50   Slow-down zone start (metres)
  stop_distance_m    float  0.25   Hard-stop / pivot zone (metres)
  pivot_speed        float  0.70   |angular.z| during pivot (rad/s, normalised)
  pivot_duration_s   float  1.20   Duration of each pivot burst (seconds)
  sensor_timeout_s   float  1.00   Treat sensor as blocked if data older than this
  control_rate_hz    float  20.0   Output publish rate (should match teleop rate)
"""

from __future__ import annotations

import enum
import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String


# ---------------------------------------------------------------------------
# QoS helpers
# ---------------------------------------------------------------------------

def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _best_effort_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class AvoidState(enum.Enum):
    PASS_THROUGH = 'pass_through'  # avoidance disabled — pure passthrough
    MOVING       = 'moving'        # enabled, path clear
    WARN         = 'warn'          # enabled, entering warning zone
    PIVOTING     = 'pivoting'      # enabled, executing in-place pivot turn


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class UltrasonicAvoidanceNode(Node):
    """
    Transparent cmd_vel mux with ultrasonic-based obstacle avoidance.

    When disabled:  /cmd_vel_in  →  /cmd_vel  (no modification, no delay)
    When enabled:   monitors /omega/ultrasonic and injects pivot turns when
                    the robot is about to collide with an obstacle.
    """

    def __init__(self) -> None:
        super().__init__('omega_ultrasonic_avoidance')

        # ---- parameters -----------------------------------------------
        self.declare_parameter('warn_distance_m',  0.50)
        self.declare_parameter('stop_distance_m',  0.25)
        self.declare_parameter('pivot_speed',      0.70)
        self.declare_parameter('pivot_duration_s', 1.20)
        self.declare_parameter('sensor_timeout_s', 1.00)
        self.declare_parameter('control_rate_hz',  20.0)

        p = self.get_parameter
        self._warn_m:    float = float(p('warn_distance_m').value)
        self._stop_m:    float = float(p('stop_distance_m').value)
        self._pivot_spd: float = float(p('pivot_speed').value)
        self._pivot_dur: float = float(p('pivot_duration_s').value)
        self._sensor_to: float = float(p('sensor_timeout_s').value)
        self._rate_hz:   float = float(p('control_rate_hz').value)

        # ---- state -------------------------------------------------------
        self._state:       AvoidState     = AvoidState.PASS_THROUGH
        self._enabled:     bool           = False
        self._distance:    Optional[float] = None   # metres, None = no reading
        self._sensor_ts:   float          = 0.0     # monotonic time of last Range msg
        self._last_cmd_in: Twist          = Twist() # latest operator command
        self._pivot_start: float          = 0.0     # monotonic time pivot began
        self._pivot_dir:   float          = 1.0     # +1 = CCW, alternates each pivot

        # ---- publishers --------------------------------------------------
        self._cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', _reliable_qos()
        )
        self._state_pub = self.create_publisher(
            String, '/omega/obstacle_avoidance/state', _best_effort_qos()
        )

        # ---- subscribers -------------------------------------------------
        self.create_subscription(
            Twist, '/cmd_vel_in', self._on_cmd_in, _reliable_qos()
        )
        self.create_subscription(
            Range, '/omega/ultrasonic', self._on_ultrasonic, _best_effort_qos()
        )
        self.create_subscription(
            Bool, '/omega/obstacle_avoidance/enable',
            self._on_enable, _reliable_qos()
        )

        # ---- control timer -----------------------------------------------
        period = 1.0 / self._rate_hz
        self._timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'UltrasonicAvoidanceNode ready '
            f'[warn={self._warn_m:.2f}m stop={self._stop_m:.2f}m '
            f'pivot_spd={self._pivot_spd:.2f} pivot_dur={self._pivot_dur:.2f}s '
            f'rate={self._rate_hz:.0f}Hz]'
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _on_cmd_in(self, msg: Twist) -> None:
        self._last_cmd_in = msg

    def _on_ultrasonic(self, msg: Range) -> None:
        # Per REP-117: range == inf means out-of-range (sensor sees nothing)
        if msg.range == float('inf') or msg.range < 0.0:
            self._distance = None
        else:
            self._distance = float(msg.range)
        self._sensor_ts = time.monotonic()

    def _on_enable(self, msg: Bool) -> None:
        was_enabled = self._enabled
        self._enabled = bool(msg.data)

        if self._enabled and not was_enabled:
            self._state = AvoidState.MOVING
            self.get_logger().info('Obstacle avoidance ENABLED')
        elif not self._enabled and was_enabled:
            self._state = AvoidState.PASS_THROUGH
            self.get_logger().info('Obstacle avoidance DISABLED — passthrough mode')

    # ------------------------------------------------------------------
    # 20 Hz control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        now = time.monotonic()

        # ----- DISABLED: transparent passthrough ----------------------
        if not self._enabled:
            self._state = AvoidState.PASS_THROUGH
            self._cmd_pub.publish(self._last_cmd_in)
            self._publish_state(now)
            return

        # ----- Sensor freshness check ---------------------------------
        sensor_stale = (now - self._sensor_ts) > self._sensor_to
        # Safe-fail: if sensor data is stale or missing, treat as blocked
        if sensor_stale or self._distance is None:
            effective_dist = 0.0
        else:
            effective_dist = self._distance

        # ----- State transitions --------------------------------------
        if self._state == AvoidState.PIVOTING:
            elapsed = now - self._pivot_start
            if elapsed >= self._pivot_dur:
                if effective_dist <= self._stop_m:
                    # Still blocked after pivot — reverse direction and try again
                    self._pivot_dir *= -1.0
                    self._pivot_start = now
                    self.get_logger().info(
                        f'Path still blocked at {effective_dist:.2f}m — '
                        f'reversing pivot to {"CCW" if self._pivot_dir > 0 else "CW"}'
                    )
                else:
                    self._state = AvoidState.MOVING
                    self.get_logger().info(
                        f'Pivot complete — path clear at {effective_dist:.2f}m, resuming'
                    )
        else:
            # Distance-based state selection
            if effective_dist <= self._stop_m:
                if self._state != AvoidState.PIVOTING:
                    self._state = AvoidState.PIVOTING
                    self._pivot_start = now
                    self.get_logger().info(
                        f'Obstacle at {effective_dist:.2f}m — '
                        f'starting {"CCW" if self._pivot_dir > 0 else "CW"} pivot'
                    )
            elif effective_dist <= self._warn_m:
                self._state = AvoidState.WARN
            else:
                self._state = AvoidState.MOVING

        # ----- Output twist -------------------------------------------
        out = Twist()

        if self._state == AvoidState.PIVOTING:
            # In-place pivot: no forward motion, spin only
            out.linear.x  = 0.0
            out.angular.z = self._pivot_spd * self._pivot_dir

        elif self._state == AvoidState.WARN:
            # Proportionally scale forward speed based on distance remaining
            # to the stop threshold. Angular and reverse motion pass through.
            cmd = self._last_cmd_in
            range_m = max(self._warn_m - self._stop_m, 1e-6)
            scale   = max(0.0, min(1.0, (effective_dist - self._stop_m) / range_m))

            if cmd.linear.x > 0.0:
                out.linear.x = cmd.linear.x * scale
            else:
                out.linear.x = cmd.linear.x   # reverse always passes through

            out.linear.y  = cmd.linear.y
            out.linear.z  = cmd.linear.z
            out.angular.x = cmd.angular.x
            out.angular.y = cmd.angular.y
            out.angular.z = cmd.angular.z

        else:  # MOVING
            out = self._last_cmd_in

        self._cmd_pub.publish(out)
        self._publish_state(now)

    # ------------------------------------------------------------------
    # Status publish
    # ------------------------------------------------------------------

    def _publish_state(self, now: float) -> None:
        sensor_stale = (now - self._sensor_ts) > self._sensor_to
        payload = {
            'state':        self._state.value,
            'enabled':      self._enabled,
            'distance_m':   round(self._distance, 3) if self._distance is not None else None,
            'sensor_stale': sensor_stale,
            'pivot_dir':    'ccw' if self._pivot_dir > 0 else 'cw',
            'warn_m':       self._warn_m,
            'stop_m':       self._stop_m,
            'ts_ms':        int(time.time() * 1000),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._state_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicAvoidanceNode()
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
