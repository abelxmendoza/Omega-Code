#!/usr/bin/env python3
"""
Omega Motor Controller Node
============================
ROS2 node that subscribes to /cmd_vel and drives the PCA9685-based
differential drive hardware on Omega-1.

Integrates the existing hardware abstraction layer without rewriting it:
  Motor (minimal_motor_control) -- PCA9685 HAL with platform detection
  MovementRamp                  -- smooth acceleration/deceleration
  ThermalSafety                 -- temperature/current protection
  MovementWatchdog              -- runaway-prevention timeout
  StraightDriveAssist           -- drift compensation
  Odometry                      -- dead-reckoning pose tracking

Published topics:
  /odom                  nav_msgs/Odometry          (BEST_EFFORT, 50 Hz)
  /omega/motor_state     std_msgs/String (JSON)     (BEST_EFFORT, 10 Hz)
  /tf                    geometry_msgs/TF           (via TransformBroadcaster)

Subscribed topics:
  /cmd_vel               geometry_msgs/Twist        (RELIABLE)

Parameters:
  sim_mode          bool    False   -- no-op all hardware writes
  wheel_base        float   0.20    -- metres between wheel centrelines
  wheel_radius      float   0.05    -- metres
  max_pwm           int     4095    -- PCA9685 full-scale PWM count
  max_rpm           float   300.0   -- used for odometry RPM estimation
  watchdog_timeout  float   0.50    -- seconds before auto-stop
  thermal_enabled   bool    True    -- enable thermal/current guard
  ramp_enabled      bool    True    -- enable speed ramping
  ramp_type         str    'linear' -- linear | exponential | s_curve
  accel_rate        float   150.0   -- PWM units per second
  decel_rate        float   200.0   -- PWM units per second
"""

from __future__ import annotations

import math
import time
import json
import logging
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros

# ---------------------------------------------------------------------------
# Hardware imports are available because the project root is pip-installed:
#   pip install -e /path/to/Omega-Code/
# This gives `servers.robot_controller_backend.*` as proper Python packages.
# No sys.path manipulation needed.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Hardware imports -- all wrapped so sim_mode works without any drivers
# ---------------------------------------------------------------------------
_hw_available = False
try:
    from servers.robot_controller_backend.movement.minimal_motor_control import Motor
    from servers.robot_controller_backend.movement.movement_ramp import (
        MovementRamp, RampType,
    )
    from servers.robot_controller_backend.movement.thermal_safety import (
        ThermalSafety, ThermalLimits, SafetyState,
    )
    from servers.robot_controller_backend.movement.movement_watchdog import (
        MovementWatchdog,
    )
    from servers.robot_controller_backend.movement.straight_drive_assist import (
        StraightDriveAssist,
    )
    _hw_available = True
except ImportError as _e:
    logging.getLogger(__name__).warning(
        'Hardware modules not importable (%s). Node will run in forced sim_mode.', _e
    )

# Odometry is pure Python — import it independently so it works in sim mode too.
_odom_available = False
try:
    from servers.robot_controller_backend.movement.odometry import Odometry as OmegaOdometry
    _odom_available = True
except ImportError:
    pass


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


def _best_effort_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ---------------------------------------------------------------------------
# Noop hardware stubs -- used in sim_mode
# ---------------------------------------------------------------------------

class _NoopMotor:
    """Silent drop-in for Motor when running without hardware."""
    left_trim = 0
    right_trim = 0

    def _set_lr(self, left: int, right: int) -> None:
        pass

    def stop(self) -> None:
        pass

    def set_trim(self, left: int = 0, right: int = 0):
        return {'left': left, 'right': right}


class _NoopWatchdog:
    """Silent drop-in for MovementWatchdog in sim mode."""
    def kick(self) -> None:
        pass

    def should_stop(self) -> bool:
        return False

    def get_status(self) -> dict:
        return {'state': 'ok', 'elapsed_since_update': 0.0, 'trigger_count': 0}


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class MotorControllerNode(Node):
    """
    Translates /cmd_vel Twist messages into differential-drive PWM outputs
    through the existing Motor HAL, with full safety and odometry support.
    """

    # PWM is 12-bit (0-4095); map normalised velocity [-1,1] to this range.
    _PWM_MAX_FLOAT = 4095.0

    def __init__(self) -> None:
        super().__init__('omega_motor_controller')

        # ---- declare parameters ----------------------------------------
        self.declare_parameter('sim_mode',        False)
        self.declare_parameter('wheel_base',       0.20)
        self.declare_parameter('wheel_radius',     0.05)
        self.declare_parameter('max_pwm',          4095)
        self.declare_parameter('max_rpm',         300.0)
        self.declare_parameter('watchdog_timeout', 0.50)
        self.declare_parameter('thermal_enabled',  True)
        self.declare_parameter('ramp_enabled',     True)
        self.declare_parameter('ramp_type',      'linear')
        self.declare_parameter('accel_rate',     150.0)
        self.declare_parameter('decel_rate',     200.0)

        p = self.get_parameter
        self._sim: bool       = p('sim_mode').value or not _hw_available
        self._wheel_base: float  = p('wheel_base').value
        self._wheel_radius: float = p('wheel_radius').value
        self._max_pwm: int    = p('max_pwm').value
        self._max_rpm: float  = p('max_rpm').value
        self._wd_timeout: float = p('watchdog_timeout').value
        self._ramp_enabled: bool = p('ramp_enabled').value
        self._max_pwm_f: float = float(self._max_pwm)

        # ---- hardware layer -------------------------------------------
        self._motor: _NoopMotor
        if self._sim:
            self._motor = _NoopMotor()
            self.get_logger().info('sim_mode=True -- hardware writes are no-ops')
        else:
            try:
                self._motor = Motor()
            except Exception as _motor_exc:
                self.get_logger().warning(
                    f'Motor hardware init failed: {_motor_exc} -- falling back to sim mode'
                )
                self._sim = True
                self._motor = _NoopMotor()

        # ---- straight-drive assist ------------------------------------
        self._sda = StraightDriveAssist(self._motor) if not self._sim else None

        # ---- speed ramps (independent per wheel) ----------------------
        if self._ramp_enabled and not self._sim:
            _rtype_map = {
                'linear':      RampType.LINEAR,
                'exponential': RampType.EXPONENTIAL,
                's_curve':     RampType.S_CURVE,
            }
            _rtype = _rtype_map.get(p('ramp_type').value, RampType.LINEAR)
            _accel = p('accel_rate').value
            _decel = p('decel_rate').value
            self._left_ramp  = MovementRamp(accel_rate=_accel, decel_rate=_decel,
                                            ramp_type=_rtype, max_pwm=self._max_pwm)
            self._right_ramp = MovementRamp(accel_rate=_accel, decel_rate=_decel,
                                            ramp_type=_rtype, max_pwm=self._max_pwm)
        else:
            self._left_ramp = None
            self._right_ramp = None

        # ---- thermal safety -------------------------------------------
        if p('thermal_enabled').value and not self._sim:
            self._thermal = ThermalSafety(enabled=True)
        else:
            self._thermal = None

        # ---- watchdog -------------------------------------------------
        # We call watchdog.kick() inside the /cmd_vel callback.
        # The control loop checks watchdog.should_stop() every cycle.
        if self._sim:
            self._watchdog = _NoopWatchdog()
        else:
            self._watchdog = MovementWatchdog(
                timeout_sec=self._wd_timeout,
                stop_callback=None,   # handled synchronously in control loop
                enabled=True,
            )

        # ---- odometry -------------------------------------------------
        # OmegaOdometry is pure Python -- create it regardless of sim_mode
        # so /odom always publishes (sim uses dead-reckoning from PWM targets).
        self._odom = OmegaOdometry(
            wheel_base=self._wheel_base,
            wheel_radius=self._wheel_radius,
        ) if _odom_available else None

        # ---- state -------------------------------------------------------
        # Target PWM values written by the subscription callback;
        # read by the control-loop timer -- both run in the same ROS spin
        # thread, so no lock is needed with a SingleThreadedExecutor.
        self._target_left_pwm: float  = 0.0
        self._target_right_pwm: float = 0.0
        self._actual_left_pwm: float  = 0.0
        self._actual_right_pwm: float = 0.0
        self._last_twist = Twist()           # last received command (for logging)
        self._last_ctrl_t: float = time.monotonic()

        # ---- TF broadcaster -------------------------------------------
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---- publishers -----------------------------------------------
        self._odom_pub = self.create_publisher(
            Odometry, '/odom', _best_effort_qos(depth=5)
        )
        self._state_pub = self.create_publisher(
            String, '/omega/motor_state', _best_effort_qos(depth=5)
        )

        # ---- subscriber -----------------------------------------------
        self._cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, _reliable_qos(depth=10)
        )

        # ---- timers ---------------------------------------------------
        # 50 Hz control loop: ramp update + hardware write + odom publish
        self._ctrl_timer = self.create_timer(0.02, self._control_loop)
        # 10 Hz diagnostics: motor state JSON
        self._state_timer = self.create_timer(0.10, self._publish_motor_state)

        self.get_logger().info(
            f'MotorControllerNode ready '
            f'[sim={self._sim} wheel_base={self._wheel_base:.3f}m '
            f'wheel_radius={self._wheel_radius:.3f}m '
            f'watchdog={self._wd_timeout:.2f}s ramp={self._ramp_enabled}]'
        )

    # ------------------------------------------------------------------
    # /cmd_vel callback
    # ------------------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        """
        Translate geometry_msgs/Twist into differential-drive target PWM.

        Convention (ROS REP 103):
          linear.x  > 0  => forward
          angular.z > 0  => counterclockwise (left turn)

        Differential drive mixing:
          left  = (linear - angular) * MAX_PWM
          right = (linear + angular) * MAX_PWM
        """
        self._watchdog.kick()
        self._last_twist = msg

        linear  = float(msg.linear.x)   # normalised to [-1, 1]
        angular = float(msg.angular.z)  # normalised to [-1, 1]

        # Clamp inputs to [-1, 1] so callers can pass raw Twist values
        linear  = max(-1.0, min(1.0, linear))
        angular = max(-1.0, min(1.0, angular))

        self._target_left_pwm  = (linear - angular) * self._max_pwm_f
        self._target_right_pwm = (linear + angular) * self._max_pwm_f

        # Clamp to max PWM (handles combined linear+angular > 1)
        self._target_left_pwm  = max(-self._max_pwm_f,
                                     min(self._max_pwm_f, self._target_left_pwm))
        self._target_right_pwm = max(-self._max_pwm_f,
                                     min(self._max_pwm_f, self._target_right_pwm))

    # ------------------------------------------------------------------
    # 50 Hz control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        now = time.monotonic()
        dt  = now - self._last_ctrl_t
        self._last_ctrl_t = now

        # -- watchdog check ---------------------------------------------
        if self._watchdog.should_stop():
            self._target_left_pwm  = 0.0
            self._target_right_pwm = 0.0

        # -- thermal safety check ---------------------------------------
        if self._thermal is not None:
            # Pass empty telemetry until real current/temp sensors exist.
            # ThermalSafety.check() gracefully returns SafetyState.OK on empty.
            state = self._thermal.check({})
            if state == SafetyState.KILL:
                self._target_left_pwm  = 0.0
                self._target_right_pwm = 0.0
                self.get_logger().warn('Thermal KILL engaged -- motors stopped')
            elif state == SafetyState.THROTTLE:
                factor = self._thermal.limits.throttle_factor
                self._target_left_pwm  *= factor
                self._target_right_pwm *= factor

        # -- ramp update ------------------------------------------------
        if self._left_ramp is not None and dt > 0:
            self._left_ramp.set_target(self._target_left_pwm)
            self._right_ramp.set_target(self._target_right_pwm)
            left_pwm  = self._left_ramp.update(dt)
            right_pwm = self._right_ramp.update(dt)
        else:
            left_pwm  = self._target_left_pwm
            right_pwm = self._target_right_pwm

        # -- hardware write ---------------------------------------------
        self._motor._set_lr(int(left_pwm), int(right_pwm))
        self._actual_left_pwm  = left_pwm
        self._actual_right_pwm = right_pwm

        # -- odometry update + publish ----------------------------------
        if self._odom is not None and dt > 0:
            left_rpm  = self._pwm_to_rpm(left_pwm)
            right_rpm = self._pwm_to_rpm(right_pwm)
            self._odom.update(left_rpm=left_rpm, right_rpm=right_rpm, dt=dt)
            self._publish_odom()

    # ------------------------------------------------------------------
    # Odometry publishing + TF broadcast
    # ------------------------------------------------------------------

    def _publish_odom(self) -> None:
        if self._odom is None:
            return

        pose = self._odom.get_pose()
        vel  = self._odom.get_velocity()
        now  = self.get_clock().now().to_msg()

        # -- nav_msgs/Odometry ------------------------------------------
        msg = Odometry()
        msg.header.stamp    = now
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'

        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = 0.0

        # Heading -> quaternion (rotation around Z)
        h_half = pose.heading * 0.5
        msg.pose.pose.orientation.z = math.sin(h_half)
        msg.pose.pose.orientation.w = math.cos(h_half)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0

        msg.twist.twist.linear.x  = vel['linear']
        msg.twist.twist.angular.z = vel['angular']

        # Covariance: diagonal, tuned for dead-reckoning without encoders.
        # [x, y, z, roll, pitch, yaw] -- large values = low confidence
        msg.pose.covariance[0]  = 0.1   # x
        msg.pose.covariance[7]  = 0.1   # y
        msg.pose.covariance[35] = 0.2   # yaw
        msg.twist.covariance[0]  = 0.05  # vx
        msg.twist.covariance[35] = 0.1   # wz

        self._odom_pub.publish(msg)

        # -- TF: odom -> base_link --------------------------------------
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = now
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'base_link'
        tf_msg.transform.translation.x = pose.x
        tf_msg.transform.translation.y = pose.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z    = math.sin(h_half)
        tf_msg.transform.rotation.w    = math.cos(h_half)
        tf_msg.transform.rotation.x    = 0.0
        tf_msg.transform.rotation.y    = 0.0
        self._tf_broadcaster.sendTransform(tf_msg)

    # ------------------------------------------------------------------
    # 10 Hz diagnostics
    # ------------------------------------------------------------------

    def _publish_motor_state(self) -> None:
        watchdog_status = self._watchdog.get_status()
        payload = {
            'left_pwm':    int(self._actual_left_pwm),
            'right_pwm':   int(self._actual_right_pwm),
            'target_left': int(self._target_left_pwm),
            'target_right': int(self._target_right_pwm),
            'linear':       round(float(self._last_twist.linear.x), 4),
            'angular':      round(float(self._last_twist.angular.z), 4),
            'watchdog': {
                'state':   watchdog_status['state'],
                'elapsed': round(watchdog_status['elapsed_since_update'], 3),
                'triggers': watchdog_status['trigger_count'],
            },
            'sim_mode': self._sim,
            'ts_ms':    int(time.time() * 1000),
        }
        if self._odom is not None:
            odom_status = self._odom.get_status()
            payload['odom'] = {
                'x':       round(odom_status['pose']['x'], 4),
                'y':       round(odom_status['pose']['y'], 4),
                'heading': round(odom_status['pose']['heading_deg'], 2),
                'dist_m':  round(odom_status['distance_traveled'], 3),
            }
        msg = String()
        msg.data = json.dumps(payload)
        self._state_pub.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _pwm_to_rpm(self, pwm: float) -> float:
        """
        Estimate wheel RPM from commanded PWM.

        This is a linear approximation for dead-reckoning in the absence of
        wheel encoders.  Replace with encoder data when hardware is available.
        """
        return (abs(pwm) / self._max_pwm_f) * self._max_rpm * (1.0 if pwm >= 0 else -1.0)

    def _safe_stop(self) -> None:
        """Zero all motor outputs immediately."""
        self._target_left_pwm  = 0.0
        self._target_right_pwm = 0.0
        self._actual_left_pwm  = 0.0
        self._actual_right_pwm = 0.0
        self._motor._set_lr(0, 0)
        self.get_logger().info('Safe stop executed')

    def destroy_node(self) -> None:
        """Ensure motors are stopped before the node shuts down."""
        self.get_logger().info('MotorControllerNode shutting down -- stopping motors')
        try:
            self._safe_stop()
            self._motor.stop()
        except Exception as exc:
            self.get_logger().error(f'Error during shutdown stop: {exc}')
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorControllerNode()
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
