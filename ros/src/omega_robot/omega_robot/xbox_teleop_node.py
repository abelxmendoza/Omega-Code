#!/usr/bin/env python3
"""
Omega Xbox Teleop Node
======================
GTA-style Xbox controller teleoperation — reads evdev directly and publishes
to /cmd_vel so motor_controller_node drives the hardware.

Controls
--------
  Right Trigger (ABS_RZ) : Forward   (hold to go)
  Left Trigger  (ABS_Z)  : Reverse / Brake
  Left Stick X  (ABS_X)  : Steer     (adds angular.z)
  A Button      (BTN_SOUTH): Emergency stop (zeroes velocity while held)
  B Button      (BTN_EAST) : Horn / Buzzer (future — publishes /omega/system_cmd)
  D-Pad Up/Down (ABS_HAT0Y): Camera tilt  (future — publishes /omega/camera_cmd)
  D-Pad L/R    (ABS_HAT0X): Camera pan   (future)

Combine triggers + stick for simultaneous drive + steer.
Pure stick with no triggers = in-place pivot.

The evdev loop runs in a daemon thread; /cmd_vel is published at cmd_rate_hz
(default 20 Hz) from a ROS2 timer so the motor watchdog stays satisfied.

Parameters
----------
  device_path  str   ''    Auto-detect (first evdev device with ABS_RZ axis)
  max_linear   float 1.0   Normalised max linear.x  (motor node scales to PWM)
  max_angular  float 1.0   Normalised max angular.z
  dead_zone    float 0.10  Axis dead-zone (applied after normalisation)
  cmd_rate_hz  float 20.0  /cmd_vel publish rate
  watchdog_s   float 0.50  Publish zero if no controller event for this long

Setup on Pi
-----------
  sudo usermod -aG input omega1   # allow /dev/input/* without root
  pip install evdev
  # then log out / log back in (or newgrp input)
"""

from __future__ import annotations

import select
import threading
import time
import logging
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

# ---------------------------------------------------------------------------
# evdev import (optional — node falls back to zero-velocity if absent)
# ---------------------------------------------------------------------------
_evdev_ok = False
try:
    from evdev import InputDevice, list_devices, ecodes  # type: ignore
    _evdev_ok = True
except ImportError:
    logging.getLogger(__name__).warning(
        'evdev not installed -- Xbox teleop will publish zero velocity. '
        'Install with: pip install evdev'
    )

# ---------------------------------------------------------------------------
# Xbox controller evdev axis / button codes
# (valid for Xbox One S/X controller via xpad or bluetooth on Linux)
# ---------------------------------------------------------------------------
if _evdev_ok:
    _ABS_LX  = ecodes.ABS_X     # Left stick horizontal
    _ABS_LY  = ecodes.ABS_Y     # Left stick vertical (unused in GTA-style)
    _ABS_LT  = ecodes.ABS_Z     # Left trigger  (0 = released, max = full)
    _ABS_RT  = ecodes.ABS_RZ    # Right trigger (0 = released, max = full)
    _ABS_DX  = ecodes.ABS_HAT0X # D-Pad horizontal (-1=left, +1=right)
    _ABS_DY  = ecodes.ABS_HAT0Y # D-Pad vertical   (-1=up,   +1=down)
    _BTN_A   = ecodes.BTN_SOUTH  # A / Cross (emergency stop while held)
    _BTN_B   = ecodes.BTN_EAST   # B / Circle (horn)
    _BTN_X   = ecodes.BTN_NORTH  # X / Square
    _BTN_Y   = ecodes.BTN_WEST   # Y / Triangle


# ---------------------------------------------------------------------------
# QoS
# ---------------------------------------------------------------------------

def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class XboxTeleopNode(Node):
    """
    Reads Xbox controller via evdev (daemon thread) and publishes /cmd_vel
    at a fixed rate from a ROS2 timer so the motor watchdog stays fed.
    """

    def __init__(self) -> None:
        super().__init__('omega_xbox_teleop')

        # ---- parameters -----------------------------------------------
        self.declare_parameter('device_path',  '')
        self.declare_parameter('max_linear',   1.0)
        self.declare_parameter('max_angular',  1.0)
        self.declare_parameter('dead_zone',    0.10)
        self.declare_parameter('cmd_rate_hz',  20.0)
        self.declare_parameter('watchdog_s',   0.50)

        p = self.get_parameter
        self._device_path: str   = p('device_path').value
        self._max_lin: float     = p('max_linear').value
        self._max_ang: float     = p('max_angular').value
        self._dead_zone: float   = p('dead_zone').value
        self._watchdog_s: float  = p('watchdog_s').value

        # ---- publisher ------------------------------------------------
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', _reliable_qos())

        # ---- controller state (protected by lock) ---------------------
        self._lock = threading.Lock()
        self._lin: float   = 0.0   # [-1, 1] normalised
        self._ang: float   = 0.0   # [-1, 1] normalised
        self._estop: bool  = False  # A button held
        self._last_event_t: float = time.monotonic()
        self._connected: bool = False

        # ---- evdev background thread ----------------------------------
        self._running = True
        if _evdev_ok:
            self._evdev_thread = threading.Thread(
                target=self._evdev_loop,
                name='xbox_evdev',
                daemon=True,
            )
            self._evdev_thread.start()
        else:
            self.get_logger().warning('evdev unavailable -- publishing zero velocity')

        # ---- timers ---------------------------------------------------
        rate = p('cmd_rate_hz').value
        self._cmd_timer = self.create_timer(1.0 / rate, self._publish_cmd)
        self._wd_timer  = self.create_timer(0.1,        self._watchdog_check)

        self.get_logger().info(
            f'XboxTeleopNode ready '
            f'[max_lin={self._max_lin} max_ang={self._max_ang} '
            f'dead_zone={self._dead_zone} watchdog={self._watchdog_s}s '
            f'rate={rate}Hz]'
        )
        self.get_logger().info(
            'Waiting for Xbox controller on /dev/input/* ...'
        )

    # ------------------------------------------------------------------
    # evdev background thread
    # ------------------------------------------------------------------

    def _evdev_loop(self) -> None:
        """Find controller, read events, reconnect on disconnect."""
        while self._running:
            device = self._open_controller()
            if device is None:
                time.sleep(2.0)
                continue

            with self._lock:
                self._connected = True
                self._last_event_t = time.monotonic()

            self.get_logger().info(
                f'Xbox controller connected: {device.name} ({device.path})'
            )

            try:
                self._read_device(device)
            except Exception as exc:
                self.get_logger().warning(f'Controller read error: {exc}')
            finally:
                with self._lock:
                    self._connected = False
                    self._lin = 0.0
                    self._ang = 0.0
                    self._estop = False
                try:
                    device.close()
                except Exception:
                    pass
                self.get_logger().warning(
                    'Xbox controller disconnected -- retrying in 2 s'
                )

    def _open_controller(self) -> Optional['InputDevice']:
        """Return first InputDevice that looks like an Xbox controller."""
        path = self._device_path
        if path:
            try:
                return InputDevice(path)
            except Exception as exc:
                self.get_logger().debug(f'Could not open {path}: {exc}')
                return None

        # Auto-detect: look for a device with both trigger axes
        for dev_path in list_devices():
            try:
                dev = InputDevice(dev_path)
                caps = dev.capabilities()
                abs_caps = caps.get(ecodes.EV_ABS, {})
                # Must have right trigger and left stick X
                if _ABS_RT in abs_caps and _ABS_LX in abs_caps:
                    return dev
                dev.close()
            except Exception:
                pass
        return None

    def _read_device(self, device: 'InputDevice') -> None:
        """
        Non-blocking evdev read loop using select() so we can check
        self._running and exit cleanly on shutdown.
        """
        abs_caps = device.capabilities().get(ecodes.EV_ABS, {})

        while self._running:
            r, _, _ = select.select([device.fd], [], [], 0.1)
            if not r:
                continue

            for event in device.read():
                if not self._running:
                    return

                if event.type == ecodes.EV_ABS:
                    self._handle_abs(event, abs_caps)
                elif event.type == ecodes.EV_KEY:
                    self._handle_key(event)

    # ------------------------------------------------------------------
    # Event handlers (called from evdev thread — use lock)
    # ------------------------------------------------------------------

    def _handle_abs(self, event, abs_caps: dict) -> None:
        code = event.code

        if code == _ABS_RT:
            # Right trigger → forward
            info = abs_caps.get(_ABS_RT)
            fwd = self._norm_trigger(event.value, info)
            with self._lock:
                self._last_event_t = time.monotonic()
                # Store as positive linear contribution
                self._lin = self._combine_triggers(fwd, self._reverse)

        elif code == _ABS_LT:
            # Left trigger → reverse
            info = abs_caps.get(_ABS_LT)
            rev = self._norm_trigger(event.value, info)
            with self._lock:
                self._last_event_t = time.monotonic()
                self._lin = self._combine_triggers(self._forward, rev)

        elif code == _ABS_LX:
            # Left stick X → angular
            info = abs_caps.get(_ABS_LX)
            ang = self._norm_stick(event.value, info)
            with self._lock:
                self._last_event_t = time.monotonic()
                self._ang = -ang   # positive stick right → turn right (negative angular)

        # D-Pad: store for future camera servo integration
        # (no publisher wired yet)

    def _handle_key(self, event) -> None:
        if event.code == _BTN_A:
            with self._lock:
                self._last_event_t = time.monotonic()
                self._estop = (event.value == 1)   # held = stop, released = resume
            if event.value == 1:
                self.get_logger().info('E-STOP: A button held -- zeroing velocity')
            else:
                self.get_logger().info('E-STOP released')

    # ------------------------------------------------------------------
    # Trigger/stick normalisation helpers
    # ------------------------------------------------------------------

    def _norm_trigger(self, raw: int, info) -> float:
        """Normalise trigger: 0 (released) → 1.0 (full press)."""
        if info is None:
            return 0.0
        max_v = info.max if hasattr(info, 'max') else 255
        if max_v == 0:
            return 0.0
        v = max(0.0, min(1.0, raw / max_v))
        return v if v > self._dead_zone else 0.0

    def _norm_stick(self, raw: int, info) -> float:
        """Normalise stick axis: center → 0.0, extreme → ±1.0."""
        if info is None:
            return 0.0
        min_v = info.min if hasattr(info, 'min') else -32768
        max_v = info.max if hasattr(info, 'max') else  32767
        center = (min_v + max_v) // 2
        half = max(1, max_v - center)
        v = (raw - center) / half
        v = max(-1.0, min(1.0, v))
        if abs(v) < self._dead_zone:
            return 0.0
        # Re-scale past dead zone so output reaches ±1.0 at full deflection
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - self._dead_zone) / (1.0 - self._dead_zone)

    @staticmethod
    def _combine_triggers(fwd: float, rev: float) -> float:
        """Net linear from forward trigger minus reverse trigger."""
        return max(-1.0, min(1.0, fwd - rev))

    # ------------------------------------------------------------------
    # Properties (thread-safe reads for sub-components)
    # ------------------------------------------------------------------

    @property
    def _forward(self) -> float:
        """Current forward trigger value (already normalised)."""
        # This property is called from _handle_abs which already holds _lock.
        # Internal use only — do not call without lock.
        return max(0.0, self._lin)

    @property
    def _reverse(self) -> float:
        """Current reverse trigger value (already normalised)."""
        return max(0.0, -self._lin)

    # ------------------------------------------------------------------
    # ROS2 timer callbacks (main thread)
    # ------------------------------------------------------------------

    def _publish_cmd(self) -> None:
        with self._lock:
            lin = self._lin
            ang = self._ang
            stop = self._estop

        if stop:
            lin, ang = 0.0, 0.0

        msg = Twist()
        msg.linear.x  = float(lin) * self._max_lin
        msg.angular.z = float(ang) * self._max_ang
        self._cmd_pub.publish(msg)

    def _watchdog_check(self) -> None:
        """Zero velocity if controller has gone silent (disconnect/timeout)."""
        with self._lock:
            connected = self._connected
            stale = (time.monotonic() - self._last_event_t) > self._watchdog_s

        if connected and stale:
            with self._lock:
                self._lin = 0.0
                self._ang = 0.0
            self.get_logger().warning(
                f'No controller event for >{self._watchdog_s}s -- zeroing velocity'
            )

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self.get_logger().info('XboxTeleopNode shutting down -- publishing stop')
        self._running = False
        # Publish one final zero before exiting
        msg = Twist()
        try:
            self._cmd_pub.publish(msg)
        except Exception:
            pass
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = XboxTeleopNode()
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
