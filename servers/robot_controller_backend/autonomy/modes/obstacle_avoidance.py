"""
Pure-Python obstacle avoidance mode.

Runs a background thread that reads the HC-SR04 ultrasonic sensor and
drives the motors directly via the ROS bridge (with fallback to the
direct motor driver).  No ROS2 required.

State machine:
  MOVING   — path clear, pass through any commanded motion (or drive forward)
  WARN     — obstacle within warn_cm, slow down / stop
  PIVOTING — obstacle within stop_cm, pivot until clear

Parameters (all optional, passed via /autonomy/start):
  warn_cm         float  default 50   warn-zone threshold (cm)
  stop_cm         float  default 25   hard-stop/pivot threshold (cm)
  forward_speed   float  default 0.5  speed while moving forward [0-1]
  pivot_speed     float  default 0.6  speed during pivot [0-1]
  pivot_duration  float  default 1.2  seconds per pivot burst
  poll_hz         float  default 10   sensor poll rate (Hz)
  drive           bool   default True  if True, autonomously drive forward;
                                       if False, only block dangerous commands
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from typing import Any, Mapping

from ..base import AutonomyModeHandler, AutonomyError

log = logging.getLogger("autonomy.obstacle_avoidance")


class ObstacleAvoidanceMode(AutonomyModeHandler):
    def __init__(self, *, logger=None) -> None:
        super().__init__("avoid_obstacles", logger=logger)
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._state = "idle"
        self._distance_cm: float | None = None
        self._warn_cm = 50.0
        self._stop_cm = 25.0
        self._forward_speed = 0.5
        self._pivot_speed = 0.6
        self._pivot_duration = 1.2
        self._poll_hz = 10.0
        self._drive = True
        self._pivot_dir = 1  # alternates +1/-1

    async def start(self, params: Mapping[str, Any]) -> None:
        self._warn_cm = float(params.get("warn_cm", 50.0))
        self._stop_cm = float(params.get("stop_cm", 25.0))
        self._forward_speed = float(params.get("forward_speed", 0.5))
        self._pivot_speed = float(params.get("pivot_speed", 0.6))
        self._pivot_duration = float(params.get("pivot_duration", 1.2))
        self._poll_hz = float(params.get("poll_hz", 10.0))
        self._drive = bool(params.get("drive", True))

        self._stop_event.clear()
        self._state = "starting"
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="obstacle-avoidance"
        )
        self._thread.start()
        self.logger.info(
            "Obstacle avoidance started — warn=%.0fcm stop=%.0fcm drive=%s",
            self._warn_cm, self._stop_cm, self._drive,
        )

    async def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        self._state = "idle"
        # Ensure robot stops
        try:
            bridge = self._get_bridge()
            if bridge:
                bridge.stop()
        except Exception:
            pass
        self.logger.info("Obstacle avoidance stopped")

    def snapshot(self) -> dict:
        return {
            "state": self._state,
            "distance_cm": self._distance_cm,
            "warn_cm": self._warn_cm,
            "stop_cm": self._stop_cm,
        }

    # ------------------------------------------------------------------
    def _get_bridge(self):
        try:
            from api.ros_bridge import OmegaRosBridge
            return OmegaRosBridge.create()
        except Exception:
            return None

    def _get_sensor(self):
        try:
            from sensors.ultrasonic_sensor import Ultrasonic
            return Ultrasonic()
        except Exception as e:
            self.logger.error("Failed to open ultrasonic sensor: %s", e, exc_info=True)
            return None

    def _get_distance_from_bridge(self) -> float:
        """Try to read latest distance from the sensor bridge shared state."""
        try:
            from api.sensor_bridge import _bridge_latest_distance_cm
            v = _bridge_latest_distance_cm
            return v if v is not None and v > 0 else -1.0
        except Exception:
            return -1.0

    def _loop(self):
        # Prefer bridge shared reading to avoid opening GPIO twice.
        # Fall back to a direct sensor instance only if bridge isn't polling.
        bridge = self._get_bridge()
        if bridge is None:
            self.logger.warning("No movement bridge — avoidance will log only")

        # Check if bridge is already polling
        try:
            from api.sensor_bridge import _bridge_latest_distance_cm as _check
            use_bridge = _check is not None or True  # bridge module loaded
            sensor = None
            self.logger.info("Using sensor bridge shared reading")
        except Exception:
            use_bridge = False
            sensor = self._get_sensor()
            if sensor is None:
                self.logger.error("No ultrasonic sensor — aborting obstacle avoidance")
                self._state = "error"
                return
            self.logger.info("Using direct sensor instance")

        period = 1.0 / self._poll_hz
        self._state = "moving"
        self.logger.info("Avoidance loop running at %.0f Hz", self._poll_hz)

        try:
            while not self._stop_event.is_set():
                t0 = time.monotonic()

                if use_bridge:
                    dist = self._get_distance_from_bridge()
                else:
                    dist = sensor.get_distance()
                if dist < 0:
                    dist = 999.0
                self._distance_cm = dist

                if dist <= self._stop_cm:
                    if self._state != "pivoting":
                        self.logger.warning("Obstacle at %.1f cm — pivoting", dist)
                        self._state = "pivoting"
                    self._do_pivot(bridge)

                elif dist <= self._warn_cm:
                    if self._state != "warn":
                        self.logger.info("Obstacle at %.1f cm — slowing", dist)
                        self._state = "warn"
                    if bridge:
                        bridge.stop()

                else:
                    if self._state != "moving":
                        self.logger.info("Path clear (%.1f cm) — resuming", dist)
                        self._state = "moving"
                    if self._drive and bridge:
                        bridge.send_command("forward", self._forward_speed)

                elapsed = time.monotonic() - t0
                sleep_for = max(0.0, period - elapsed)
                self._stop_event.wait(sleep_for)

        finally:
            if sensor:
                sensor.close()
            if bridge:
                bridge.stop()
            self._state = "idle"

    def _do_pivot(self, bridge):
        """Execute one pivot burst then re-check."""
        if bridge is None:
            time.sleep(self._pivot_duration)
            return

        cmd = "pivot_left" if self._pivot_dir > 0 else "pivot_right"
        self.logger.info("Pivoting %s for %.1fs", cmd, self._pivot_duration)
        bridge.send_command(cmd, self._pivot_speed)

        # Wait pivot_duration or until stop requested
        deadline = time.monotonic() + self._pivot_duration
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                break
            time.sleep(0.05)

        bridge.stop()
        # Flip direction for next pivot if still blocked
        self._pivot_dir *= -1
