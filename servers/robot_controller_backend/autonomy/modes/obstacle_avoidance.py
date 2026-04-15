"""
Obstacle avoidance autonomy mode — enhanced.

Strategy
--------
1.  Drive forward continuously (if drive=True).
2.  WARN zone (stop_cm < dist ≤ warn_cm): proportionally reduce forward speed
    so the robot decelerates smoothly rather than stopping abruptly.
3.  STOP zone (dist ≤ stop_cm): pivot until the full warn zone is clear again
    (not a fixed-duration burst — handles obstacles of any width).
4.  After each pivot the robot does a brief counter-pivot recovery step to
    partially restore the original heading across multiple consecutive obstacles.
5.  Camera secondary sensor (optional, Pi 4B compatible):
    background thread fetches a snapshot from the Flask video server at ~2 FPS
    and runs a cheap edge-density check on the lower-centre ROI.  When the
    camera flags an obstacle the effective stop threshold is raised to give
    more reaction margin.
6.  Emergency stop: imports the shared ESTOP_EVENT from movement.estop.
    When set the avoidance loop halts motors immediately and waits until the
    event is cleared (happens automatically on the next /autonomy/start) or
    until the mode is stopped via /autonomy/stop.

State machine values exposed in snapshot()
-------------------------------------------
  starting       — just launched, not yet polled sensor
  moving         — path clear, driving forward
  slowing        — obstacle in warn zone, speed reduced
  pivoting       — obstacle in stop zone, rotating
  recovering     — brief counter-pivot after clearing obstacle
  sensor_offline — no valid ultrasonic reading for > SENSOR_TIMEOUT_S
  estop          — ESTOP_EVENT is set; motors halted, waiting for clear
  error          — unrecoverable hardware init failure
  idle           — mode stopped normally

Parameters (all optional, pass via POST /autonomy/start)
---------------------------------------------------------
  warn_cm         float  50     warn-zone inner edge (cm)
  stop_cm         float  25     hard-stop / pivot trigger (cm)
  forward_speed   float  0.5    forward drive speed [0-1]
  pivot_speed     float  0.6    rotation speed during pivot [0-1]
  recovery_ratio  float  0.30   fraction of pivot time used for counter-pivot
  recovery_speed  float  0.35   counter-pivot speed [0-1]
  poll_hz         float  10     ultrasonic poll rate (Hz)
  drive           bool   True   if False: react only, do not drive forward
  camera_assist   bool   False  enable camera snapshot secondary detection
  camera_url      str    ""     override snapshot URL (auto = http://127.0.0.1:5000/snapshot)
"""

from __future__ import annotations

import logging
import threading
import time
import urllib.request
from typing import Any, Mapping

from ..base import AutonomyModeHandler

log = logging.getLogger("autonomy.obstacle_avoidance")

# No valid ultrasonic reading for this long → sensor considered offline.
SENSOR_TIMEOUT_S = 0.5
# Polling rate while pivoting (independent of poll_hz so we check quickly).
_PIVOT_POLL_S    = 1.0 / 15.0   # ~15 Hz
# Maximum time to spend pivoting in one direction before reversing.
_PIVOT_TIMEOUT_S = 8.0


# ---------------------------------------------------------------------------
# Shared emergency-stop event
# ---------------------------------------------------------------------------
try:
    from movement.estop import ESTOP_EVENT as _ESTOP
except ImportError:
    # Fallback: private event so the module still loads outside the Pi env.
    _ESTOP = threading.Event()


class ObstacleAvoidanceMode(AutonomyModeHandler):
    """
    Sensor-driven forward navigation with ultrasonic + optional camera
    obstacle detection and automatic pivot-until-clear avoidance.
    """

    def __init__(self, *, logger=None) -> None:
        super().__init__("avoid_obstacles", logger=logger)

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._state = "idle"

        # Latest sensor reading (shared with snapshot())
        self._distance_cm: float | None = None
        self._pivot_count        = 0
        self._heading_offset_deg = 0.0   # +ve = turned left, -ve = right (approx)

        # Configurable (set in start())
        self._warn_cm        = 50.0
        self._stop_cm        = 25.0
        self._forward_speed  = 0.5
        self._pivot_speed    = 0.6
        self._recovery_ratio = 0.30
        self._recovery_speed = 0.35
        self._poll_hz        = 10.0
        self._drive          = True
        self._pivot_dir      = 1   # +1 = left, -1 = right; alternates each obstacle

        # Camera secondary sensor
        self._camera_assist   = False
        self._camera_url      = ""
        self._camera_obstacle = False      # flag updated by camera thread
        self._camera_stop     = threading.Event()
        self._camera_thread: threading.Thread | None = None

        # Cached bridge (created once at loop start)
        self._bridge = None

    # ------------------------------------------------------------------
    # AutonomyModeHandler interface
    # ------------------------------------------------------------------

    async def start(self, params: Mapping[str, Any]) -> None:
        self._warn_cm        = float(params.get("warn_cm",        50.0))
        self._stop_cm        = float(params.get("stop_cm",        25.0))
        self._forward_speed  = float(params.get("forward_speed",   0.5))
        self._pivot_speed    = float(params.get("pivot_speed",     0.6))
        self._recovery_ratio = float(params.get("recovery_ratio",  0.30))
        self._recovery_speed = float(params.get("recovery_speed",  0.35))
        self._poll_hz        = float(params.get("poll_hz",        10.0))
        self._drive          = bool(params.get("drive",            True))
        self._camera_assist  = bool(params.get("camera_assist",   False))
        self._camera_url     = str(params.get("camera_url",         ""))

        self._stop_event.clear()
        self._camera_stop.clear()
        self._state              = "starting"
        self._pivot_count        = 0
        self._heading_offset_deg = 0.0
        self._camera_obstacle    = False

        # Clear any previous e-stop so we start fresh.
        _ESTOP.clear()

        if self._camera_assist:
            self._camera_thread = threading.Thread(
                target=self._camera_loop, daemon=True, name="avoidance-camera",
            )
            self._camera_thread.start()

        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="obstacle-avoidance",
        )
        self._thread.start()
        self.logger.info(
            "Obstacle avoidance started — warn=%.0fcm stop=%.0fcm "
            "drive=%s camera=%s recovery=%.0f%%",
            self._warn_cm, self._stop_cm, self._drive,
            self._camera_assist, self._recovery_ratio * 100,
        )

    async def stop(self) -> None:
        self._stop_event.set()
        self._camera_stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        if self._camera_thread and self._camera_thread.is_alive():
            self._camera_thread.join(timeout=2.0)
        self._state = "idle"
        # Belt-and-suspenders: stop motors even if the loop already did it.
        if self._bridge:
            try:
                self._bridge.stop()
            except Exception:
                pass
        self.logger.info("Obstacle avoidance stopped")

    async def update(self, params: Mapping[str, Any]) -> None:
        """Live-update tuneable params while mode is running."""
        if "warn_cm"       in params: self._warn_cm        = float(params["warn_cm"])
        if "stop_cm"       in params: self._stop_cm        = float(params["stop_cm"])
        if "forward_speed" in params: self._forward_speed  = float(params["forward_speed"])
        if "pivot_speed"   in params: self._pivot_speed    = float(params["pivot_speed"])
        if "recovery_ratio"in params: self._recovery_ratio = float(params["recovery_ratio"])

    def snapshot(self) -> dict:
        return {
            "state":               self._state,
            "distance_cm":         self._distance_cm,
            "warn_cm":             self._warn_cm,
            "stop_cm":             self._stop_cm,
            "pivot_count":         self._pivot_count,
            "heading_offset_deg":  round(self._heading_offset_deg, 1),
            "camera_assist":       self._camera_assist,
            "camera_obstacle":     self._camera_obstacle,
        }

    # ------------------------------------------------------------------
    # Hardware helpers
    # ------------------------------------------------------------------

    def _acquire_bridge(self):
        """Create the motor bridge once and cache it for the loop lifetime."""
        try:
            from api.ros_bridge import OmegaRosBridge
            return OmegaRosBridge.create()
        except Exception as exc:
            self.logger.warning("Could not create motor bridge: %s", exc)
            return None

    def _get_distance_from_bridge(self) -> float | None:
        """Read the latest ultrasonic distance from the sensor bridge shared var."""
        try:
            from api.sensor_bridge import _bridge_latest_distance_cm, _bridge_latest_ultra_ts
            v  = _bridge_latest_distance_cm
            ts = _bridge_latest_ultra_ts
            if v is None or v <= 0:
                return None
            if ts > 0 and (time.monotonic() - ts) > SENSOR_TIMEOUT_S:
                return None   # stale
            return v
        except Exception:
            return None

    def _open_direct_sensor(self):
        try:
            from sensors.ultrasonic_sensor import Ultrasonic
            return Ultrasonic()
        except Exception as exc:
            self.logger.error("Failed to open ultrasonic sensor: %s", exc)
            return None

    # ------------------------------------------------------------------
    # Camera secondary sensor (background thread, ~2 FPS)
    # ------------------------------------------------------------------

    def _camera_loop(self) -> None:
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.logger.warning("cv2/numpy unavailable — camera assist disabled")
            self._camera_assist = False
            return

        url    = self._camera_url or "http://127.0.0.1:5000/snapshot?quality=40"
        period = 0.5   # 2 FPS is sufficient for secondary detection

        while not self._camera_stop.is_set() and not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                with urllib.request.urlopen(url, timeout=0.4) as resp:
                    raw = resp.read()
                arr   = np.frombuffer(raw, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
                if frame is not None:
                    self._camera_obstacle = self._frame_has_obstacle(frame, cv2)
            except Exception:
                pass   # camera offline — keep last value

            elapsed = time.monotonic() - t0
            self._camera_stop.wait(max(0.0, period - elapsed))

    @staticmethod
    def _frame_has_obstacle(gray, cv2) -> bool:
        """
        Returns True when the lower-centre ROI has high edge density —
        a heuristic for a close, textured obstacle filling the forward view.
        Threshold tuned for 640×480 JPEG@40%.
        """
        h, w = gray.shape
        roi  = gray[h // 2:, w // 4: 3 * w // 4]   # bottom-half, centre-half
        gx   = cv2.Sobel(roi, cv2.CV_32F, 1, 0, ksize=3)
        gy   = cv2.Sobel(roi, cv2.CV_32F, 0, 1, ksize=3)
        mag  = cv2.magnitude(gx, gy)
        # Normalise to [0,1] and compute mean density
        density = float(mag.mean()) / (mag.max() + 1e-6)
        return density > 0.28

    # ------------------------------------------------------------------
    # Main avoidance loop
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        # Acquire motor bridge once for the lifetime of this mode.
        self._bridge = self._acquire_bridge()
        if self._bridge is None:
            self.logger.warning("No motor bridge — avoidance running in log-only mode")

        # Prefer the sensor bridge shared reading (avoids opening GPIO twice).
        use_bridge_sensor = False
        direct_sensor     = None
        try:
            from api.sensor_bridge import _bridge_latest_distance_cm  # noqa: F401
            use_bridge_sensor = True
            self.logger.info("Avoidance: using sensor bridge shared reading")
        except ImportError:
            direct_sensor = self._open_direct_sensor()
            if direct_sensor is None:
                self.logger.error("No ultrasonic sensor available — aborting")
                self._state = "error"
                return
            self.logger.info("Avoidance: using direct sensor instance")

        period         = 1.0 / max(self._poll_hz, 1.0)
        last_valid_ts  = time.monotonic()
        self._state    = "moving"

        def get_dist() -> float | None:
            if use_bridge_sensor:
                return self._get_distance_from_bridge()
            try:
                return direct_sensor.get_distance()
            except Exception:
                return None

        self.logger.info(
            "Avoidance loop running at %.0f Hz (warn=%.0fcm stop=%.0fcm drive=%s)",
            self._poll_hz, self._warn_cm, self._stop_cm, self._drive,
        )

        try:
            while not self._stop_event.is_set():
                t0 = time.monotonic()

                # ── Emergency stop ─────────────────────────────────────
                if _ESTOP.is_set():
                    if self._state != "estop":
                        self.logger.warning("ESTOP received — halting autonomy loop")
                        self._state = "estop"
                    if self._bridge:
                        self._bridge.stop()
                    # Park here until cleared or stop_event fires.
                    self._stop_event.wait(timeout=0.1)
                    continue

                # ── Sensor read ────────────────────────────────────────
                dist = get_dist()
                if dist is not None and 2.0 <= dist <= 400.0:
                    last_valid_ts     = time.monotonic()
                    self._distance_cm = dist
                else:
                    dist = None

                sensor_age = time.monotonic() - last_valid_ts
                if sensor_age > SENSOR_TIMEOUT_S:
                    if self._state != "sensor_offline":
                        self.logger.warning(
                            "Ultrasonic offline (%.1fs stale) — halting", sensor_age,
                        )
                        self._state = "sensor_offline"
                    if self._bridge:
                        self._bridge.stop()
                    elapsed = time.monotonic() - t0
                    self._stop_event.wait(max(0.0, period - elapsed))
                    continue

                # ── Camera secondary signal ───────────────────────────
                # If camera detects something close, raise effective stop
                # threshold so the robot reacts earlier.
                effective_stop_cm = self._stop_cm
                if self._camera_obstacle:
                    effective_stop_cm = max(self._stop_cm, self._warn_cm * 0.70)

                # ── Decision ──────────────────────────────────────────
                if dist <= effective_stop_cm:
                    # Stop zone: pivot until fully clear
                    self._state = "pivoting"
                    self._pivot_until_clear(get_dist)

                elif dist <= self._warn_cm:
                    # Warn zone: proportional speed reduction
                    span     = max(self._warn_cm - effective_stop_cm, 1.0)
                    fraction = (dist - effective_stop_cm) / span
                    slow_spd = self._forward_speed * max(0.15, min(1.0, fraction))
                    if self._state != "slowing":
                        self.logger.info(
                            "Warn zone %.1fcm — slowing to %.0f%%",
                            dist, slow_spd * 100,
                        )
                        self._state = "slowing"
                    if self._drive and self._bridge:
                        self._bridge.send_command("forward", slow_spd)

                else:
                    # Path clear: full-speed forward
                    if self._state not in ("moving", "starting"):
                        self.logger.info("Path clear %.1fcm — resuming forward", dist)
                        self._state = "moving"
                    if self._drive and self._bridge:
                        self._bridge.send_command("forward", self._forward_speed)

                elapsed = time.monotonic() - t0
                self._stop_event.wait(max(0.0, period - elapsed))

        finally:
            if direct_sensor:
                try:
                    direct_sensor.close()
                except Exception:
                    pass
            if self._bridge:
                self._bridge.stop()
            self._state = "idle"

    # ------------------------------------------------------------------
    # Pivot-until-clear with heading recovery
    # ------------------------------------------------------------------

    def _pivot_until_clear(self, get_dist) -> None:
        """
        Phase 1 — rotate until dist > warn_cm (path fully clear).
        Phase 2 — brief counter-pivot recovery to restore approx heading.

        Direction alternates each time an obstacle is encountered so the
        robot doesn't spiral in one direction across multiple obstacles.
        """
        bridge = self._bridge

        pivot_cmd    = "pivot_left"  if self._pivot_dir > 0 else "pivot_right"
        recover_cmd  = "pivot_right" if self._pivot_dir > 0 else "pivot_left"

        self.logger.info(
            "Pivoting %s until clear (warn=%.0fcm, timeout=%.0fs)",
            pivot_cmd, self._warn_cm, _PIVOT_TIMEOUT_S,
        )

        pivot_start = time.monotonic()

        # ── Phase 1: rotate until clear ──────────────────────────────
        while not self._stop_event.is_set() and not _ESTOP.is_set():
            if bridge:
                bridge.send_command(pivot_cmd, self._pivot_speed)

            self._stop_event.wait(timeout=_PIVOT_POLL_S)

            dist = get_dist()
            elapsed = time.monotonic() - pivot_start

            # Clear: full warn zone is open
            if dist is not None and dist > self._warn_cm:
                if bridge:
                    bridge.stop()
                self.logger.info(
                    "Obstacle cleared after %.1fs pivot (%s)", elapsed, pivot_cmd,
                )
                break

            # Safety timeout: reverse pivot direction for next attempt
            if elapsed >= _PIVOT_TIMEOUT_S:
                self.logger.warning(
                    "Pivot timeout %.0fs — flipping direction", _PIVOT_TIMEOUT_S,
                )
                if bridge:
                    bridge.stop()
                break
        else:
            # stop_event or estop fired during pivot
            if bridge:
                bridge.stop()
            return

        pivot_elapsed = time.monotonic() - pivot_start
        self._pivot_count += 1

        # Track approximate heading (90°/s assumed pivot rate)
        deg_turned = pivot_elapsed * 90.0 * self._pivot_dir
        self._heading_offset_deg += deg_turned

        self.logger.info(
            "Pivot done in %.1fs (~%.0f° offset total %.1f°)",
            pivot_elapsed, abs(deg_turned), self._heading_offset_deg,
        )

        # Flip direction so next obstacle is handled from the other side.
        self._pivot_dir *= -1

        # ── Phase 2: recovery counter-pivot ──────────────────────────
        # Rotate back a fraction of the pivot time to restore heading.
        if self._recovery_ratio > 0 and pivot_elapsed > 0.15:
            recovery_t = pivot_elapsed * self._recovery_ratio
            self._state = "recovering"
            self.logger.info(
                "Recovery: %s for %.1fs @ %.0f%%",
                recover_cmd, recovery_t, self._recovery_speed * 100,
            )
            deadline = time.monotonic() + recovery_t
            while time.monotonic() < deadline:
                if self._stop_event.is_set() or _ESTOP.is_set():
                    break
                if bridge:
                    bridge.send_command(recover_cmd, self._recovery_speed)
                self._stop_event.wait(timeout=0.05)

            if bridge:
                bridge.stop()

            # Update heading offset for the counter-rotation
            recover_dir              = -self._pivot_dir   # opposite of what we just flipped to
            self._heading_offset_deg += recovery_t * 90.0 * recover_dir

        # Brief pause so the robot settles before the next sensor read
        self._stop_event.wait(timeout=0.12)
