"""ArUco seek-and-approach autonomy mode.

State machine
-------------
  search   — rotate slowly until a marker enters frame
  align    — rotate in place until |bearing| < align_thresh_rad
  approach — drive forward, re-align if bearing drifts; stop at stop_dist_m
  stopped  — halted at target; mode exits cleanly

Motor commands use the same discrete bridge interface as obstacle_avoidance:
  forward, pivot_left, pivot_right (+ stop()).

Distance / bearing computation
-------------------------------
  If pose estimation is configured (ARUCO_MARKER_LENGTH + ARUCO_CALIBRATION_FILE):
    - distance = tvec[2]  (metres, camera Z-axis)
    - bearing  = atan2(-tvec[0], tvec[2])   (+ = marker is LEFT)

  Uncalibrated fallback (marker_length env not set):
    - distance ≈ (marker_length_m × focal_px) / corner_pixel_width
    - bearing  ≈ atan2(-(cx_pixel - frame_cx), focal_px)
    Accurate to ±20 % for flat-on markers at < 2 m — sufficient for demo docking.

Start via:
    POST /autonomy/start  {"mode": "aruco_seek", "stop_dist": 0.4, "target_id": 0}
"""

from __future__ import annotations

import logging
import math
import threading
import time
import urllib.request
from typing import Any, Mapping, Optional, Tuple

import numpy as np

from ..base import AutonomyModeHandler

log = logging.getLogger("autonomy.aruco_seek")

try:
    from movement.estop import ESTOP_EVENT as _ESTOP
except ImportError:
    _ESTOP = threading.Event()

# ---------------------------------------------------------------------------
# Module-level detector singleton — created once, reused across mode restarts.
# ---------------------------------------------------------------------------
_det_lock = threading.Lock()
_det_instance = None


def _get_detector(marker_length: float):
    global _det_instance
    with _det_lock:
        if _det_instance is None:
            try:
                from video.aruco_detection import ArucoDetector
                _det_instance = ArucoDetector(marker_length=marker_length)
                log.info(
                    "ArucoDetector ready (marker_length=%.3fm pose_est=%s)",
                    marker_length,
                    _det_instance._camera_matrix is not None,
                )
            except Exception as exc:
                log.warning("ArucoDetector unavailable: %s", exc)
        return _det_instance


def _pick_best(detections, target_id: int):
    """Return the detection closest to camera, filtered by target_id if set."""
    if target_id >= 0:
        detections = [d for d in detections if d.marker_id == target_id]
    if not detections:
        return None

    def _depth_key(d):
        if d.tvec is not None:
            return float(d.tvec[2])
        c = np.array(d.corners, dtype=float)
        w = float(np.linalg.norm(c[1] - c[0]))
        return 1e6 / max(w, 1.0)   # larger pixel width → smaller key → closer

    return min(detections, key=_depth_key)


# ---------------------------------------------------------------------------
class ArucoSeekMode(AutonomyModeHandler):
    """Detect an ArUco marker, align to it, approach, and stop at stop_dist."""

    _SNAP_HZ  = 8.0     # Hz — snapshot poll rate (video server is ~30 fps; 8 is plenty)
    _FOCAL_PX = 500.0   # px — approximate focal length for uncalibrated fallback

    # Default tuning — all overridable via POST /autonomy/start params
    _D_STOP_DIST     = 0.40   # m — stop this far from the marker face
    _D_ALIGN_THRESH  = 0.12   # rad (~7°) — "aligned enough" to begin approach
    _D_FORWARD_SPEED = 0.35   # [0–1] — forward drive speed during approach
    _D_SEARCH_SPEED  = 0.40   # [0–1] — rotation speed while searching / aligning
    _D_STEER_KP      = 1.20   # P-gain: bearing_rad → normalised omega
    _D_MARKER_LEN    = 0.10   # m — physical side length of the target marker
    _D_LOST_TIMEOUT  = 1.00   # s — return to search if marker absent this long

    def __init__(self, *, logger=None) -> None:
        super().__init__("aruco_seek", logger=logger)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._bridge = None
        self._state  = "idle"

        # Telemetry exposed via snapshot()
        self._marker_id:   Optional[int]   = None
        self._distance_m:  Optional[float] = None
        self._bearing_deg: Optional[float] = None

        # Runtime config (filled in start())
        self._stop_dist     = self._D_STOP_DIST
        self._align_thresh  = self._D_ALIGN_THRESH
        self._forward_speed = self._D_FORWARD_SPEED
        self._search_speed  = self._D_SEARCH_SPEED
        self._steer_kp      = self._D_STEER_KP
        self._marker_length = self._D_MARKER_LEN
        self._lost_timeout  = self._D_LOST_TIMEOUT
        self._target_id     = -1
        self._snap_url      = "http://127.0.0.1:5000/snapshot?quality=60"

    # ------------------------------------------------------------------
    # AutonomyModeHandler interface
    # ------------------------------------------------------------------

    async def start(self, params: Mapping[str, Any]) -> None:
        self._stop_dist     = float(params.get("stop_dist",     self._D_STOP_DIST))
        self._align_thresh  = float(params.get("align_thresh",  self._D_ALIGN_THRESH))
        self._forward_speed = float(params.get("forward_speed", self._D_FORWARD_SPEED))
        self._search_speed  = float(params.get("search_speed",  self._D_SEARCH_SPEED))
        self._steer_kp      = float(params.get("steer_kp",      self._D_STEER_KP))
        self._marker_length = float(params.get("marker_length", self._D_MARKER_LEN))
        self._lost_timeout  = float(params.get("lost_timeout",  self._D_LOST_TIMEOUT))
        self._target_id     = int(params.get("target_id",      -1))
        self._snap_url      = str(params.get("snapshot_url",    self._snap_url))

        self._stop_event.clear()
        self._state = "search"
        _ESTOP.clear()

        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="aruco-seek",
        )
        self._thread.start()
        self.logger.info(
            "ArUco seek started — target=%s stop_dist=%.2fm align_thresh=%.0f°",
            self._target_id if self._target_id >= 0 else "any",
            self._stop_dist,
            math.degrees(self._align_thresh),
        )

    async def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        if self._bridge:
            try:
                self._bridge.stop()
            except Exception:
                pass
        self._state = "idle"
        self.logger.info("ArUco seek stopped")

    def snapshot(self) -> dict:
        return {
            "state":       self._state,
            "marker_id":   self._marker_id,
            "distance_m":  round(self._distance_m, 3) if self._distance_m is not None else None,
            "bearing_deg": round(self._bearing_deg, 1) if self._bearing_deg is not None else None,
            "stop_dist_m": self._stop_dist,
        }

    # ------------------------------------------------------------------
    # Hardware helpers
    # ------------------------------------------------------------------

    def _acquire_bridge(self):
        try:
            from api.ros_bridge import OmegaRosBridge
            return OmegaRosBridge.create()
        except Exception as exc:
            self.logger.warning("Motor bridge unavailable: %s", exc)
            return None

    # ------------------------------------------------------------------
    # Vision helpers
    # ------------------------------------------------------------------

    def _fetch_frame(self):
        """Pull one JPEG from the video server snapshot endpoint."""
        try:
            import cv2
            with urllib.request.urlopen(self._snap_url, timeout=0.8) as r:
                raw = r.read()
            arr = np.frombuffer(raw, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            return None

    def _detect(self, frame) -> Optional[Tuple[int, float, float]]:
        """
        Returns (marker_id, distance_m, bearing_rad) or None.

        bearing_rad sign convention:
            > 0  → marker is to the LEFT  → robot should pivot CCW (pivot_left)
            < 0  → marker is to the RIGHT → robot should pivot CW  (pivot_right)
        """
        detector = _get_detector(self._marker_length)
        if detector is None or not detector.active:
            return None

        _, detections = detector.annotate(frame)
        if not detections:
            return None

        best = _pick_best(detections, self._target_id)
        if best is None:
            return None

        h, w = frame.shape[:2]

        if best.tvec is not None:
            # ── Calibrated path ──────────────────────────────────────
            tx, _ty, tz = best.tvec
            dist    = float(max(tz, 0.01))
            bearing = float(math.atan2(-tx, tz))   # - because camera x+ is right
        else:
            # ── Uncalibrated fallback ─────────────────────────────────
            corners = np.array(best.corners, dtype=float)
            px_w    = float(np.linalg.norm(corners[1] - corners[0]))
            if px_w < 4.0:
                return None
            dist    = (self._marker_length * self._FOCAL_PX) / px_w
            cx_off  = best.centre[0] - w / 2.0
            bearing = float(math.atan2(-cx_off, self._FOCAL_PX))

        return best.marker_id, dist, bearing

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        self._bridge = self._acquire_bridge()
        if self._bridge is None:
            self.logger.warning("Running in log-only mode (no motor bridge)")

        period      = 1.0 / self._SNAP_HZ
        last_det_ts = 0.0

        try:
            while not self._stop_event.is_set():
                t0 = time.monotonic()

                # Emergency stop takes priority over everything
                if _ESTOP.is_set():
                    if self._state != "estop":
                        self._state = "estop"
                        self.logger.warning("ESTOP — halting")
                    if self._bridge:
                        self._bridge.stop()
                    self._stop_event.wait(0.1)
                    continue

                frame = self._fetch_frame()
                if frame is None:
                    self._stop_event.wait(0.5)
                    continue

                result = self._detect(frame)

                if result is not None:
                    mid, dist, bearing = result
                    self._marker_id   = mid
                    self._distance_m  = dist
                    self._bearing_deg = math.degrees(bearing)
                    last_det_ts       = time.monotonic()
                    self._step(dist, bearing)
                else:
                    self._marker_id   = None
                    self._distance_m  = None
                    self._bearing_deg = None
                    lost_for = time.monotonic() - last_det_ts
                    if lost_for > self._lost_timeout:
                        if self._state not in ("search", "idle"):
                            self.logger.info("Marker lost (%.1fs) — SEARCH", lost_for)
                        self._state = "search"
                        if self._bridge:
                            self._bridge.send_command("pivot_left", self._search_speed)

                elapsed = time.monotonic() - t0
                self._stop_event.wait(max(0.0, period - elapsed))

        finally:
            if self._bridge:
                self._bridge.stop()
            self._state = "idle"

    def _step(self, dist: float, bearing: float) -> None:
        """One control tick given a valid detection."""
        b   = self._bridge
        babs = abs(bearing)

        # ── STOP ────────────────────────────────────────────────────────
        if dist <= self._stop_dist:
            if self._state != "stopped":
                self.logger.info(
                    "STOP — dist=%.2fm bearing=%.1f° marker=%s",
                    dist, math.degrees(bearing), self._marker_id,
                )
                self._state = "stopped"
                if b:
                    b.stop()
                self._stop_event.set()
            return

        # ── ALIGN ────────────────────────────────────────────────────────
        if babs > self._align_thresh:
            self._state = "align"
            omega = self._steer_kp * bearing              # signed [-1, 1]
            spd   = min(abs(omega), self._search_speed)   # never faster than search_speed
            cmd   = "pivot_left" if omega > 0 else "pivot_right"
            if b:
                b.send_command(cmd, spd)
            self.logger.debug("ALIGN bearing=%.1f° → %s spd=%.2f", math.degrees(bearing), cmd, spd)
            return

        # ── APPROACH ─────────────────────────────────────────────────────
        self._state = "approach"
        if babs < self._align_thresh * 0.4:
            # Well-centred: drive straight
            if b:
                b.send_command("forward", self._forward_speed)
        else:
            # Minor drift: brief corrective pivot at half forward speed
            spd = self._forward_speed * 0.5
            cmd = "pivot_left" if bearing > 0 else "pivot_right"
            if b:
                b.send_command(cmd, spd)
        self.logger.debug("APPROACH dist=%.2fm bearing=%.1f°", dist, math.degrees(bearing))
