# File: /Omega-Code/servers/robot-controller-backend/video/camera.py
"""
Summary
-------
OpenCV VideoCapture wrapper for Raspberry Pi that:
  • Opens the camera in MJPEG mode with desired width/height/FPS
  • Warms up and verifies real frames before declaring success (fail-fast)
  • Captures frames on a background thread (non-blocking Flask)
  • Guarantees 3-channel BGR frames (converts grayscale/BGRA if needed)
  • Throttles "failed to capture" warnings
  • Tracks rough FPS and last-good-frame timestamps for health checks

Used by: video/video_server.py (motion detection / tracking layers build on this)
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import warnings
try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cv2 = None  # type: ignore
    warnings.warn("OpenCV not installed. Camera features will be disabled.", ImportWarning)


class Camera:
    """
    Threaded camera reader with fail-fast warm-up and throttled warnings.

    Parameters
    ----------
    device : str | int
        V4L2 device path or index (default: "/dev/video0")
    width, height : int
        Requested frame size (best-effort)
    target_fps : int
        Desired capture FPS hint (not guaranteed by all drivers)
    prefer_v4l2 : bool
        Use CAP_V4L2 if available (Linux)
    fourcc : str
        Preferred pixel format (default "MJPG")
    warmup_timeout_s : float
        Max time to wait for a couple of valid frames before failing
    warmup_min_frames : int
        Number of good frames required to consider the camera "alive"
    warn_every_s : float
        Throttle interval for dropped-frame warnings
    """

    def __init__(
        self,
        device: str | int = "/dev/video0",
        width: int = 640,
        height: int = 480,
        *,
        target_fps: int = 30,
        prefer_v4l2: bool = True,
        fourcc: str = "MJPG",
        warmup_timeout_s: float = 0.8,
        warmup_min_frames: int = 2,
        warn_every_s: float = 2.0,
    ) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV not available")

        self.device = device
        self.width = width
        self.height = height
        self._target_fps = max(1, int(target_fps))
        self._warn_every_s = max(0.2, float(warn_every_s))
        self._last_warn_ts = 0.0

        # Metrics / state
        self._frame: Optional["cv2.Mat"] = None
        self._last_frame_ts: float = 0.0
        self._fps: float = 0.0
        self._frames_total: int = 0
        self._drops_total: int = 0
        self._running = False
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

        # ---------- Open the capture ----------
        cap_flags = cv2.CAP_V4L2 if prefer_v4l2 and hasattr(cv2, "CAP_V4L2") else 0
        cap = cv2.VideoCapture(self.device, cap_flags)
        if not cap or not cap.isOpened():
            raise RuntimeError(f"❌ Error: Could not open camera at {self.device}")

        # Hints (best-effort; some backends ignore them)
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  int(self.width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.height))
            cap.set(cv2.CAP_PROP_FPS, float(self._target_fps))
            if fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        except Exception:
            pass

        self._cap = cap

        # ---------- Start capture thread ----------
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, name="CameraCapture", daemon=True)
        self._thread.start()

        # ---------- Warm up: require N good frames quickly ----------
        ok_count = 0
        t0 = time.time()
        while time.time() - t0 < warmup_timeout_s and ok_count < warmup_min_frames:
            if self._last_frame_ts > 0.0:
                ok_count += 1
            time.sleep(0.03)

        if ok_count < warmup_min_frames:
            self.stop()
            raise RuntimeError(f"❌ Error: Could not open camera at {self.device} (no frames)")

    # -------------------------- Public API --------------------------

    def get_frame(self) -> Optional["cv2.Mat"]:
        """Return the latest frame (copy) or None if not available."""
        if cv2 is None or not self._running:
            return None
        with self._lock:
            if self._frame is None:
                return None
            # Return a copy so callers can draw/encode safely
            return self._frame.copy()

    def capture_image(self, filename: str = "snapshot.jpg") -> Optional[str]:
        """Save a single image from the last frame (returns filename or None)."""
        frame = self.get_frame()
        if frame is None:
            return None
        try:
            ok = cv2.imwrite(filename, frame)
            return filename if ok else None
        except Exception:
            return None

    def is_alive(self, stale_ms: int = 2500) -> bool:
        """True if we have seen a frame more recently than `stale_ms`."""
        return (time.time() - self._last_frame_ts) * 1000.0 < stale_ms

    @property
    def fps(self) -> float:
        """Smoothed FPS estimate from the capture loop."""
        return self._fps

    @property
    def frames_total(self) -> int:
        return self._frames_total

    @property
    def drops_total(self) -> int:
        return self._drops_total

    def stop(self) -> None:
        """Stop background thread and release device."""
        self._running = False
        try:
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if hasattr(self, "_cap") and self._cap is not None:
                self._cap.release()
        finally:
            self._cap = None
        logging.info("✅ Camera stopped and released.")

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass

    # ------------------------- Internal loop -------------------------

    def _capture_loop(self) -> None:
        """Continuously grab frames; compute FPS; throttle warnings on failure."""
        cap = self._cap
        if cap is None:
            return

        next_sleep = 1.0 / float(self._target_fps)
        last_fps_tick = time.time()
        frames_in_window = 0

        while self._running:
            ok, frame = cap.read()

            if not ok or frame is None:
                self._drops_total += 1
                now = time.time()
                if now - self._last_warn_ts >= self._warn_every_s:
                    logging.warning("⚠️ Warning: Failed to capture frame.")
                    self._last_warn_ts = now
                time.sleep(min(0.02, next_sleep))  # small backoff on failure
                continue

            # Normalize to 3-channel BGR
            try:
                if len(frame.shape) == 2:  # grayscale → BGR
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                elif frame.shape[-1] == 4:  # BGRA → BGR
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            except Exception:
                # If conversion fails, keep original
                pass

            with self._lock:
                self._frame = frame
                self._last_frame_ts = time.time()
                self._frames_total += 1

            # FPS calc (once per ~1s window)
            frames_in_window += 1
            now = time.time()
            if now - last_fps_tick >= 1.0:
                dt = now - last_fps_tick
                self._fps = frames_in_window / dt if dt > 0 else 0.0
                frames_in_window = 0
                last_fps_tick = now

            # Light pacing to avoid hot loop if driver floods frames
            if next_sleep > 0:
                time.sleep(max(0.0, next_sleep * 0.3))
