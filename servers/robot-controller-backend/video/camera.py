# File: /Omega-Code/servers/robot-controller-backend/video/camera.py
"""
Summary
-------
Unified camera facade for Raspberry Pi robots.

- Prefers Picamera2 (libcamera) for CSI ribbon cameras on Bookworm
- Falls back to OpenCV/V4L2 for USB webcams (/dev/video*)
- Background capture thread; returns latest frame as 3-channel BGR numpy array
- Warm-up checks (fail-fast if no frames), smoothed FPS, throttled warnings
- Drop-in for video/video_server.py (motion detection / tracking use BGR frames)

Environment (optional)
----------------------
CAMERA_BACKEND = auto | picamera2 | v4l2   (default: auto)
CAMERA_DEVICE  = V4L2 path/index (for v4l2 backend; default: /dev/video0)
CAMERA_WIDTH   = 640
CAMERA_HEIGHT  = 480
FRAME_STALE_MS = 2500   # used by is_alive() default
"""

from __future__ import annotations

import os
import time
import threading
import logging
import warnings
from typing import Optional

import numpy as np

# --- OpenCV (used by V4L2 and for RGB->BGR conversion in Picamera2 path) ---
try:
    import cv2  # type: ignore
except Exception as e:  # pragma: no cover
    cv2 = None  # type: ignore
    warnings.warn(
        f"OpenCV not available ({e}). V4L2 capture and JPEG encoding require it.",
        ImportWarning,
    )

# --- Picamera2 (preferred CSI ribbon backend) ---
try:
    from picamera2 import Picamera2  # type: ignore
    _PICAM2_OK = True
except Exception:
    _PICAM2_OK = False

log = logging.getLogger(__name__)


# =========================
# Backend: Picamera2 (CSI)
# =========================
class _PiCam2Backend:
    """
    Threaded Picamera2 reader. Captures RGB888 and converts to BGR for OpenCV consumers.
    """

    def __init__(
        self,
        width: int,
        height: int,
        target_fps: int = 30,
        warmup_timeout_s: float = 1.2,
        warmup_min_frames: int = 2,
    ) -> None:
        if not _PICAM2_OK:
            raise RuntimeError("Picamera2 is not installed")

        self.width = int(width)
        self.height = int(height)
        self.target_fps = max(1, int(target_fps))

        self.picam: Optional[Picamera2] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._frame: Optional[np.ndarray] = None  # BGR
        self._last_frame_ts: float = 0.0
        self._fps: float = 0.0
        self._frames_total = 0
        self._drops_total = 0

        # Configure Picamera2 for RGB888 at requested size. Some control names differ
        # across versions; wrap in a try so we always configure something workable.
        self.picam = Picamera2()
        try:
            cfg = self.picam.create_video_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"},
                controls={"FrameDurationLimits": (int(1e9 / self.target_fps), int(1e9 / self.target_fps))},
            )
        except Exception:
            cfg = self.picam.create_video_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
        self.picam.configure(cfg)
        self.picam.start()

        # Start capture thread
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="PiCam2Capture", daemon=True)
        self._thread.start()

        # Warm-up: require N frames within timeout
        if not self._wait_for_frames(warmup_timeout_s, warmup_min_frames):
            self.stop()
            raise RuntimeError("❌ Picamera2 did not deliver frames in time")

        log.info("📷 Camera backend: picamera2 (%dx%d@~%dfps)", self.width, self.height, self.target_fps)

    def _wait_for_frames(self, timeout_s: float, need: int) -> bool:
        got = 0
        t0 = time.time()
        while time.time() - t0 < timeout_s and got < need:
            if self._last_frame_ts > 0:
                got += 1
            time.sleep(0.03)
        return got >= need

    def _loop(self) -> None:
        assert self.picam is not None
        last_fps_tick = time.time()
        frames_in_win = 0

        while self._running:
            try:
                rgb = self.picam.capture_array("main")  # RGB888
                if rgb is None:
                    self._drops_total += 1
                    time.sleep(0.01)
                    continue

                # Convert to BGR for downstream OpenCV code (if OpenCV is available)
                frame = (
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if cv2 is not None else rgb  # type: ignore[assignment]
                )

                with self._lock:
                    self._frame = frame
                    self._last_frame_ts = time.time()
                    self._frames_total += 1

                frames_in_win += 1
                now = time.time()
                if now - last_fps_tick >= 1.0:
                    dt = now - last_fps_tick
                    self._fps = frames_in_win / dt if dt > 0 else 0.0
                    frames_in_win = 0
                    last_fps_tick = now

            except Exception as e:
                self._drops_total += 1
                log.warning("Picamera2 capture error: %s", e)
                time.sleep(0.03)

    # -- API surface used by the facade --
    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def stop(self) -> None:
        self._running = False
        try:
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=0.8)
        except Exception:
            pass
        try:
            if self.picam:
                try:
                    self.picam.stop()
                finally:
                    self.picam.close()
        except Exception:
            pass
        self.picam = None

    # Metrics
    @property
    def last_frame_ts(self) -> float: return self._last_frame_ts
    @property
    def fps(self) -> float: return self._fps
    @property
    def frames_total(self) -> int: return self._frames_total
    @property
    def drops_total(self) -> int: return self._drops_total


# ==========================
# Backend: OpenCV / V4L2
# ==========================
class _V4L2Backend:
    """
    Threaded OpenCV VideoCapture reader. Works with USB webcams (/dev/videoN).
    """

    def __init__(
        self,
        device: str | int,
        width: int,
        height: int,
        *,
        target_fps: int = 30,
        fourcc: str = "MJPG",
        warmup_timeout_s: float = 1.0,
        warmup_min_frames: int = 2,
        warn_every_s: float = 2.0,
    ) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV not available for V4L2 backend")

        self.device = device
        self.width = int(width)
        self.height = int(height)
        self._target_fps = max(1, int(target_fps))
        self._warn_every_s = max(0.2, float(warn_every_s))
        self._last_warn_ts = 0.0

        self._cap: Optional["cv2.VideoCapture"] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._frame: Optional[np.ndarray] = None  # BGR
        self._last_frame_ts: float = 0.0
        self._fps: float = 0.0
        self._frames_total = 0
        self._drops_total = 0

        # Open V4L2 device (with Linux CAP_V4L2 if available)
        flags = cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") else 0
        cap = cv2.VideoCapture(self.device, flags)
        if not cap or not cap.isOpened():
            raise RuntimeError(f"❌ Could not open V4L2 device: {self.device}")

        # Best-effort hints
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, float(self._target_fps))
            if fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        except Exception:
            pass

        self._cap = cap
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="V4L2Capture", daemon=True)
        self._thread.start()

        if not self._wait_for_frames(warmup_timeout_s, warmup_min_frames):
            self.stop()
            raise RuntimeError(f"❌ Could not open camera at {self.device} (no frames)")

        log.info("📷 Camera backend: v4l2 %s (%dx%d@~%dfps)", self.device, self.width, self.height, self._target_fps)

    def _wait_for_frames(self, timeout_s: float, need: int) -> bool:
        got = 0
        t0 = time.time()
        while time.time() - t0 < timeout_s and got < need:
            if self._last_frame_ts > 0.0:
                got += 1
            time.sleep(0.03)
        return got >= need

    def _loop(self) -> None:
        assert self._cap is not None
        last_tick = time.time()
        frames_in_win = 0

        while self._running:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                self._drops_total += 1
                now = time.time()
                if now - self._last_warn_ts >= self._warn_every_s:
                    log.warning("⚠️ Warning: Failed to capture frame.")
                    self._last_warn_ts = now
                time.sleep(0.02)
                continue

            # Normalize to 3-channel BGR
            try:
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                elif frame.shape[-1] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            except Exception:
                pass

            with self._lock:
                self._frame = frame
                self._last_frame_ts = time.time()
                self._frames_total += 1

            frames_in_win += 1
            now = time.time()
            if now - last_tick >= 1.0:
                dt = now - last_tick
                self._fps = frames_in_win / dt if dt > 0 else 0.0
                frames_in_win = 0
                last_tick = now

            # Light pacing; many drivers already throttle appropriately
            time.sleep(1.0 / (self._target_fps * 3.0))

    def stop(self) -> None:
        self._running = False
        try:
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=0.8)
        except Exception:
            pass
        try:
            if self._cap:
                self._cap.release()
        except Exception:
            pass
        self._cap = None

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    # Metrics
    @property
    def last_frame_ts(self) -> float: return self._last_frame_ts
    @property
    def fps(self) -> float: return self._fps
    @property
    def frames_total(self) -> int: return self._frames_total
    @property
    def drops_total(self) -> int: return self._drops_total


# ==========================
# Public facade
# ==========================
class Camera:
    """
    Public facade that selects a backend.

    Selection order:
      • CAMERA_BACKEND=picamera2 -> Picamera2
      • CAMERA_BACKEND=v4l2      -> OpenCV/V4L2
      • CAMERA_BACKEND=auto (default):
            try Picamera2 (if installed), then V4L2 (/dev/video0)
    """

    def __init__(
        self,
        device: str | int = "/dev/video0",
        width: int = int(os.getenv("CAMERA_WIDTH", "640")),
        height: int = int(os.getenv("CAMERA_HEIGHT", "480")),
        target_fps: int = 30,
    ) -> None:
        self.width = width
        self.height = height
        self.target_fps = target_fps
        self.backend_name: Optional[str] = None

        want = (os.getenv("CAMERA_BACKEND") or "auto").strip().lower()
        dev = os.getenv("CAMERA_DEVICE", str(device))

        self._backend: Optional[_PiCam2Backend | _V4L2Backend] = None
        last_err: Optional[Exception] = None

        def try_picam() -> _PiCam2Backend:
            return _PiCam2Backend(width=self.width, height=self.height, target_fps=self.target_fps)

        def try_v4l2() -> _V4L2Backend:
            return _V4L2Backend(device=dev, width=self.width, height=self.height, target_fps=self.target_fps)

        attempts: list[tuple[str, callable]] = []
        if want in ("picamera2", "picam2"):
            attempts = [("picamera2", try_picam)]
        elif want == "v4l2":
            attempts = [("v4l2", try_v4l2)]
        else:  # auto
            if _PICAM2_OK:
                attempts.append(("picamera2", try_picam))
            if cv2 is not None:
                attempts.append(("v4l2", try_v4l2))

        for name, ctor in attempts:
            try:
                self._backend = ctor()
                self.backend_name = name
                break
            except Exception as e:
                last_err = e
                log.warning("Camera backend '%s' failed: %s", name, e)

        if self._backend is None:
            raise RuntimeError(f"No camera backend available (last error: {last_err})")

    # ---- public API used by video_server.py ----
    def get_frame(self) -> Optional[np.ndarray]:
        return None if self._backend is None else self._backend.get_frame()

    def capture_image(self, filename: str = "snapshot.jpg") -> Optional[str]:
        frame = self.get_frame()
        if frame is None or cv2 is None:
            return None
        try:
            if cv2.imwrite(filename, frame):
                return filename
        except Exception:
            pass
        return None

    def is_alive(self, stale_ms: Optional[int] = None) -> bool:
        """True if last frame is newer than stale_ms (default from env FRAME_STALE_MS or 2500)."""
        if self._backend is None:
            return False
        default_ms = int(os.getenv("FRAME_STALE_MS", "2500"))
        limit = stale_ms if stale_ms is not None else default_ms
        last_ts = getattr(self._backend, "last_frame_ts", 0.0)
        return (time.time() - last_ts) * 1000.0 < float(limit)

    @property
    def fps(self) -> float:
        return 0.0 if self._backend is None else getattr(self._backend, "fps", 0.0)

    @property
    def frames_total(self) -> int:
        return 0 if self._backend is None else getattr(self._backend, "frames_total", 0)

    @property
    def drops_total(self) -> int:
        return 0 if self._backend is None else getattr(self._backend, "drops_total", 0)

    @property
    def backend(self) -> str:
        return self.backend_name or "unknown"

    def stop(self) -> None:
        try:
            self._backend and self._backend.stop()
        finally:
            self._backend = None
            log.info("Camera stopped.")
