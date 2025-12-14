# File: /Omega-Code/servers/robot_controller_backend/video/camera.py
"""
Picamera2-only camera backend for Raspberry Pi CSI cameras.

- Uses Picamera2 (libcamera) exclusively for CSI ribbon cameras
- Background capture thread; returns latest frame as 3-channel BGR numpy array
- Warm-up checks (fail-fast if no frames), smoothed FPS, throttled warnings
- Drop-in for video/video_server.py (motion detection / tracking use BGR frames)

Environment (optional)
----------------------
CAMERA_WIDTH        = 640
CAMERA_HEIGHT       = 480
CAMERA_FPS          = 30
CAMERA_WARMUP_MS    = 3000  (default)
CAMERA_WARMUP_FRAMES= 2
FRAME_STALE_MS      = 2500  (used by is_alive())
"""

from __future__ import annotations

import os
import atexit
import time
import threading
import logging
from typing import Optional

import numpy as np

# Hardware detection for optimizations
try:
    import psutil
except ImportError:
    psutil = None

# --- OpenCV (for RGB->BGR conversion only) ---
try:
    import cv2  # type: ignore
except Exception:
    cv2 = None

# --- Picamera2 (required CSI ribbon backend) ---
try:
    from picamera2 import Picamera2  # type: ignore
    _PICAM2_OK = True
except ImportError:
    _PICAM2_OK = False
    raise ImportError(
        "Picamera2 is required but not installed. "
        "Install with: sudo apt install -y python3-picamera2"
    )

log = logging.getLogger(__name__)


def _detect_raspberry_pi() -> bool:
    """Detect if running on Raspberry Pi hardware."""
    try:
        if os.path.exists("/proc/device-tree/model"):
            with open("/proc/device-tree/model", "r") as f:
                model = f.read().strip()
                if "Raspberry Pi" in model:
                    return True
    except Exception:
        pass
    
    try:
        if os.path.exists("/proc/cpuinfo"):
            with open("/proc/cpuinfo", "r") as f:
                cpuinfo = f.read()
                if any(cpu_id in cpuinfo for cpu_id in ["BCM2711", "BCM2835", "BCM2836", "BCM2837"]):
                    return True
    except Exception:
        pass
    
    return False


def _env_int(name: str, default: int) -> int:
    try:
        v = os.getenv(name)
        return int(v) if v and v.strip() else default
    except Exception:
        return default


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
        warmup_timeout_s: float = 3.0,
        warmup_min_frames: int = 2,
    ) -> None:
        if not _PICAM2_OK:
            raise RuntimeError("Picamera2 is not installed")

        # Allow env overrides
        warmup_timeout_s = max(
            warmup_timeout_s,
            _env_int("CAMERA_WARMUP_MS", int(warmup_timeout_s * 1000)) / 1000.0,
        )
        warmup_min_frames = max(1, _env_int("CAMERA_WARMUP_FRAMES", warmup_min_frames))

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

        # Configure Picamera2 for RGB888 at requested size
        self.picam = Picamera2()
        try:
            cfg = self.picam.create_video_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
        except Exception:
            # Fallback to default size if requested size fails
            cfg = self.picam.create_video_configuration(
                main={"size": (640, 480), "format": "RGB888"}
            )
        self.picam.configure(cfg)
        self.picam.start()
        
        # Hardware settle time
        time.sleep(0.5)

        # Start capture thread
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="PiCam2Capture", daemon=True)
        self._thread.start()

        # Warm-up: require new frames within timeout
        if not self._wait_for_frames(warmup_timeout_s, warmup_min_frames):
            self.stop()
            raise RuntimeError("Picamera2 did not deliver frames in time")

        # Get camera info for logging
        camera_info = self._get_camera_info()
        log.info(
            "📷 Camera initialized: Backend=picamera2, Camera=%s, Resolution=%dx%d, FPS=~%d",
            camera_info,
            self.width,
            self.height,
            self.target_fps
        )

    def _get_camera_info(self) -> str:
        """Get camera sensor name/info."""
        try:
            if self.picam:
                # Try to get camera info from libcamera
                camera_info = getattr(self.picam, 'camera_properties', {})
                if camera_info:
                    model = camera_info.get('Model', 'Unknown')
                    return model
        except Exception:
            pass
        return "ov5647"  # Default for OV5647

    def _wait_for_frames(self, timeout_s: float, need: int) -> bool:
        got = 0
        t0 = time.time()
        last_seen = 0.0
        while time.time() - t0 < timeout_s and got < need:
            ts = self._last_frame_ts
            if ts > 0 and ts != last_seen:
                got += 1
                last_seen = ts
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
                frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if cv2 is not None else rgb

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
                # Throttle warnings
                if (self._drops_total % 50) == 1:
                    log.warning("Picamera2 capture warning: %s", e)
                time.sleep(0.05)

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
        log.info("Camera stopped cleanly")

    # Metrics
    @property
    def last_frame_ts(self) -> float:
        return self._last_frame_ts

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def frames_total(self) -> int:
        return self._frames_total

    @property
    def drops_total(self) -> int:
        return self._drops_total


# ==========================
# Public facade
# ==========================
class Camera:
    """
    Picamera2-only camera facade for Raspberry Pi.
    
    Forces Picamera2 backend on Raspberry Pi hardware.
    No fallback or backend selection - Picamera2 is required.
    """

    def __init__(
        self,
        device: str | int = None,  # Ignored - Picamera2 doesn't use device paths
        width: int = int(os.getenv("CAMERA_WIDTH", "640")),
        height: int = int(os.getenv("CAMERA_HEIGHT", "480")),
        target_fps: int = _env_int("CAMERA_FPS", 30),
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.target_fps = int(target_fps)
        self.backend_name = "picamera2"

        # Verify we're on Raspberry Pi
        if not _detect_raspberry_pi():
            log.warning("Not running on Raspberry Pi - Picamera2 may not work correctly")

        # Check Picamera2 availability
        if not _PICAM2_OK:
            raise RuntimeError(
                "Picamera2 is required but not available. "
                "Install with: sudo apt install -y python3-picamera2"
            )

        # Initialize Picamera2 backend (no fallback)
        try:
            self._backend = _PiCam2Backend(
                width=self.width,
                height=self.height,
                target_fps=self.target_fps,
            )
        except Exception as e:
            log.error("Failed to initialize Picamera2: %s", e)
            raise RuntimeError(f"Picamera2 initialization failed: {e}") from e

        atexit.register(self.stop)

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
        default_ms = _env_int("FRAME_STALE_MS", 2500)
        limit = stale_ms if stale_ms is not None else default_ms
        last_ts = getattr(self._backend, "last_frame_ts", 0.0)
        try:
            return (time.time() - float(last_ts)) * 1000.0 < float(limit)
        except Exception:
            return False

    @property
    def fps(self) -> float:
        return 0.0 if self._backend is None else float(getattr(self._backend, "fps", 0.0))

    @property
    def frames_total(self) -> int:
        return 0 if self._backend is None else int(getattr(self._backend, "frames_total", 0))

    @property
    def drops_total(self) -> int:
        return 0 if self._backend is None else int(getattr(self._backend, "drops_total", 0))

    @property
    def backend(self) -> str:
        return self.backend_name

    def stop(self) -> None:
        try:
            if self._backend:
                self._backend.stop()
        finally:
            self._backend = None
