"""
Camera backend for Raspberry Pi CSI cameras on Ubuntu.
Tries picamera2 first (ov5647 via libcamera), falls back to GStreamer v4l2src.
"""

from __future__ import annotations

import os
import sys
import time
import types
import threading
import logging
from typing import Optional

import cv2
import numpy as np

log = logging.getLogger(__name__)

CAMERA_WIDTH    = int(os.environ.get("CAMERA_WIDTH", 640))
CAMERA_HEIGHT   = int(os.environ.get("CAMERA_HEIGHT", 480))
CAMERA_FPS      = int(os.environ.get("CAMERA_FPS", 10))
FRAME_STALE_MS  = int(os.environ.get("FRAME_STALE_MS", 2500))

V4L2_PIPELINE = (
    f"v4l2src device=/dev/video0 ! "
    f"video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},"
    f"framerate={CAMERA_FPS}/1 ! "
    f"videoconvert ! video/x-raw,format=BGR ! "
    f"appsink max-buffers=1 drop=true sync=false"
)

GSTREAMER_PIPELINE = (
    f"libcamerasrc ! "
    f"video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},"
    f"framerate={CAMERA_FPS}/1 ! "
    f"queue leaky=downstream max-size-buffers=1 ! "
    f"videoconvert ! video/x-raw,format=BGR ! "
    f"appsink max-buffers=1 drop=true sync=false"
)


def _stub_pykms():
    """Inject a pykms stub so picamera2 can import without the real kms library."""
    if 'pykms' not in sys.modules:
        class _AnyAttr:
            def __getattr__(self, name):
                return name
        pykms = types.ModuleType('pykms')
        pykms.PixelFormat = _AnyAttr()
        pykms.Card = None
        pykms.DumbFramebuffer = None
        sys.modules['pykms'] = pykms
        sys.modules['kms'] = pykms


def _try_open_picamera2():
    """Try to open picamera2. Returns a Picamera2 instance or None."""
    try:
        _stub_pykms()
        from picamera2 import Picamera2
        picam = Picamera2()
        cfg = picam.create_video_configuration(
            main={"format": "RGB888", "size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
            controls={"FrameRate": float(CAMERA_FPS)}
        )
        picam.configure(cfg)
        picam.start()
        log.info("picamera2 opened: %dx%d @ %d fps", CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS)
        return picam
    except Exception as e:
        log.warning("picamera2 unavailable: %s", e)
        return None


class Camera:
    def __init__(self, width=None, height=None, fps=None, **kwargs):
        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._last_frame_time: float = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._cap: Optional[cv2.VideoCapture] = None
        self._picam = None
        self.backend = "none"
        self.fps: float = 0.0
        self._start()

    def _start(self):
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if self._frame is not None:
                log.info("Camera ready — %dx%d @ %d fps (backend: %s)",
                         CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, self.backend)
                return
            time.sleep(0.1)
        log.warning("Camera warmup timeout — no frames yet")

    def _capture_loop(self):
        # Try picamera2 first
        self._picam = _try_open_picamera2()
        if self._picam is not None:
            self.backend = "picamera2"
            self._run_picamera2_loop()
            return

        # Fall back to GStreamer libcamerasrc
        log.info("Trying GStreamer libcamerasrc pipeline")
        self._cap = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            # Fall back to v4l2src
            log.warning("libcamerasrc failed — trying v4l2src GStreamer pipeline")
            self._cap = cv2.VideoCapture(V4L2_PIPELINE, cv2.CAP_GSTREAMER)
            if not self._cap.isOpened():
                log.error("Failed to open camera (tried picamera2, libcamerasrc, v4l2src)")
                self._running = False
                return
            self.backend = "v4l2"
            log.info("v4l2src GStreamer pipeline opened")
        else:
            self.backend = "gstreamer"

        self._run_opencv_loop()

    def _run_picamera2_loop(self):
        _fps_t0 = time.time()
        _fps_count = 0
        while self._running:
            try:
                rgb = self._picam.capture_array("main")
                frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                now = time.time()
                _fps_count += 1
                elapsed = now - _fps_t0
                if elapsed >= 2.0:
                    self.fps = _fps_count / elapsed
                    _fps_count = 0
                    _fps_t0 = now
                with self._lock:
                    self._frame = frame
                    self._last_frame_time = now
            except Exception as e:
                log.warning("picamera2 frame error: %s", e)
                time.sleep(0.1)
        try:
            self._picam.stop()
        except Exception:
            pass
        try:
            # close() returns the camera to Available state so it can be
            # re-acquired later without "Camera in Configured state" errors.
            self._picam.close()
        except Exception:
            pass
        self._picam = None
        time.sleep(0.5)  # brief settle before any re-acquire attempt

    def _run_opencv_loop(self):
        _fps_t0 = time.time()
        _fps_count = 0
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                log.warning("Frame read failed — retrying in 1s")
                time.sleep(1.0)
                continue
            now = time.time()
            _fps_count += 1
            elapsed = now - _fps_t0
            if elapsed >= 2.0:
                self.fps = _fps_count / elapsed
                _fps_count = 0
                _fps_t0 = now
            with self._lock:
                self._frame = frame
                self._last_frame_time = now
        self._cap.release()

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def is_alive(self, stale_ms=None) -> bool:
        if self._last_frame_time == 0:
            return False
        threshold = stale_ms if stale_ms is not None else FRAME_STALE_MS
        return (time.time() - self._last_frame_time) * 1000 < threshold

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)

    # Compatibility shims for video_server.py
    @property
    def is_open(self) -> bool:
        return self.is_alive()

    def read(self):
        frame = self.get_frame()
        return (frame is not None), frame
