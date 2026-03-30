"""
GStreamer/OpenCV camera backend for Raspberry Pi CSI cameras on Ubuntu.
Drop-in replacement for picamera2-based camera.py.
Uses libcamerasrc GStreamer pipeline — works headless, no pykms required.
"""

from __future__ import annotations

import os
import time
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

GSTREAMER_PIPELINE = (
    f"libcamerasrc ! "
    f"video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},"
    f"framerate={CAMERA_FPS}/1 ! "
    f"queue leaky=downstream max-size-buffers=1 ! "
    f"videoconvert ! video/x-raw,format=BGR ! "
    f"appsink max-buffers=1 drop=true sync=false"
)


class Camera:
    def __init__(self, width=None, height=None, fps=None, **kwargs):
        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._last_frame_time: float = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._cap: Optional[cv2.VideoCapture] = None
        self.backend = "gstreamer"
        self.fps: float = 0.0
        self._start()

    def _start(self):
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        # Wait up to 5s for first frame
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if self._frame is not None:
                log.info("Camera ready — %dx%d @ %d fps",
                         CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS)
                return
            time.sleep(0.1)
        log.warning("Camera warmup timeout — no frames yet")

    def _capture_loop(self):
        log.info("Opening GStreamer pipeline: %s", GSTREAMER_PIPELINE)
        self._cap = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            log.warning("GStreamer pipeline failed — falling back to V4L2 (/dev/video0)")
            self._cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
                self._cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
                self.backend = "v4l2"
                log.info("V4L2 backend opened: %dx%d @ %d fps",
                         CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS)
            else:
                log.error("Failed to open camera (tried GStreamer and V4L2)")
                self._running = False
                return

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

        if self._cap:
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
            self._thread.join(timeout=3.0)

    # Compatibility shims for video_server.py
    @property
    def is_open(self) -> bool:
        return self.is_alive()

    def read(self):
        frame = self.get_frame()
        return (frame is not None), frame


