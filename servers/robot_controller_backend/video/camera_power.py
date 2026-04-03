"""
camera_power.py — Singleton that owns the camera enabled/disabled state.

All camera power decisions go through this module so there is exactly one
source of truth. video_server.py delegates start/stop calls here.
"""

from __future__ import annotations

import threading
import time
import logging

log = logging.getLogger(__name__)


class CameraPowerManager:
    """
    Singleton camera power controller.

    Usage in video_server.py:
        from video.camera_power import camera_power

        # startup
        camera_power.enable(create_fn=_create_camera)

        # power toggle endpoint
        camera_power.disable(stop_fn=_stop_camera)

        # in generate_frames generator
        while camera_power.is_enabled(): ...
        camera_power.mark_frame()

        # in health / video_feed
        if not camera_power.is_enabled(): return 503
    """

    def __init__(self) -> None:
        self._enabled: bool = False
        self._lock = threading.Lock()
        self._last_frame_ts: float = 0.0

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def is_enabled(self) -> bool:
        return self._enabled

    def enable(self, create_fn) -> bool:
        """
        Mark camera as enabled and call create_fn() to bring up hardware.
        Returns True if create_fn succeeded or camera was already enabled.
        """
        with self._lock:
            if self._enabled:
                return True
            ok: bool = bool(create_fn())
            if ok:
                self._enabled = True
                self._last_frame_ts = time.time()
                log.info("[CameraPower] enabled")
            else:
                log.warning("[CameraPower] enable failed — create_fn returned False")
            return ok

    def disable(self, stop_fn) -> None:
        """
        Mark camera as disabled and call stop_fn() to release hardware.
        Safe to call when already disabled.
        """
        with self._lock:
            if not self._enabled:
                return
            log.info("[CameraPower] disabling")
            try:
                stop_fn()
            except Exception as exc:
                log.warning("[CameraPower] stop_fn error: %s", exc)
            self._enabled = False

    def mark_frame(self) -> None:
        """Call after every successfully delivered frame to update health timestamp."""
        self._last_frame_ts = time.time()

    def is_healthy(self, stale_sec: float = 2.0) -> bool:
        """True only when enabled and a frame has been seen recently."""
        if not self._enabled:
            return False
        if self._last_frame_ts == 0.0:
            return False
        return (time.time() - self._last_frame_ts) < stale_sec

    def status_dict(self) -> dict:
        return {
            "enabled": self._enabled,
            "healthy": self.is_healthy(),
        }


# Module-level singleton — import this everywhere.
camera_power = CameraPowerManager()
