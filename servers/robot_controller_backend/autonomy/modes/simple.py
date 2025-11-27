"""Built-in autonomy mode handlers used by the default registry.

The goal of these handlers is to provide sensible development-time behaviour so
that the UI can interact with the backend immediately.  They intentionally keep
side-effects to logging so they are safe to run in environments without real
hardware attached.
"""

from __future__ import annotations

import time
from typing import Any, Mapping, MutableMapping

from ..base import AutonomyModeHandler, AutonomyError

__all__ = [
    "IdleMode",
    "LoggingMode",
    "WaypointMode",
    "DockMode",
    "DEFAULT_MODE_FACTORIES",
]


class IdleMode(AutonomyModeHandler):
    async def start(self, params: Mapping[str, Any]) -> None:
        self.logger.info("Robot is now idle (params=%s)", dict(params))

    async def stop(self) -> None:
        self.logger.info("Leaving idle mode")


class LoggingMode(AutonomyModeHandler):
    """Generic handler that tracks the last parameters it received."""

    def __init__(self, name: str, *, logger=None) -> None:
        super().__init__(name, logger=logger)
        self._state: MutableMapping[str, Any] = {}

    async def start(self, params: Mapping[str, Any]) -> None:
        self._state = dict(params)
        self._state.setdefault("startedAt", time.time())
        self.logger.info("Mode '%s' start → %s", self.name, self._state)

    async def update(self, params: Mapping[str, Any]) -> None:
        self._state.update(dict(params))
        self.logger.info("Mode '%s' update → %s", self.name, self._state)

    async def stop(self) -> None:
        self.logger.info("Mode '%s' stopped", self.name)

    def snapshot(self) -> MutableMapping[str, Any]:
        return dict(self._state)


class WaypointMode(LoggingMode):
    def __init__(self, *, logger=None) -> None:
        super().__init__("waypoints", logger=logger)
        self._last_waypoint: MutableMapping[str, Any] | None = None

    async def set_waypoint(self, label: str, lat: float, lon: float) -> None:
        self._last_waypoint = {"label": label, "lat": lat, "lon": lon, "ts": time.time()}
        self.logger.info("New waypoint recorded: %s", self._last_waypoint)

    def snapshot(self) -> MutableMapping[str, Any]:
        snap = super().snapshot()
        if self._last_waypoint:
            snap = dict(snap)
            snap["lastWaypoint"] = dict(self._last_waypoint)
        return snap


class DockMode(LoggingMode):
    def __init__(self, *, logger=None) -> None:
        super().__init__("dock", logger=logger)
        self._dock_requested = False

    async def start(self, params: Mapping[str, Any]) -> None:
        await super().start(params)
        self._dock_requested = True
        self.logger.info("Docking sequence initiated")

    async def dock(self) -> None:
        if not self._dock_requested:
            raise AutonomyError("dock() called before start()")
        self.logger.info("Dock confirmation acknowledged")


def DEFAULT_MODE_FACTORIES() -> Mapping[str, Any]:
    """Return lazily-built factories used by :func:`build_default_registry`."""

    return {
        "idle": lambda **kw: IdleMode(logger=kw.get("logger")),
        "patrol": lambda **kw: LoggingMode("patrol", logger=kw.get("logger")),
        "follow": lambda **kw: LoggingMode("follow", logger=kw.get("logger")),
        "line_follow": lambda **kw: LoggingMode("line_follow", logger=kw.get("logger")),
        "avoid_obstacles": lambda **kw: LoggingMode("avoid_obstacles", logger=kw.get("logger")),
        "edge_detect": lambda **kw: LoggingMode("edge_detect", logger=kw.get("logger")),
        "waypoints": lambda **kw: WaypointMode(logger=kw.get("logger")),
        "color_track": lambda **kw: LoggingMode("color_track", logger=kw.get("logger")),
        "aruco": lambda **kw: LoggingMode("aruco", logger=kw.get("logger")),
        "person_follow": lambda **kw: LoggingMode("person_follow", logger=kw.get("logger")),
        "scan_servo": lambda **kw: LoggingMode("scan_servo", logger=kw.get("logger")),
        "dock": lambda **kw: DockMode(logger=kw.get("logger")),
    }
