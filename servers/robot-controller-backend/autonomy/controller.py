"""Autonomy controller orchestrating mode handlers and routing commands.

The controller acts as a tiny runtime responsible for:

* Keeping track of which autonomy mode is currently active.
* Serialising access across WebSocket commands (``start``/``stop``/``update``).
* Instantiating handlers from a registry so individual behaviours remain
  composable and testable in isolation.
* Normalising all status information into a dictionary that can be surfaced to
  the UI.

It purposely avoids tying the implementation to a specific web framework so the
same controller can be reused by the WebSocket server and future HTTP APIs.
"""

from __future__ import annotations

import asyncio
import inspect
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, Mapping, MutableMapping, Optional

from .base import AutonomyError, AutonomyModeHandler

__all__ = [
    "ModeRegistry",
    "AutonomyController",
]

ModeFactory = Callable[..., AutonomyModeHandler]


# Backwards compatibility aliases for legacy mode names used by older UIs.
_MODE_ALIASES = {
    "line_track": "line_follow",
}


def _canonical_name(name: str) -> str:
    cleaned = (name or "").strip().lower().replace("-", "_").replace(" ", "_")
    if not cleaned:
        raise AutonomyError("mode name is empty")
    return _MODE_ALIASES.get(cleaned, cleaned)


@dataclass(slots=True)
class _ActiveSession:
    mode: str
    handler: AutonomyModeHandler
    params: MutableMapping[str, Any] = field(default_factory=dict)
    started_at: float = field(default_factory=lambda: time.time())

    def snapshot(self) -> MutableMapping[str, Any]:
        snap = self.handler.snapshot()
        if not isinstance(snap, MutableMapping):
            snap = dict(snap or {})
        return snap


class ModeRegistry:
    """Stores factories for autonomy handlers."""

    def __init__(self) -> None:
        self._factories: Dict[str, ModeFactory] = {}

    def register(self, name: str, factory: ModeFactory) -> None:
        key = _canonical_name(name)
        if not callable(factory):
            raise TypeError("factory must be callable")
        self._factories[key] = factory

    def get(self, name: str) -> ModeFactory:
        key = _canonical_name(name)
        try:
            return self._factories[key]
        except KeyError as exc:  # pragma: no cover - exercised via AutonomyController
            raise AutonomyError(f"unknown autonomy mode '{name}'") from exc

    def has(self, name: str) -> bool:
        try:
            self.get(name)
            return True
        except AutonomyError:
            return False

    def create(self, name: str, *, context: Mapping[str, Any], logger: logging.Logger) -> AutonomyModeHandler:
        factory = self.get(name)
        # Provide the factory with a mode-specific child logger for consistency.
        mode_logger = logger.getChild(_canonical_name(name))
        try:
            handler = factory(context=context, logger=mode_logger)
        except TypeError:
            try:
                handler = factory(context)
            except TypeError:
                handler = factory()
        if not isinstance(handler, AutonomyModeHandler):
            raise AutonomyError(f"factory for mode '{name}' did not return an AutonomyModeHandler")
        return handler

    def available_modes(self) -> Iterable[str]:
        return sorted(self._factories.keys())


class AutonomyController:
    """Coordinates autonomy commands coming from the transport layer."""

    def __init__(self, registry: ModeRegistry | None = None, *, context: Mapping[str, Any] | None = None,
                 logger: logging.Logger | None = None) -> None:
        self.registry = registry or ModeRegistry()
        self.context: Dict[str, Any] = dict(context or {})
        self.logger = logger or logging.getLogger("autonomy")
        self._lock = asyncio.Lock()
        self._session: Optional[_ActiveSession] = None

    # ------------------------------------------------------------------
    def status(self) -> Dict[str, Any]:
        if not self._session:
            return {
                "active": False,
                "mode": "idle",
                "params": {},
                "availableModes": list(self.registry.available_modes()),
            }
        snap = self._session.params.copy()
        snap.update(self._session.snapshot())
        return {
            "active": True,
            "mode": self._session.mode,
            "params": snap,
            "startedAt": self._session.started_at,
            "handler": type(self._session.handler).__name__,
            "availableModes": list(self.registry.available_modes()),
        }

    # ------------------------------------------------------------------
    async def start(self, mode: str, params: Mapping[str, Any] | None = None) -> Dict[str, Any]:
        canonical = _canonical_name(mode)
        clean_params = self._ensure_mapping(params)
        async with self._lock:
            handler = self.registry.create(canonical, context=self.context, logger=self.logger)
            await self._stop_active_locked()
            await self._call(handler.start, clean_params)
            session = _ActiveSession(mode=canonical, handler=handler, params=dict(clean_params))
            session.params.update(session.snapshot())
            self._session = session
            self.logger.info("autonomy mode '%s' started", canonical)
            return self.status()

    async def stop(self) -> Dict[str, Any]:
        async with self._lock:
            await self._stop_active_locked()
            self.logger.info("autonomy stopped")
            return self.status()

    async def update(self, params: Mapping[str, Any]) -> Dict[str, Any]:
        clean_params = self._ensure_mapping(params)
        async with self._lock:
            session = self._require_session()
            await self._call(session.handler.update, clean_params)
            session.params.update(clean_params)
            session.params.update(session.snapshot())
            self.logger.debug("autonomy mode '%s' updated -> %s", session.mode, session.params)
            return self.status()

    async def dock(self) -> Dict[str, Any]:
        async with self._lock:
            if self.registry.has("dock") and (not self._session or self._session.mode != "dock"):
                handler = self.registry.create("dock", context=self.context, logger=self.logger)
                await self._stop_active_locked()
                await self._call(handler.start, {})
                session = _ActiveSession(mode="dock", handler=handler, params={})
                session.params.update(session.snapshot())
                self._session = session
                self.logger.info("autonomy mode 'dock' started")
                return self.status()
            session = self._require_session()
            await self._call(session.handler.dock)
            session.params.update(session.snapshot())
            self.logger.info("autonomy dock command acknowledged")
            return self.status()

    async def set_waypoint(self, label: str, lat: float, lon: float) -> Dict[str, Any]:
        if not label:
            raise AutonomyError("waypoint label is required")
        try:
            lat_f = float(lat)
            lon_f = float(lon)
        except Exception as exc:  # pragma: no cover - validated in tests
            raise AutonomyError("latitude/longitude must be numbers") from exc

        async with self._lock:
            session = self._require_session()
            await self._call(session.handler.set_waypoint, label, lat_f, lon_f)
            session.params.setdefault("lastWaypoint", {})
            session.params["lastWaypoint"].update({"label": label, "lat": lat_f, "lon": lon_f, "ts": time.time()})
            session.params.update(session.snapshot())
            self.logger.info("autonomy waypoint set -> %s", session.params["lastWaypoint"])
            return self.status()

    # ------------------------------------------------------------------
    async def _stop_active_locked(self) -> None:
        if not self._session:
            return
        session, self._session = self._session, None
        try:
            await self._call(session.handler.stop)
        finally:
            self.logger.debug("autonomy mode '%s' stopped", session.mode)

    def _require_session(self) -> _ActiveSession:
        if not self._session:
            raise AutonomyError("no autonomy mode is active")
        return self._session

    async def _call(self, fn: Callable[..., Any], *args: Any) -> Any:
        try:
            result = fn(*args)
            if inspect.isawaitable(result):
                return await result
            return result
        except AutonomyError:
            raise
        except Exception as exc:  # pragma: no cover - caught via tests
            raise AutonomyError(str(exc)) from exc

    @staticmethod
    def _ensure_mapping(value: Mapping[str, Any] | None) -> MutableMapping[str, Any]:
        if value is None:
            return {}
        if not isinstance(value, Mapping):
            raise AutonomyError("params must be a mapping")
        return dict(value)
