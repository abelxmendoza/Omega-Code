"""Core primitives for the autonomy control subsystem.

This module defines the :class:`AutonomyModeHandler` contract used by the
backend autonomy controller.  Handlers encapsulate the behaviour for a single
mode ("patrol", "line_follow", etc.) and expose optional hooks that can be
implemented depending on the capabilities of the mode.

Handlers are intentionally light weight so that they can be composed and
swapped out easily.  They receive a dedicated logger scoped to their mode name
so implementations can integrate with whatever telemetry or logging pipeline the
robot uses without knowing about the server itself.
"""

from __future__ import annotations

import logging
from typing import Any, Mapping, MutableMapping

__all__ = [
    "AutonomyError",
    "AutonomyModeHandler",
]


class AutonomyError(RuntimeError):
    """Raised when an autonomy operation cannot be completed."""


class AutonomyModeHandler:
    """Base class for individual autonomy mode handlers.

    Sub-classes should implement :meth:`start` at minimum.  Optional hooks can
    be implemented when the mode supports additional operations (e.g.
    :meth:`set_waypoint`).  By default the optional hooks raise
    :class:`AutonomyError` so the caller can gracefully propagate a structured
    error back to the UI.
    """

    name: str

    def __init__(self, name: str, *, logger: logging.Logger | None = None) -> None:
        self.name = name
        self.logger = logger or logging.getLogger(f"autonomy.{name}")

    # ------------------------------------------------------------------
    # Lifecycle hooks
    # ------------------------------------------------------------------
    async def start(self, params: Mapping[str, Any]) -> None:  # pragma: no cover - interface
        """Begin executing the mode with the supplied parameters."""
        raise NotImplementedError

    async def stop(self) -> None:  # pragma: no cover - default implementation
        """Stop the mode.  Sub-classes may override to clean up resources."""
        self.logger.debug("stop() noop for %s", self.name)

    async def update(self, params: Mapping[str, Any]) -> None:  # pragma: no cover - default implementation
        """Update the mode with new parameters."""
        if params:
            self.logger.debug("update(%s)", dict(params))

    # ------------------------------------------------------------------
    # Optional hooks
    # ------------------------------------------------------------------
    async def dock(self) -> None:  # pragma: no cover - default implementation
        raise AutonomyError(f"mode '{self.name}' does not implement dock()")

    async def set_waypoint(self, label: str, lat: float, lon: float) -> None:  # pragma: no cover - default implementation
        raise AutonomyError(f"mode '{self.name}' does not implement set_waypoint()")

    # ------------------------------------------------------------------
    def snapshot(self) -> MutableMapping[str, Any]:  # pragma: no cover - default implementation
        """Return a mutable snapshot of the mode state.

        The controller stores the returned mapping to expose richer status data
        to the UI.  Handlers can override this to surface progress information,
        diagnostics, etc.  By default we return an empty dict so the caller can
        mutate it without needing to guard against ``None``.
        """

        return {}
