"""Autonomy control subsystem."""

from __future__ import annotations

from typing import Mapping

from .base import AutonomyError, AutonomyModeHandler
from .controller import AutonomyController, ModeRegistry
from .modes.simple import DEFAULT_MODE_FACTORIES

__all__ = [
    "AutonomyController",
    "AutonomyError",
    "AutonomyModeHandler",
    "ModeRegistry",
    "build_default_registry",
    "build_default_controller",
]


def build_default_registry() -> ModeRegistry:
    registry = ModeRegistry()
    for name, factory in DEFAULT_MODE_FACTORIES().items():
        registry.register(name, factory)
    return registry


def build_default_controller(*, context: Mapping[str, object] | None = None):
    registry = build_default_registry()
    return AutonomyController(registry, context=context)
