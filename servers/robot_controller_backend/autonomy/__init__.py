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
    """Build a registry with all default mode factories.
    
    ROS2 modes automatically register if ROS2 is available, overriding simple modes.
    """
    registry = ModeRegistry()
    
    # Add default modes first
    for name, factory in DEFAULT_MODE_FACTORIES().items():
        registry.register(name, factory)
    
    # Add ROS2 modes (they override simple modes if ROS2 is available)
    try:
        from .modes.ros2_modes import ROS2_MODE_FACTORIES
        for name, factory in ROS2_MODE_FACTORIES().items():
            # Only register if ROS2 is available
            try:
                from api.ros_native_bridge import _rclpy_available
                if _rclpy_available:
                    registry.register(name, factory)  # This overwrites the simple mode
            except ImportError:
                pass  # ROS2 not available, keep simple mode
    except ImportError:
        pass  # ROS2 modes not available
    
    return registry


def build_default_controller(*, context: Mapping[str, object] | None = None):
    registry = build_default_registry()
    return AutonomyController(registry, context=context)
