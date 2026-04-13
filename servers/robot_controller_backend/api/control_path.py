"""
ControlPathRegistry  (Step 1 — Phase 1)
-----------------------------------------
Additive-only registry that records which control paths are active at
startup (motor, sensor, autonomy).  Nothing in the existing code changes —
callers register paths after their own init; the registry only reads and logs.

Exposed via GET /api/control/status so the UI (or an operator) can check
which paths are live without digging through logs.

CONTROL_MODE=auto (default)  — both direct and ROS2 paths attempted
CONTROL_MODE=direct           — skip ROS2 publish, go straight to PCA9685
CONTROL_MODE=ros2             — ROS2 only, no direct fallback
"""

from __future__ import annotations

import os
import logging
from typing import Dict, Any

from fastapi import APIRouter

__all__ = ["router", "registry"]

log = logging.getLogger("control_path")

# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

class ControlPathRegistry:
    """
    Lightweight in-process registry of active control paths.

    Keys and their expected values
    --------------------------------
    motor   : 'direct' | 'ros2' | 'both' | 'none'
    sensor  : 'direct' | 'ros2' | 'none'
    autonomy: 'fastapi' | 'ws_server' | 'none'
    """

    def __init__(self) -> None:
        self._paths: Dict[str, str] = {
            "motor":    "none",
            "sensor":   "none",
            "autonomy": "none",
        }
        self._control_mode: str = os.environ.get("CONTROL_MODE", "auto").lower()

    def register(self, subsystem: str, path: str) -> None:
        """Record which path a subsystem is using.  Idempotent."""
        self._paths[subsystem] = path
        log.info("[CTRL_PATH] %s → %s", subsystem, path)

    def status(self) -> Dict[str, Any]:
        return {
            "control_mode": self._control_mode,
            "paths": dict(self._paths),
        }


registry = ControlPathRegistry()

# ---------------------------------------------------------------------------
# Route
# ---------------------------------------------------------------------------

router = APIRouter(prefix="/api/control", tags=["ControlPath"])


@router.get("/status")
async def control_status() -> Dict[str, Any]:
    """Return the active control path for each subsystem."""
    return registry.status()
