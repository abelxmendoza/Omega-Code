"""FastAPI routes for Pi-local gamepad/controller detection.

Reports whether a physical gamepad (e.g. Xbox controller) is connected
to the Raspberry Pi via USB or Bluetooth by querying evdev.  This lets
the browser UI know a Pi-side controller is active even though the
Web Gamepad API can only see browser-local devices.
"""

from __future__ import annotations

import logging
from typing import Any, Dict

from fastapi import APIRouter

__all__ = ["router"]

logger = logging.getLogger("gamepad.api")

router = APIRouter(prefix="/gamepad", tags=["Gamepad"])


def _probe_evdev() -> Dict[str, Any]:
    """Return the first gamepad found via evdev, or connected=False."""
    try:
        from evdev import list_devices, InputDevice, ecodes  # type: ignore

        for path in list_devices():
            try:
                dev = InputDevice(path)
                caps = dev.capabilities()
                if ecodes.EV_ABS not in caps:
                    continue
                abs_caps = caps[ecodes.EV_ABS]
                # Must have at least one analogue stick axis
                STICK_AXES = {ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY}
                if not STICK_AXES.intersection(abs_caps):
                    continue
                return {"connected": True, "name": dev.name, "path": str(path)}
            except Exception:
                continue

        return {"connected": False}

    except ImportError:
        return {"connected": False, "evdev": False}
    except Exception as exc:
        logger.debug("evdev probe error: %s", exc)
        return {"connected": False}


@router.get("/status")
async def gamepad_status() -> Dict[str, Any]:
    """Return whether a gamepad is physically connected to the Pi."""
    return _probe_evdev()
