"""FastAPI routes for Pi-local gamepad/controller detection.

Reports whether a physical gamepad (e.g. Xbox controller) is connected
to the Raspberry Pi via USB or Bluetooth.

Detection strategy (in order, stops at first success):
  1. /proc/bus/input/devices  — world-readable, gives device name
  2. /dev/input/js*           — joystick nodes exist when any joystick connected
  3. evdev list_devices()     — needs input group; richest info but fails without it
"""

from __future__ import annotations

import glob
import logging
import os
import re
from typing import Any, Dict

from fastapi import APIRouter

__all__ = ["router"]

logger = logging.getLogger("gamepad.api")

router = APIRouter(prefix="/gamepad", tags=["Gamepad"])

# Heuristic: these substrings in a device name indicate a gamepad/joystick
_GAMEPAD_KEYWORDS = (
    "xbox", "gamepad", "joystick", "controller", "pad", "joypad",
    "playstation", "dualshock", "dualsense",
)


def _probe_proc() -> Dict[str, Any]:
    """Parse /proc/bus/input/devices (world-readable) for gamepad entries."""
    try:
        with open("/proc/bus/input/devices", "r") as fh:
            content = fh.read()

        # Each device block is separated by blank lines.
        # "N: Name=..." gives the name; "H: Handlers=..." gives event/js nodes.
        name = None
        handlers: list[str] = []
        for line in content.splitlines():
            if line.startswith("N: Name="):
                name = line.split("=", 1)[1].strip('"')
                handlers = []
            elif line.startswith("H: Handlers="):
                handlers = line.split("=", 1)[1].split()
            elif line == "" and name:
                # End of block — check if it looks like a gamepad
                if any(k in name.lower() for k in _GAMEPAD_KEYWORDS):
                    # Must have a js* or event* handler
                    if any(h.startswith(("js", "event")) for h in handlers):
                        return {"connected": True, "name": name, "source": "proc"}
                name = None
                handlers = []

        return {"connected": False}

    except Exception as exc:
        logger.debug("proc probe error: %s", exc)
        return {"connected": False}


def _probe_js_nodes() -> Dict[str, Any]:
    """Check for /dev/input/js* nodes — world-readable (r-- for others)."""
    try:
        nodes = sorted(glob.glob("/dev/input/js*"))
        if not nodes:
            return {"connected": False}
        # Try to read the device name from sysfs
        for node in nodes:
            idx = re.search(r"js(\d+)$", node)
            if not idx:
                continue
            sysfs = f"/sys/class/input/js{idx.group(1)}/device/name"
            try:
                name = open(sysfs).read().strip()
            except Exception:
                name = os.path.basename(node)
            return {"connected": True, "name": name, "path": node, "source": "js_node"}
        return {"connected": True, "name": "Unknown gamepad", "source": "js_node"}
    except Exception as exc:
        logger.debug("js-node probe error: %s", exc)
        return {"connected": False}


def _probe_evdev() -> Dict[str, Any]:
    """Rich evdev probe — requires input group membership."""
    try:
        from evdev import list_devices, InputDevice, ecodes  # type: ignore

        for path in list_devices():
            try:
                dev = InputDevice(path)
                caps = dev.capabilities()
                if ecodes.EV_ABS not in caps:
                    continue
                abs_caps = caps[ecodes.EV_ABS]
                STICK_AXES = {ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY}
                if not STICK_AXES.intersection(abs_caps):
                    continue
                return {"connected": True, "name": dev.name, "path": str(path), "source": "evdev"}
            except Exception:
                continue

        return {"connected": False}

    except ImportError:
        return {"connected": False}
    except Exception as exc:
        logger.debug("evdev probe error: %s", exc)
        return {"connected": False}


def _probe() -> Dict[str, Any]:
    """Try all probes in order; return first success."""
    for fn in (_probe_proc, _probe_js_nodes, _probe_evdev):
        result = fn()
        if result.get("connected"):
            return result
    return {"connected": False}


@router.get("/status")
async def gamepad_status() -> Dict[str, Any]:
    """Return whether a gamepad is physically connected to the Pi."""
    return _probe()
