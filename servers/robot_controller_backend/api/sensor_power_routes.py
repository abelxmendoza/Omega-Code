"""
Sensor REST endpoints.

GET  /api/sensors/power     → {"ok": true, "enabled": bool}
POST /api/sensors/power     → body {"enabled": bool}
                            → {"ok": true, "enabled": bool, "changed": bool}
GET  /api/sensors/distance  → {"ok": true, "distance_cm": float|null, "stale": bool}
"""

from __future__ import annotations

import time

from fastapi import APIRouter, Request
from pydantic import BaseModel

router = APIRouter(prefix="/api/sensors", tags=["Sensors"])

# Reading older than this is considered stale
_STALE_THRESHOLD_S = 2.0


class SensorPowerBody(BaseModel):
    enabled: bool


@router.get("/power")
async def get_sensor_power(request: Request):
    bridge = getattr(request.app.state, "sensor_bridge", None)
    enabled = bridge.is_streaming() if bridge else False
    return {"ok": True, "enabled": enabled}


@router.post("/power")
async def set_sensor_power(body: SensorPowerBody, request: Request):
    bridge = getattr(request.app.state, "sensor_bridge", None)
    if bridge is None:
        return {"ok": False, "error": "sensor_bridge_unavailable", "enabled": False}

    was_enabled = bridge.is_streaming()
    if body.enabled:
        bridge.enable_streaming()
    else:
        bridge.disable_streaming()

    return {"ok": True, "enabled": bridge.is_streaming(), "changed": was_enabled != body.enabled}


@router.get("/distance")
async def get_distance(request: Request):
    bridge = getattr(request.app.state, "sensor_bridge", None)
    if bridge is None or bridge.latest_ultrasonic is None:
        return {"ok": False, "distance_cm": None, "stale": True}

    age = time.monotonic() - bridge.last_valid_ultra_ts
    stale = age > _STALE_THRESHOLD_S
    return {"ok": True, "distance_cm": bridge.latest_ultrasonic, "stale": stale}
