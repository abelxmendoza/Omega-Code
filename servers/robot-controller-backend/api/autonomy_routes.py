"""FastAPI routes for the Autonomy control subsystem.

The routes wrap :class:`autonomy.controller.AutonomyController` so the UI can
start, stop, and update robot autonomy modes without needing to speak the raw
WebSocket protocol.  Responses mirror the structure returned by the controller
so the frontend can hydrate its status view immediately.
"""

from __future__ import annotations

import logging
from typing import Any, Dict

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from autonomy import AutonomyError, build_default_controller

__all__ = [
    "router",
]

logger = logging.getLogger("autonomy.api")
controller = build_default_controller()

router = APIRouter(prefix="/autonomy", tags=["Autonomy"])


class StartPayload(BaseModel):
    mode: str = Field(..., description="Name of the autonomy mode to activate.")
    params: Dict[str, Any] = Field(
        default_factory=dict,
        description="Arbitrary parameter map passed to the mode handler.",
    )


class UpdatePayload(BaseModel):
    params: Dict[str, Any] = Field(
        default_factory=dict,
        description="Arbitrary parameter map passed to the active mode handler.",
    )


class WaypointPayload(BaseModel):
    label: str = Field(..., min_length=1, description="Human readable waypoint label.")
    lat: float = Field(..., ge=-90.0, le=90.0, description="Latitude in degrees.")
    lon: float = Field(..., ge=-180.0, le=180.0, description="Longitude in degrees.")


def _ok(action: str, autonomy: Dict[str, Any]) -> Dict[str, Any]:
    return {"status": "ok", "action": action, "autonomy": autonomy}


def _handle_autonomy_error(exc: AutonomyError) -> None:
    logger.warning("Autonomy command failed: %s", exc)
    raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.get("/status")
async def get_status() -> Dict[str, Any]:
    """Return the current autonomy session state."""

    return _ok("status", controller.status())


@router.post("/start")
async def start(payload: StartPayload) -> Dict[str, Any]:
    try:
        status = await controller.start(payload.mode, payload.params)
    except AutonomyError as exc:
        _handle_autonomy_error(exc)
    return _ok("start", status)


@router.post("/stop")
async def stop() -> Dict[str, Any]:
    try:
        status = await controller.stop()
    except AutonomyError as exc:
        _handle_autonomy_error(exc)
    return _ok("stop", status)


@router.post("/update")
async def update(payload: UpdatePayload) -> Dict[str, Any]:
    try:
        status = await controller.update(payload.params)
    except AutonomyError as exc:
        _handle_autonomy_error(exc)
    return _ok("update", status)


@router.post("/dock")
async def dock() -> Dict[str, Any]:
    try:
        status = await controller.dock()
    except AutonomyError as exc:
        _handle_autonomy_error(exc)
    return _ok("dock", status)


@router.post("/set_waypoint")
async def set_waypoint(payload: WaypointPayload) -> Dict[str, Any]:
    try:
        status = await controller.set_waypoint(payload.label, payload.lat, payload.lon)
    except AutonomyError as exc:
        _handle_autonomy_error(exc)
    return _ok("set_waypoint", status)
