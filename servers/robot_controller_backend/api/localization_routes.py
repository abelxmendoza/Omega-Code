"""FastAPI routes for hybrid dead-reckoning + ArUco localization.

Architecture
------------
  Prediction:
    A background asyncio task calls se2_filter.predict() at PREDICT_HZ using
    the last known motor command.  The command is updated via POST /localization/command
    which movement_routes.py calls whenever a new direction is issued.

  Correction:
    video/video_server.py (Flask :5000) POSTs ArUco detections with rvec/tvec
    to POST /localization/aruco_update.  The handler converts the 6-DOF pose to
    a world-frame SE(2) measurement and applies it to the filter.

  Output:
    GET /localization/pose returns the current SE(2) estimate + covariance.

Marker map
----------
Define your known marker world positions in MARKER_MAP below, or set the
ARUCO_MARKER_MAP_FILE env var to a JSON file path.  Map format:

    { "7":  {"x": 1.0, "y": 0.0, "alpha": 3.14159},
      "12": {"x": 0.0, "y": 2.0, "alpha": 1.5708}  }

where alpha is the marker's facing direction (radians, 0 = faces +X world axis).
An empty map means ArUco corrections are silently ignored (prediction still runs).
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import time
from typing import Any, Dict, List, Optional

import numpy as np
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from movement.pose_ekf import MarkerPose, SE2PoseFilter

log = logging.getLogger("localization")

router = APIRouter(prefix="/localization", tags=["Localization"])

# ---------------------------------------------------------------------------
# Prediction loop rate and velocity mapping
# ---------------------------------------------------------------------------
PREDICT_HZ = 20        # Hz — how often the background loop calls predict()
_DT = 1.0 / PREDICT_HZ

# Motor command → velocity estimate.
# These are rough values; tune them by measuring actual robot speeds.
# v_max: max linear speed (m/s) at PWM 4095
# omega_max: max angular speed (rad/s) during pivot
_V_MAX    = 0.40   # m/s at full speed
_W_MAX    = 1.80   # rad/s during pivot turn

# Map from movement command strings (same as ws_movement) to (v, omega) fractions
_CMD_TO_VW: Dict[str, tuple[float, float]] = {
    "forward":   ( 1.0,  0.0),
    "backward":  (-1.0,  0.0),
    "left":      ( 0.0,  1.0),   # pivot left = CCW = positive omega
    "right":     ( 0.0, -1.0),
    "stop":      ( 0.0,  0.0),
    "move-up":   ( 1.0,  0.0),
    "move-down": (-1.0,  0.0),
    "move-left": ( 0.0,  1.0),
    "move-right":( 0.0, -1.0),
    "move-stop": ( 0.0,  0.0),
}

# ---------------------------------------------------------------------------
# Marker map (loaded once at startup)
# ---------------------------------------------------------------------------

def _load_marker_map() -> Dict[int, MarkerPose]:
    """Load marker world poses from ARUCO_MARKER_MAP_FILE or return empty dict."""
    path = os.getenv("ARUCO_MARKER_MAP_FILE", "").strip()
    if not path:
        log.info("ARUCO_MARKER_MAP_FILE not set — ArUco corrections disabled until map is loaded")
        return {}
    try:
        with open(path) as f:
            raw = json.load(f)
        result = {
            int(k): MarkerPose(x=float(v["x"]), y=float(v["y"]), alpha=float(v["alpha"]))
            for k, v in raw.items()
        }
        log.info("Loaded %d marker poses from %s", len(result), path)
        return result
    except Exception as exc:
        log.warning("Failed to load marker map from %s: %s", path, exc)
        return {}


# ---------------------------------------------------------------------------
# Singleton filter (one per process lifetime)
# ---------------------------------------------------------------------------

_filter = SE2PoseFilter(marker_map=_load_marker_map())

# Last commanded velocity — updated by /localization/command
_last_v:     float = 0.0
_last_omega: float = 0.0
_last_speed_fraction: float = 0.0   # 0–1, proportional to PWM

# Background prediction task handle
_predict_task: Optional[asyncio.Task] = None


async def _prediction_loop() -> None:
    """Background task: call predict() at PREDICT_HZ using last known command."""
    while True:
        await asyncio.sleep(_DT)
        try:
            v     = _last_v     * _last_speed_fraction
            omega = _last_omega * _last_speed_fraction
            _filter.predict(v=v, omega=omega, dt=_DT)
        except Exception as exc:
            log.debug("predict() error: %s", exc)


def start_prediction_loop() -> None:
    """Start the background prediction task.  Call once from FastAPI lifespan."""
    global _predict_task
    if _predict_task is None or _predict_task.done():
        _predict_task = asyncio.create_task(_prediction_loop())
        log.info("Localization prediction loop started at %d Hz", PREDICT_HZ)


def stop_prediction_loop() -> None:
    """Cancel the prediction task on shutdown."""
    global _predict_task
    if _predict_task and not _predict_task.done():
        _predict_task.cancel()
        log.info("Localization prediction loop stopped")


# ---------------------------------------------------------------------------
# Request / response models
# ---------------------------------------------------------------------------

class ArucoUpdateRequest(BaseModel):
    """Payload from video_server when a marker is detected with pose data."""
    marker_id: int
    rvec:      List[float] = Field(..., min_length=3, max_length=3,
                                   description="Rodrigues rotation vector (3 floats)")
    tvec:      List[float] = Field(..., min_length=3, max_length=3,
                                   description="Translation vector in camera frame (3 floats, metres)")
    timestamp: Optional[float] = None


class CommandUpdateRequest(BaseModel):
    """Motor command hint for the prediction step."""
    command:        str   = Field(..., description="e.g. 'forward', 'left', 'stop'")
    speed_fraction: float = Field(default=1.0, ge=0.0, le=1.0,
                                  description="Speed as 0–1 fraction of max")


class PoseResetRequest(BaseModel):
    x:     float = 0.0
    y:     float = 0.0
    theta: float = 0.0


class MarkerMapEntry(BaseModel):
    x:     float
    y:     float
    alpha: float


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@router.post("/aruco_update")
async def aruco_update(body: ArucoUpdateRequest) -> Dict[str, Any]:
    """Receive an ArUco detection and apply it as an EKF measurement update.

    Called by video/video_server.py (Flask process) when a marker is detected
    with valid rvec/tvec (requires ARUCO_MARKER_LENGTH and ARUCO_CALIBRATION_FILE
    environment variables to be set on the video server side).

    Returns the updated pose so the caller can log it.
    """
    rvec = np.array(body.rvec, dtype=float)
    tvec = np.array(body.tvec, dtype=float)

    applied = _filter.update_from_aruco(
        marker_id=body.marker_id,
        rvec=rvec,
        tvec=tvec,
    )

    pose = _filter.get_pose()
    return {
        "ok": True,
        "update_applied": applied,
        "reason": "ok" if applied else "marker_not_in_map",
        "pose": {
            "x":         pose.x,
            "y":         pose.y,
            "theta_rad": pose.theta,
            "theta_deg": math.degrees(pose.theta),
            "quality":   pose.quality,
        },
        "correction_count": _filter.correction_count,
    }


@router.post("/command")
async def command_update(body: CommandUpdateRequest) -> Dict[str, Any]:
    """Update the velocity hint used by the prediction loop.

    Call this from movement_routes.py / ws_movement whenever a new direction
    command is issued.  This keeps the prediction loop aware of what the robot
    is doing without needing real encoder feedback.
    """
    global _last_v, _last_omega, _last_speed_fraction

    vw = _CMD_TO_VW.get(body.command.lower().strip())
    if vw is None:
        # Unknown command — treat as stop
        _last_v     = 0.0
        _last_omega = 0.0
    else:
        v_frac, w_frac = vw
        _last_v     = v_frac     * _V_MAX
        _last_omega = w_frac     * _W_MAX

    _last_speed_fraction = body.speed_fraction
    return {"ok": True, "v": _last_v * _last_speed_fraction, "omega": _last_omega * _last_speed_fraction}


@router.get("/pose")
async def get_pose() -> Dict[str, Any]:
    """Return the current SE(2) pose estimate.

    Response fields:
      x, y          — position in metres (relative to where filter was reset)
      theta_rad     — heading in radians, wrapped to [-π, π]
      theta_deg     — heading in degrees
      covariance    — 3×3 covariance matrix (list of lists)
      quality       — 0–1 confidence score (decays without ArUco corrections)
      correction_count — total ArUco corrections applied since last reset
      last_marker_seen — most recently used marker ID (null if none)
    """
    pose = _filter.get_pose()
    return {
        "x":               pose.x,
        "y":               pose.y,
        "theta_rad":       pose.theta,
        "theta_deg":       math.degrees(pose.theta),
        "covariance":      pose.covariance,
        "quality":         pose.quality,
        "correction_count": _filter.correction_count,
        "last_marker_seen": _filter.last_marker_seen,
        "ts":              pose.timestamp,
    }


@router.post("/reset")
async def reset_pose(body: PoseResetRequest) -> Dict[str, Any]:
    """Reset the filter to a known pose (e.g. at startup or after relocation)."""
    _filter.reset(x=body.x, y=body.y, theta=body.theta)
    log.info("Pose reset to x=%.3f y=%.3f θ=%.3f", body.x, body.y, body.theta)
    return {"ok": True, "pose": {"x": body.x, "y": body.y, "theta_rad": body.theta}}


@router.post("/marker_map")
async def update_marker_map(entries: Dict[str, MarkerMapEntry]) -> Dict[str, Any]:
    """Replace the marker map at runtime.

    Example body:
      { "7":  {"x": 1.0, "y": 0.0, "alpha": 3.14159},
        "12": {"x": 0.0, "y": 2.0, "alpha": 1.5708}  }
    """
    new_map = {
        int(k): MarkerPose(x=v.x, y=v.y, alpha=v.alpha)
        for k, v in entries.items()
    }
    _filter.set_marker_map(new_map)
    log.info("Marker map updated: %d markers", len(new_map))
    return {"ok": True, "marker_count": len(new_map)}


@router.get("/status")
async def localization_status() -> Dict[str, Any]:
    """Summary of the localization system state — useful for debugging."""
    pose = _filter.get_pose()
    return {
        "filter_active":    True,
        "predict_hz":       PREDICT_HZ,
        "last_v":           _last_v * _last_speed_fraction,
        "last_omega":       _last_omega * _last_speed_fraction,
        "correction_count": _filter.correction_count,
        "last_marker_seen": _filter.last_marker_seen,
        "pose_quality":     pose.quality,
        "covariance_trace": float(np.trace(np.array(pose.covariance))),
    }
