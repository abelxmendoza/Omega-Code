"""FastAPI routes for the software-simulated robot engine.

Only mounted when the environment variable SIM_MODE=1 is set.
Provides full control over the simulated robot: start/stop the sim loop,
teleport the robot, inject velocity commands, load missions and scenarios,
and query the current simulation state.

All ArUco observations generated here are posted to the real
/localization/aruco_update endpoint, driving the real SE(2) EKF.
Nothing in the downstream localization stack is mocked.

Usage:
    SIM_MODE=1 uvicorn main_api:app --host 0.0.0.0 --port 8000
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import threading
import time
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field

from simulation.sim_engine import (
    BUILTIN_SCENARIOS,
    MarkerWorldPose,
    SimulatedRobot,
    SimScenario,
    Waypoint,
)

log = logging.getLogger("sim_routes")

router = APIRouter(prefix="/sim", tags=["Simulation"])

# ---------------------------------------------------------------------------
# Module-level sim state (one robot, one loop per process)
# ---------------------------------------------------------------------------

_robot: Optional[SimulatedRobot] = None
_loop_task: Optional[asyncio.Task] = None
_sim_hz: float = 20.0
_started_at: Optional[float] = None

# ---------------------------------------------------------------------------
# Mission event broadcast — fan-out to all /ws/mission connections.
# Queues live here (independent of robot lifetime) so WebSocket clients
# survive robot restarts (e.g. scenario reloads).
# ---------------------------------------------------------------------------

_mission_queues: set = set()
_mq_lock = threading.Lock()


def subscribe_mission_events() -> asyncio.Queue:
    """Create and register a new event queue for a /ws/mission client."""
    q: asyncio.Queue = asyncio.Queue(maxsize=64)
    with _mq_lock:
        _mission_queues.add(q)
    return q


def unsubscribe_mission_events(q: asyncio.Queue) -> None:
    with _mq_lock:
        _mission_queues.discard(q)


def _broadcast_mission_event(event: dict) -> None:
    """Fan-out a mission event to every subscribed WebSocket queue.

    Always called from the asyncio event loop (sim step or HTTP handler),
    so asyncio.Queue.put_nowait() is safe.
    """
    with _mq_lock:
        queues = list(_mission_queues)
    for q in queues:
        try:
            q.put_nowait(event)
        except asyncio.QueueFull:
            pass  # client too slow — drop


def get_sim_status() -> Optional[dict]:
    """Return the current robot status dict, or None if sim is not running."""
    return _robot.get_status() if _robot is not None else None


# ---------------------------------------------------------------------------
# Background loop
# ---------------------------------------------------------------------------

async def _sim_loop() -> None:
    """Main simulation loop: advances the robot at _sim_hz."""
    global _robot
    interval = 1.0 / _sim_hz
    last_t   = time.monotonic()

    log.info("Sim loop started at %.0f Hz", _sim_hz)
    try:
        while True:
            await asyncio.sleep(interval)
            if _robot is None:
                continue
            now = time.monotonic()
            dt  = min(now - last_t, 0.5)   # cap at 0.5 s to survive scheduling hiccups
            last_t = now
            await _robot.step(dt)
    except asyncio.CancelledError:
        log.info("Sim loop stopped")
        raise


def _ensure_loop_running() -> None:
    """Start the background loop if it is not already running."""
    global _loop_task
    if _loop_task is None or _loop_task.done():
        _loop_task = asyncio.create_task(_sim_loop())


def _stop_loop() -> None:
    """Cancel the background loop."""
    global _loop_task
    if _loop_task and not _loop_task.done():
        _loop_task.cancel()
        _loop_task = None


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get_robot() -> SimulatedRobot:
    if _robot is None:
        raise HTTPException(status_code=409, detail="Simulation not started. POST /sim/start first.")
    return _robot


async def _push_marker_map_to_ekf(marker_map: Dict[int, MarkerWorldPose]) -> None:
    """Push the sim marker map to the EKF via its own route handler."""
    try:
        from api.localization_routes import update_marker_map, MarkerMapEntry
        entries = {
            str(mid): MarkerMapEntry(x=mp.x, y=mp.y, alpha=mp.alpha)
            for mid, mp in marker_map.items()
        }
        await update_marker_map(entries)
        log.info("EKF marker map updated with %d markers from sim", len(entries))
    except Exception as exc:
        log.warning("Failed to push marker map to EKF: %s", exc)


async def _reset_ekf_pose(x: float, y: float, theta: float) -> None:
    """Reset the EKF to the sim's initial pose."""
    try:
        from api.localization_routes import reset_pose, PoseResetRequest
        await reset_pose(PoseResetRequest(x=x, y=y, theta=theta))
        log.info("EKF pose reset to x=%.3f y=%.3f θ=%.3f", x, y, theta)
    except Exception as exc:
        log.warning("Failed to reset EKF pose: %s", exc)


# ---------------------------------------------------------------------------
# Request / response models
# ---------------------------------------------------------------------------

class StartRequest(BaseModel):
    x:       float = Field(default=0.0, description="Initial X position (metres)")
    y:       float = Field(default=0.0, description="Initial Y position (metres)")
    theta:   float = Field(default=0.0, description="Initial heading (radians)")
    hz:      float = Field(default=20.0, ge=1.0, le=50.0, description="Simulation frequency (Hz)")
    reset_ekf: bool = Field(default=True, description="Reset the EKF to the initial pose")


class StopRequest(BaseModel):
    pass


class TeleportRequest(BaseModel):
    x:     float = Field(..., description="Target X position (metres)")
    y:     float = Field(..., description="Target Y position (metres)")
    theta: float = Field(default=0.0, description="Target heading (radians)")
    reset_ekf: bool = Field(default=False, description="Also reset the EKF to this pose")


class VelocityRequest(BaseModel):
    v:     float = Field(..., ge=-1.0, le=1.0, description="Linear velocity (m/s)")
    omega: float = Field(..., ge=-3.15, le=3.15, description="Angular velocity (rad/s)")


class WaypointItem(BaseModel):
    x:     float
    y:     float
    label: str = ""


class MissionRequest(BaseModel):
    waypoints:      List[WaypointItem] = Field(..., min_length=1)
    goal_tolerance: float = Field(default=0.15, ge=0.02, le=2.0)
    max_v:          float = Field(default=0.30, ge=0.05, le=0.50)
    max_omega:      float = Field(default=0.80, ge=0.10, le=2.0)


class MarkerEntry(BaseModel):
    x:     float
    y:     float
    alpha: float = Field(default=0.0, description="Marker facing direction (radians)")


class MarkerMapRequest(BaseModel):
    markers: Dict[str, MarkerEntry] = Field(
        ...,
        description='Map of marker_id → pose. e.g. {"0": {"x":0,"y":0,"alpha":0}}'
    )


class ScenarioRequest(BaseModel):
    name: str = Field(..., description="Scenario name. GET /sim/scenarios for available list.")


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@router.post("/start")
async def sim_start(body: StartRequest) -> Dict[str, Any]:
    """Start (or restart) the simulation loop.

    Creates a fresh SimulatedRobot at the given initial pose.
    Optionally resets the EKF to the same pose so both start in sync.
    """
    global _robot, _started_at, _sim_hz

    # Stop any existing loop
    _stop_loop()

    _sim_hz = body.hz
    _robot  = SimulatedRobot()
    _robot.teleport(body.x, body.y, body.theta)
    _robot.set_event_callback(_broadcast_mission_event)
    _started_at = time.time()

    if body.reset_ekf:
        await _reset_ekf_pose(body.x, body.y, body.theta)

    _ensure_loop_running()

    log.info("Simulation started at (%.2f, %.2f, %.2f°) @ %.0f Hz",
             body.x, body.y, math.degrees(body.theta), _sim_hz)
    return {
        "ok": True,
        "status": "running",
        "initial_pose": {"x": body.x, "y": body.y, "theta_rad": body.theta},
        "hz": _sim_hz,
    }


@router.post("/stop")
async def sim_stop() -> Dict[str, Any]:
    """Stop the simulation loop and halt the robot."""
    global _robot, _started_at

    _stop_loop()

    if _robot:
        _robot.abort_mission()
        _robot.set_velocity(0.0, 0.0)

    _robot      = None
    _started_at = None

    log.info("Simulation stopped")
    return {"ok": True, "status": "stopped"}


@router.post("/pose")
async def sim_teleport(body: TeleportRequest) -> Dict[str, Any]:
    """Teleport the robot to an arbitrary world pose.

    Useful for injecting a disturbance mid-mission to test EKF recovery.
    """
    robot = _get_robot()
    robot.teleport(body.x, body.y, body.theta)

    if body.reset_ekf:
        await _reset_ekf_pose(body.x, body.y, body.theta)

    return {
        "ok": True,
        "pose": {
            "x": body.x,
            "y": body.y,
            "theta_rad": body.theta,
            "theta_deg": math.degrees(body.theta),
        },
        "ekf_reset": body.reset_ekf,
    }


@router.post("/velocity")
async def sim_velocity(body: VelocityRequest) -> Dict[str, Any]:
    """Manually inject velocity commands.

    Overrides the mission navigator for the current tick.
    Note: if a mission is active the navigator will resume on the next tick.
    To drive manually, call /sim/mission/abort first.
    """
    robot = _get_robot()
    robot.set_velocity(body.v, body.omega)
    return {"ok": True, "v": body.v, "omega": body.omega}


@router.post("/mission")
async def sim_mission(body: MissionRequest) -> Dict[str, Any]:
    """Load and start a waypoint mission.

    The robot's internal P-controller will navigate through each waypoint
    in order, using the current EKF pose as feedback.
    """
    robot = _get_robot()
    waypoints = [Waypoint(x=w.x, y=w.y, label=w.label) for w in body.waypoints]
    robot.load_mission(
        waypoints=waypoints,
        goal_tolerance=body.goal_tolerance,
        max_v=body.max_v,
        max_omega=body.max_omega,
    )
    return {
        "ok": True,
        "waypoints_loaded": len(waypoints),
        "goal_tolerance_m": body.goal_tolerance,
    }


@router.post("/mission/abort")
async def sim_mission_abort() -> Dict[str, Any]:
    """Abort the current mission and halt the robot."""
    robot = _get_robot()
    robot.abort_mission()
    return {"ok": True, "status": "mission aborted"}


@router.post("/markers")
async def sim_set_markers(body: MarkerMapRequest) -> Dict[str, Any]:
    """Load a custom marker map into both the sim and the EKF.

    The sim uses this map to decide which markers are visible.
    The EKF uses it to compute pose corrections.
    Both must have the same map or corrections will be wrong.
    """
    robot = _get_robot()

    marker_map = {
        int(k): MarkerWorldPose(x=v.x, y=v.y, alpha=v.alpha)
        for k, v in body.markers.items()
    }
    robot.load_marker_map(marker_map)
    await _push_marker_map_to_ekf(marker_map)

    return {"ok": True, "marker_count": len(marker_map)}


@router.post("/scenario")
async def sim_load_scenario(body: ScenarioRequest) -> Dict[str, Any]:
    """Load a built-in scenario: sets initial pose, marker map, and mission.

    Also resets the EKF to the scenario's initial pose and updates the EKF
    marker map, so the entire system is in sync from a clean state.

    GET /sim/scenarios to see what is available.
    """
    scenario = BUILTIN_SCENARIOS.get(body.name)
    if scenario is None:
        available = list(BUILTIN_SCENARIOS.keys())
        raise HTTPException(
            status_code=404,
            detail=f"Scenario '{body.name}' not found. Available: {available}",
        )

    global _robot, _started_at, _sim_hz
    _stop_loop()

    nav = scenario.nav_params
    ix, iy, itheta = scenario.initial_pose

    _robot = SimulatedRobot()
    _robot.teleport(ix, iy, itheta)
    _robot.set_event_callback(_broadcast_mission_event)
    _started_at = time.time()

    # Build marker map
    marker_map = {
        mid: MarkerWorldPose(x=x, y=y, alpha=a)
        for mid, (x, y, a) in scenario.marker_map.items()
    }
    _robot.load_marker_map(marker_map)

    # Build mission
    waypoints = [
        Waypoint(x=x, y=y, label=f"WP{i+1}")
        for i, (x, y) in enumerate(scenario.waypoints)
    ]
    _robot.load_mission(
        waypoints=waypoints,
        goal_tolerance=nav.get("goal_tolerance", 0.15),
        max_v=nav.get("max_v", 0.30),
        max_omega=nav.get("max_omega", 0.80),
    )

    # Sync EKF
    await _reset_ekf_pose(ix, iy, itheta)
    await _push_marker_map_to_ekf(marker_map)

    _ensure_loop_running()

    log.info("Scenario '%s' loaded and running", scenario.name)
    return {
        "ok":           True,
        "scenario":     scenario.name,
        "description":  scenario.description,
        "initial_pose": {"x": ix, "y": iy, "theta_rad": itheta},
        "marker_count": len(marker_map),
        "waypoints":    len(waypoints),
        "hz":           _sim_hz,
    }


@router.get("/scenarios")
async def sim_list_scenarios() -> Dict[str, Any]:
    """List all available built-in scenarios."""
    return {
        "scenarios": [
            {
                "name":         name,
                "description":  s.description,
                "marker_count": len(s.marker_map),
                "waypoints":    len(s.waypoints),
            }
            for name, s in BUILTIN_SCENARIOS.items()
        ]
    }


@router.get("/status")
async def sim_status() -> Dict[str, Any]:
    """Return the current simulation state.

    Returns a 'running: false' response (not an error) if sim is not started.
    """
    if _robot is None:
        return {
            "running":     False,
            "uptime_s":    0,
            "hz":          _sim_hz,
            "loop_active": _loop_task is not None and not _loop_task.done(),
        }

    status = _robot.get_status()
    status.update({
        "running":     True,
        "uptime_s":    round(time.time() - _started_at, 1) if _started_at else 0,
        "hz":          _sim_hz,
        "loop_active": _loop_task is not None and not _loop_task.done(),
    })
    return status


# ---------------------------------------------------------------------------
# Lifecycle helpers (called from main_api.py lifespan)
# ---------------------------------------------------------------------------

def shutdown_sim() -> None:
    """Clean shutdown — call from FastAPI lifespan on exit."""
    _stop_loop()
    log.info("Simulation engine shut down")
