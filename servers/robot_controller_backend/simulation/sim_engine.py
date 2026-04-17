"""Simulated robot engine for Omega-1.

Runs differential-drive kinematics in software and synthesises geometrically
correct ArUco rvec/tvec observations that are posted directly to the real
/localization/aruco_update endpoint.  The real SE(2) EKF receives these
observations exactly as it would receive them from a physical camera, so the
full localization → /localization/pose → UI pipeline works without hardware.

Coordinate conventions (must match pose_ekf._aruco_to_world_pose):
  World frame:  +X east, +Y north, θ=0 facing +X, positive CCW
  Camera frame: +Z forward (into scene), +X right, +Y down  [OpenCV standard]
  Marker frame: +Z normal (faces robot), +X right-along-wall, +Y down  [OpenCV ArUco]

rvec/tvec synthesis derivation
-------------------------------
Given robot at (rx, ry, rtheta) and marker at (mx, my, malpha):

  # Marker-frame coordinates of the robot (camera)
  dx      = rx - mx
  dy      = ry - my
  depth   = dx*cos(ma) + dy*sin(ma)      # along marker normal (+Z)
  lateral = -dx*sin(ma) + dy*cos(ma)     # along marker +X (horizontal)

  # Rotation angle of camera relative to marker (rotation around marker Y axis)
  psi = rtheta - π - malpha              # satisfies EKF heading recovery

  # Rotation matrix R_mc (marker→camera) = Ry(psi)
  # Encoded as Rodrigues: rvec = [0, psi, 0]

  # Translation: marker origin in camera frame
  tvec[0] = -(cos(psi)*lateral + sin(psi)*depth)
  tvec[1] = 0.0
  tvec[2] =   sin(psi)*lateral - cos(psi)*depth

These values, when fed to pose_ekf._aruco_to_world_pose, recover (rx, ry, rtheta)
exactly (plus any added noise).
"""

from __future__ import annotations

import asyncio
import logging
import math
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

log = logging.getLogger("sim_engine")


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class MarkerWorldPose:
    """World-frame pose of a simulated ArUco marker."""
    x:     float   # metres
    y:     float   # metres
    alpha: float   # radians — direction marker normal faces (0 = faces +X)


@dataclass
class Waypoint:
    x:     float
    y:     float
    label: str = ""


@dataclass
class SimScenario:
    name:          str
    description:   str
    initial_pose:  Tuple[float, float, float]           # x, y, theta (rad)
    marker_map:    Dict[int, Tuple[float, float, float]] # id → (x, y, alpha)
    waypoints:     List[Tuple[float, float]]             # [(x, y), ...]
    nav_params:    Dict = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Built-in scenarios
# ---------------------------------------------------------------------------

BUILTIN_SCENARIOS: Dict[str, SimScenario] = {
    "marker_circuit": SimScenario(
        name="Marker Circuit",
        description="Robot navigates between 4 ArUco markers at corners of a 2×2 m square",
        initial_pose=(0.0, 0.0, 0.0),
        marker_map={
            0: (0.0, 0.0, 0.0),          # origin, faces +X
            1: (2.0, 0.0, math.pi),       # right wall, faces -X
            2: (2.0, 2.0, math.pi / 2),   # far-right, faces +Y... wait, faces robot
            3: (0.0, 2.0, -math.pi / 2),  # far-left, faces -Y
        },
        waypoints=[(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)],
        nav_params={"max_v": 0.30, "max_omega": 0.80, "goal_tolerance": 0.15},
    ),
    "straight_line": SimScenario(
        name="Straight Line",
        description="Robot drives 3 m forward between two markers",
        initial_pose=(0.0, 0.0, 0.0),
        marker_map={
            0: (0.0, 0.0, 0.0),
            1: (3.0, 0.0, math.pi),
        },
        waypoints=[(3.0, 0.0), (0.0, 0.0)],
        nav_params={"max_v": 0.30, "max_omega": 0.80, "goal_tolerance": 0.15},
    ),
    "three_point_turn": SimScenario(
        name="Three Point",
        description="L-shaped path with three markers testing heading changes",
        initial_pose=(0.0, 0.0, 0.0),
        marker_map={
            0: (0.0, 0.0, 0.0),
            1: (2.0, 0.0, math.pi / 2),
            2: (2.0, 2.0, math.pi),
        },
        waypoints=[(2.0, 0.0), (2.0, 2.0), (0.0, 0.0)],
        nav_params={"max_v": 0.25, "max_omega": 1.0, "goal_tolerance": 0.15},
    ),
}


# ---------------------------------------------------------------------------
# Simulated robot
# ---------------------------------------------------------------------------

# Sensor simulation parameters
_CAMERA_FOV_HALF_RAD = math.radians(40.0)  # ±40° horizontal FOV half-angle
_CAMERA_MAX_RANGE_M  = 3.0                  # metres
_CAMERA_MIN_RANGE_M  = 0.15                 # below this marker fills the frame

# Noise — mimics realistic OV5647 + ArUco measurement noise
_MOTION_NOISE_V     = 0.010   # m/s  Gaussian std dev on linear speed
_MOTION_NOISE_OMEGA = 0.020   # rad/s Gaussian std dev on angular speed
_ARUCO_NOISE_POS    = 0.020   # m    Gaussian std dev on tvec components
_ARUCO_NOISE_ANGLE  = 0.015   # rad  Gaussian std dev on rvec angle

# Navigation P-controller gains
_K_V          = 0.50   # linear velocity gain
_K_OMEGA      = 1.20   # angular velocity gain
_MAX_V        = 0.35   # m/s
_MAX_OMEGA    = 1.00   # rad/s
_GOAL_TOL_DEF = 0.15   # m  default goal tolerance
_HEADING_TOL  = 0.12   # rad — rotate-in-place threshold (~7°)


def _wrap(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _clip(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


class SimulatedRobot:
    """Software robot that feeds real-format ArUco observations into the EKF.

    Thread-safe: all state access is protected by self._lock.
    The step() coroutine must be called from an asyncio event loop.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()

        # Kinematic state
        self.x:     float = 0.0
        self.y:     float = 0.0
        self.theta: float = 0.0   # radians, world frame

        # Velocity commands
        self.v:     float = 0.0   # m/s  — overridden by navigator when mission active
        self.omega: float = 0.0   # rad/s

        # World map of markers
        self._marker_map: Dict[int, MarkerWorldPose] = {}

        # Mission
        self._waypoints:      List[Waypoint] = []
        self._mission_index:  int            = 0
        self._goal_tolerance: float          = _GOAL_TOL_DEF
        self._max_v:          float          = _MAX_V
        self._max_omega:      float          = _MAX_OMEGA
        self._mission_active: bool           = False

        # Stats
        self.total_steps:       int   = 0
        self.aruco_posts_sent:  int   = 0
        self.aruco_posts_ok:    int   = 0

        # Mission event broadcast (set by sim_routes, called from event loop)
        self._event_callback:         Optional[Callable[[dict], None]] = None
        self._mission_complete:       bool = False
        self._mission_started_emitted: bool = False

    # ------------------------------------------------------------------
    # Public control API
    # ------------------------------------------------------------------

    def set_event_callback(self, cb: Optional[Callable[[dict], None]]) -> None:
        """Register a callback for mission events (called from asyncio event loop)."""
        self._event_callback = cb

    def set_velocity(self, v: float, omega: float) -> None:
        """Override velocity directly (disables mission nav for this tick)."""
        with self._lock:
            self.v     = _clip(v,     -self._max_v,     self._max_v)
            self.omega = _clip(omega, -self._max_omega, self._max_omega)

    def teleport(self, x: float, y: float, theta: float) -> None:
        """Instantly move the robot to a given world pose."""
        with self._lock:
            self.x     = x
            self.y     = y
            self.theta = _wrap(theta)
        log.info("Teleported to x=%.3f y=%.3f θ=%.3f°", x, y, math.degrees(theta))

    def load_marker_map(self, marker_map: Dict[int, MarkerWorldPose]) -> None:
        with self._lock:
            self._marker_map = dict(marker_map)
        log.info("Sim marker map loaded: %d markers", len(marker_map))

    def load_mission(
        self,
        waypoints: List[Waypoint],
        goal_tolerance: float = _GOAL_TOL_DEF,
        max_v: float = _MAX_V,
        max_omega: float = _MAX_OMEGA,
    ) -> None:
        with self._lock:
            self._waypoints      = list(waypoints)
            self._mission_index  = 0
            self._goal_tolerance = goal_tolerance
            self._max_v          = max_v
            self._max_omega      = max_omega
            self._mission_active          = bool(waypoints)
            self._mission_complete        = False
            self._mission_started_emitted = False
        log.info("Mission loaded: %d waypoints", len(waypoints))

    def abort_mission(self) -> None:
        with self._lock:
            was_active           = self._mission_active
            idx                  = self._mission_index
            self._mission_active = False
            self.v               = 0.0
            self.omega           = 0.0
        if was_active and self._event_callback:
            self._event_callback({
                "type":           "mission_event",
                "event":          "mission_aborted",
                "waypoint_index": idx,
                "timestamp":      time.time(),
            })

    def get_status(self) -> dict:
        with self._lock:
            wp_label = ""
            if self._waypoints and self._mission_index < len(self._waypoints):
                wp = self._waypoints[self._mission_index]
                wp_label = wp.label
            remaining = len(self._waypoints) - self._mission_index if self._mission_active else 0
            return {
                "x":                  round(self.x,     4),
                "y":                  round(self.y,     4),
                "theta_rad":          round(self.theta, 4),
                "theta_deg":          round(math.degrees(self.theta), 2),
                "v":                  round(self.v,     4),
                "omega":              round(self.omega, 4),
                "mission_active":     self._mission_active,
                "waypoint_index":     self._mission_index,
                "waypoints_total":    len(self._waypoints),
                "waypoints_remaining":remaining,
                "current_goal":       wp_label,
                "marker_count":       len(self._marker_map),
                "total_steps":        self.total_steps,
                "aruco_posts_sent":   self.aruco_posts_sent,
                "aruco_posts_ok":     self.aruco_posts_ok,
                "mission_complete":   self._mission_complete,
            }

    # ------------------------------------------------------------------
    # Main simulation step (called at ~20 Hz)
    # ------------------------------------------------------------------

    async def step(self, dt: float) -> None:
        """Advance simulation by dt seconds.

        1. Run navigation controller (updates self.v, self.omega)
        2. Integrate kinematics with noise
        3. Check marker visibility and post ArUco updates
        """
        if dt <= 0 or dt > 1.0:
            return

        with self._lock:
            # 1. Navigation P-controller — returns events to dispatch after lock
            pending_events: List[dict] = []
            if self._mission_active:
                pending_events = self._navigate_locked()

            # 2. Differential-drive kinematics with motion noise
            v_noisy     = self.v     + random.gauss(0, _MOTION_NOISE_V)
            omega_noisy = self.omega + random.gauss(0, _MOTION_NOISE_OMEGA)

            self.x     += v_noisy * math.cos(self.theta) * dt
            self.y     += v_noisy * math.sin(self.theta) * dt
            self.theta  = _wrap(self.theta + omega_noisy * dt)

            # Snapshot for ArUco check (release lock during async I/O)
            rx, ry, rtheta = self.x, self.y, self.theta
            marker_snap    = dict(self._marker_map)

            self.total_steps += 1

        # 3. Check which markers are visible and post observations
        await self._post_visible_markers(rx, ry, rtheta, marker_snap)

        # 4. Dispatch mission events (in event loop, after lock release — safe for asyncio.Queue)
        if pending_events and self._event_callback:
            for evt in pending_events:
                self._event_callback(evt)

    # ------------------------------------------------------------------
    # Navigation controller (called with self._lock held)
    # ------------------------------------------------------------------

    def _navigate_locked(self) -> List[dict]:
        """P-controller: rotate to face goal, then drive.

        Returns a list of mission event dicts to be dispatched after the
        lock is released (so put_nowait on asyncio queues is safe).
        """
        events: List[dict] = []

        # Guard: index out of bounds → mission is over
        if not self._waypoints or self._mission_index >= len(self._waypoints):
            if self._mission_active and not self._mission_complete:
                self._mission_complete = True
                events.append({
                    "type":           "mission_event",
                    "event":          "mission_completed",
                    "waypoint_index": self._mission_index,
                    "timestamp":      time.time(),
                })
            self._mission_active = False
            self.v     = 0.0
            self.omega = 0.0
            return events

        # Emit mission_started exactly once per mission
        if not self._mission_started_emitted:
            self._mission_started_emitted = True
            events.append({
                "type":           "mission_event",
                "event":          "mission_started",
                "waypoint_index": 0,
                "waypoints_total": len(self._waypoints),
                "timestamp":      time.time(),
            })

        goal = self._waypoints[self._mission_index]
        dx   = goal.x - self.x
        dy   = goal.y - self.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self._goal_tolerance:
            log.info(
                "Waypoint %d/%d reached: '%s' at (%.2f, %.2f)",
                self._mission_index + 1,
                len(self._waypoints),
                goal.label,
                goal.x,
                goal.y,
            )
            events.append({
                "type":           "mission_event",
                "event":          "waypoint_reached",
                "waypoint_index": self._mission_index,
                "waypoints_total": len(self._waypoints),
                "timestamp":      time.time(),
            })
            self._mission_index += 1
            self.v     = 0.0
            self.omega = 0.0
            # Check if that was the last waypoint
            if self._mission_index >= len(self._waypoints):
                self._mission_active  = False
                self._mission_complete = True
                events.append({
                    "type":           "mission_event",
                    "event":          "mission_completed",
                    "waypoint_index": self._mission_index,
                    "waypoints_total": len(self._waypoints),
                    "timestamp":      time.time(),
                })
            return events

        heading_to_goal = math.atan2(dy, dx)
        heading_error   = _wrap(heading_to_goal - self.theta)

        if abs(heading_error) > _HEADING_TOL:
            # Rotate in place
            self.v     = 0.0
            self.omega = _clip(
                _K_OMEGA * heading_error, -self._max_omega, self._max_omega
            )
        else:
            # Drive forward with heading correction
            self.v = _clip(
                _K_V * dist, 0.05, self._max_v  # 0.05 m/s minimum creep
            )
            self.omega = _clip(
                _K_OMEGA * heading_error, -self._max_omega, self._max_omega
            )

        return events

    # ------------------------------------------------------------------
    # ArUco observation synthesis
    # ------------------------------------------------------------------

    async def _post_visible_markers(
        self,
        rx: float,
        ry: float,
        rtheta: float,
        marker_map: Dict[int, MarkerWorldPose],
    ) -> None:
        """Synthesise rvec/tvec for each visible marker and post to EKF."""
        for marker_id, mp in marker_map.items():
            rvec, tvec = self._synthesise_observation(rx, ry, rtheta, mp)
            if rvec is None:
                continue  # not visible

            self.aruco_posts_sent += 1
            ok = await self._call_aruco_update(marker_id, rvec, tvec)
            if ok:
                self.aruco_posts_ok += 1

    def _synthesise_observation(
        self,
        rx: float,
        ry: float,
        rtheta: float,
        mp: MarkerWorldPose,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Return (rvec, tvec) for a marker, or (None, None) if not in view.

        See module docstring for full derivation.
        """
        # --- Visibility check ------------------------------------------
        dx   = mp.x - rx
        dy   = mp.y - ry
        dist = math.sqrt(dx * dx + dy * dy)

        if not (_CAMERA_MIN_RANGE_M < dist < _CAMERA_MAX_RANGE_M):
            return None, None

        # Angle of marker relative to robot heading
        angle_to_marker  = math.atan2(dy, dx)
        relative_bearing = _wrap(angle_to_marker - rtheta)

        if abs(relative_bearing) > _CAMERA_FOV_HALF_RAD:
            return None, None  # outside horizontal FOV

        # --- Marker-frame coordinates of camera ------------------------
        # (robot/camera origin expressed in marker coordinate frame)
        cam_dx   = rx - mp.x
        cam_dy   = ry - mp.y
        depth    = cam_dx * math.cos(mp.alpha) + cam_dy * math.sin(mp.alpha)
        lateral  = -cam_dx * math.sin(mp.alpha) + cam_dy * math.cos(mp.alpha)

        if depth <= 0:
            # Robot is behind the marker plane — not detectable
            return None, None

        # --- Add measurement noise -------------------------------------
        noisy_lateral = lateral + random.gauss(0, _ARUCO_NOISE_POS)
        noisy_depth   = max(0.05, depth + random.gauss(0, _ARUCO_NOISE_POS))

        # Rotation angle of camera frame relative to marker frame
        # (rotation around marker +Y axis)
        psi        = rtheta - math.pi - mp.alpha
        noisy_psi  = psi + random.gauss(0, _ARUCO_NOISE_ANGLE)

        # --- Rodrigues vector: rvec = Y-axis rotation by noisy_psi ----
        rvec = np.array([0.0, noisy_psi, 0.0], dtype=float)

        # --- Translation vector: marker origin in camera frame ---------
        # Derived from: t_cm = [lateral, 0, depth]  and  tvec = -R_mc @ t_cm
        # where R_mc = Ry(psi).
        cp = math.cos(noisy_psi)
        sp = math.sin(noisy_psi)
        tvec = np.array([
            -(cp * noisy_lateral + sp * noisy_depth),
            0.0,
            sp * noisy_lateral - cp * noisy_depth,
        ], dtype=float)

        return rvec, tvec

    async def _call_aruco_update(
        self,
        marker_id: int,
        rvec: np.ndarray,
        tvec: np.ndarray,
    ) -> bool:
        """Post an ArUco detection directly to the localization route handler.

        Calls the FastAPI handler function directly (in-process, no HTTP
        overhead) using the same Pydantic model the real camera uses.
        This ensures the sim exercises the identical code path.
        """
        try:
            from api.localization_routes import aruco_update, ArucoUpdateRequest
            body = ArucoUpdateRequest(
                marker_id=marker_id,
                rvec=rvec.tolist(),
                tvec=tvec.tolist(),
                timestamp=time.time(),
            )
            result = await aruco_update(body)
            return result.get("update_applied", False)
        except Exception as exc:
            log.debug("aruco_update call failed for marker %d: %s", marker_id, exc)
            return False
