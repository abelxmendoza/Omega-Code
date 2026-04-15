"""SE(2) Extended Kalman Filter for 2D robot pose estimation.

State vector:  x = [px, py, θ]
Covariance:    P  (3×3 symmetric positive definite)

Prediction uses differential-drive kinematics fed by the last known motor
command (linear velocity v, angular velocity ω).  Correction is a direct
SE(2) observation from ArUco marker pose computation — the measurement model
h(x) = x so the Jacobian H = I₃, keeping the update step numerically stable.

This module is independent of hardware/sensor_fusion.py, which remains the
1-D forward-distance filter for obstacle avoidance.  The two filters serve
different purposes and should not be merged.

Thread safety: all public methods acquire self._lock.  The background
prediction loop in api/localization_routes.py calls predict(); the
FastAPI request handler calls update().  Both run from different threads.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Public data types
# ---------------------------------------------------------------------------

@dataclass
class SE2Pose:
    """Robot pose estimate with uncertainty."""
    x:         float          # metres
    y:         float          # metres
    theta:     float          # radians, wrapped to [-π, π]
    covariance: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    timestamp: float = 0.0
    quality:   float = 0.0    # 0–1; 1 = very confident


@dataclass
class MarkerPose:
    """Known world-frame pose of a single ArUco marker.

    x, y   – marker centre position in metres (robot's world frame origin)
    alpha  – marker facing direction in radians (normal vector direction;
             0 = marker faces +X, π/2 = faces +Y)
    """
    x:     float
    y:     float
    alpha: float


# ---------------------------------------------------------------------------
# SE(2) EKF
# ---------------------------------------------------------------------------

# Process noise — accounts for drift in dead-reckoning between corrections.
# Tune: larger = less trust in the kinematic model = faster covariance growth.
_DEFAULT_Q = np.diag([0.04, 0.04, 0.02])   # (0.2m)² / (0.14rad)² per step

# Measurement noise — accounts for ArUco pose estimation errors.
# Tune: larger = less trust in ArUco = smaller corrections.
_DEFAULT_R = np.diag([0.01, 0.01, 0.005])  # (0.1m)² / (0.07rad)²


class SE2PoseFilter:
    """Extended Kalman Filter for SE(2) pose estimation.

    Usage::

        filt = SE2PoseFilter(marker_map={7: MarkerPose(1.0, 0.0, math.pi)})

        # call at ~20 Hz from a background loop
        filt.predict(v=0.3, omega=0.0, dt=0.05)

        # call when ArUco detection arrives
        filt.update_from_aruco(marker_id=7, rvec=rvec, tvec=tvec)

        pose = filt.get_pose()
    """

    def __init__(
        self,
        marker_map: Optional[Dict[int, MarkerPose]] = None,
        Q: Optional[np.ndarray] = None,
        R: Optional[np.ndarray] = None,
        initial_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self._lock = threading.Lock()

        # State
        self._x = np.array(list(initial_pose), dtype=float)   # [px, py, θ]
        self._P = np.diag([1.0, 1.0, 0.5])                    # initial covariance

        # Noise matrices
        self._Q = Q if Q is not None else _DEFAULT_Q.copy()
        self._R = R if R is not None else _DEFAULT_R.copy()

        # Known marker world positions: {marker_id: MarkerPose}
        self._marker_map: Dict[int, MarkerPose] = marker_map or {}

        # Bookkeeping
        self._last_update_ts: float = 0.0
        self._correction_count: int = 0
        self._last_marker_seen: Optional[int] = None

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def predict(self, v: float, omega: float, dt: float) -> None:
        """Propagate state using the differential-drive kinematic model.

        Args:
            v:      linear velocity (m/s), positive = forward
            omega:  angular velocity (rad/s), positive = CCW
            dt:     time step (seconds)
        """
        if dt <= 0:
            return

        with self._lock:
            px, py, theta = self._x

            # State transition (Euler integration)
            self._x[0] = px    + v * math.cos(theta) * dt
            self._x[1] = py    + v * math.sin(theta) * dt
            self._x[2] = theta + omega * dt
            self._x[2] = self._wrap_angle(self._x[2])

            # Jacobian of f w.r.t. state (linearised around current theta)
            F = np.array([
                [1.0, 0.0, -v * math.sin(theta) * dt],
                [0.0, 1.0,  v * math.cos(theta) * dt],
                [0.0, 0.0,  1.0                      ],
            ], dtype=float)

            # Covariance propagation:  P⁻ = F·P·Fᵀ + Q
            self._P = F @ self._P @ F.T + self._Q * (dt / 0.05)

    def update_from_aruco(
        self,
        marker_id: int,
        rvec: np.ndarray,
        tvec: np.ndarray,
    ) -> bool:
        """Correct pose estimate using an ArUco marker detection.

        The marker must be in self._marker_map or the update is ignored.

        Args:
            marker_id: ArUco marker ID
            rvec:      (3,) or (3,1) Rodrigues rotation vector (marker→camera)
            tvec:      (3,) or (3,1) translation vector (marker origin in camera frame, metres)

        Returns:
            True if the update was applied; False if marker not in map or math failed.
        """
        if marker_id not in self._marker_map:
            return False

        try:
            z_x, z_y, z_theta = self._aruco_to_world_pose(
                marker_id, np.asarray(rvec, dtype=float).ravel(),
                np.asarray(tvec, dtype=float).ravel()
            )
        except Exception:
            return False

        self._apply_update(z_x, z_y, z_theta)
        self._last_marker_seen = marker_id
        self._last_update_ts = time.monotonic()
        return True

    def get_pose(self) -> SE2Pose:
        """Return a snapshot of the current pose estimate."""
        with self._lock:
            # Quality combines two factors:
            #   cov_q  — covariance trace (how uncertain is the estimate geometrically?)
            #            → 1.0 when trace≈0 (tight posterior), → 0 as uncertainty grows
            #   stale_q — time since last ArUco correction (only used after first fix)
            #            → 1.0 right after correction, → 0 after 60 s with no new fix
            #
            # Before any ArUco correction the filter is dead-reckoning only; quality
            # is capped at 0.5 to signal "alive but no absolute reference".
            trace  = float(np.trace(self._P))
            cov_q  = 1.0 / (1.0 + trace * 0.5)   # 1/(1+x) decay; trace 2 → 0.5

            if self._last_update_ts > 0:
                staleness = time.monotonic() - self._last_update_ts
                stale_q   = max(0.0, 1.0 - staleness / 60.0)
                quality   = cov_q * stale_q
            else:
                # Dead-reckoning only — cap at 0.5 to indicate no absolute reference
                quality = cov_q * 0.5

            quality = float(max(0.0, min(1.0, quality)))
            return SE2Pose(
                x=float(self._x[0]),
                y=float(self._x[1]),
                theta=float(self._x[2]),
                covariance=self._P.tolist(),
                timestamp=time.monotonic(),
                quality=quality,
            )

    def reset(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
    ) -> None:
        """Reset pose to given values and reinitialise covariance."""
        with self._lock:
            self._x = np.array([x, y, theta], dtype=float)
            self._P = np.diag([1.0, 1.0, 0.5])
            self._correction_count = 0
            self._last_update_ts = 0.0

    def set_marker_map(self, marker_map: Dict[int, MarkerPose]) -> None:
        """Replace the known marker map at runtime (e.g. from a config file)."""
        with self._lock:
            self._marker_map = dict(marker_map)

    @property
    def correction_count(self) -> int:
        return self._correction_count

    @property
    def last_marker_seen(self) -> Optional[int]:
        return self._last_marker_seen

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #

    def _apply_update(self, z_x: float, z_y: float, z_theta: float) -> None:
        """EKF measurement update.  Measurement model h(x) = x → H = I₃."""
        with self._lock:
            z = np.array([z_x, z_y, z_theta], dtype=float)

            # Innovation — wrap angle component
            y = z - self._x
            y[2] = self._wrap_angle(y[2])

            # H = I₃ → simplifies all matrix products
            S = self._P + self._R                         # innovation covariance
            K = self._P @ np.linalg.inv(S)               # Kalman gain

            self._x = self._x + K @ y
            self._x[2] = self._wrap_angle(self._x[2])

            I = np.eye(3, dtype=float)
            self._P = (I - K) @ self._P

            # Enforce symmetry to prevent numerical drift
            self._P = (self._P + self._P.T) / 2.0

            self._correction_count += 1

    def _aruco_to_world_pose(
        self,
        marker_id: int,
        rvec: np.ndarray,   # shape (3,)
        tvec: np.ndarray,   # shape (3,)
    ) -> Tuple[float, float, float]:
        """Convert ArUco rvec/tvec to robot world pose.

        Coordinate convention (OpenCV estimatePoseSingleMarkers):
          rvec — Rodrigues vector: rotation FROM marker frame TO camera frame
          tvec — translation: marker origin expressed IN camera frame

        Camera frame (robot forward-facing camera at robot centre):
          +Z = forward (depth into scene)
          +X = right
          +Y = down

        Marker assumed vertical (mounted on a wall), flat on the floor plane
        not supported.  Works best when the marker normal faces the robot.

        Returns: (robot_x, robot_y, robot_theta) in world frame (metres, radians).
        """
        try:
            import cv2
        except ImportError:
            raise RuntimeError("OpenCV required for ArUco pose conversion")

        mp = self._marker_map[marker_id]

        # Rotation matrix: marker → camera
        R_mc, _ = cv2.Rodrigues(rvec)

        # Camera (robot) position in marker frame:
        #   R_cm = R_mc.T  (orthogonal matrix inverse = transpose)
        #   t_cm = -R_mc.T @ tvec
        R_cm = R_mc.T
        t_cm = -R_cm @ tvec   # camera origin in marker frame (3,)

        # 2D projection: use marker X-axis (t_cm[0]) and normal direction (t_cm[2]).
        # Marker coordinate frame (vertical marker facing robot):
        #   marker +Z = normal vector pointing away from wall towards robot
        #   marker +X = horizontal right (along wall)
        #   marker +Y = up
        #
        # In the marker frame: camera is at (t_cm[0], t_cm[1], t_cm[2])
        # For 2D floor navigation we use (t_cm[0], t_cm[2]) — lateral and depth.
        lateral = float(t_cm[0])   # camera's X offset in marker frame
        depth   = float(t_cm[2])   # camera's distance from marker plane

        # World pose: rotate marker-frame offset by marker world orientation
        ma = mp.alpha
        robot_x = mp.x + depth * math.cos(ma) - lateral * math.sin(ma)
        robot_y = mp.y + depth * math.sin(ma) + lateral * math.cos(ma)

        # Robot heading from R_mc:
        # R_mc row 2 = [R_mc[2,0], R_mc[2,1], R_mc[2,2]]
        # R_mc[2, 2] = cos(angle between camera Z and marker Z)
        # R_mc[2, 0] = component of camera Z along marker X
        # The robot heading relative to marker facing direction:
        heading_offset = math.atan2(-R_mc[2, 0], R_mc[2, 2])
        # When robot faces the marker squarely, heading_offset ≈ π
        # World heading:
        robot_theta = self._wrap_angle(ma + heading_offset + math.pi)

        return robot_x, robot_y, robot_theta

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-π, π]."""
        return math.atan2(math.sin(angle), math.cos(angle))


__all__ = ["SE2PoseFilter", "SE2Pose", "MarkerPose"]
