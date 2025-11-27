"""
Movement V2 - Odometry Module

Tracks robot position and heading using dead reckoning.
Placeholder for future encoder-based odometry.
"""

import time
import math
import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass

# Performance: Use monotonic time
_monotonic = time.monotonic

# Performance: Pre-compute constants
_TWO_PI = 2.0 * math.pi
_RPM_TO_RAD_PER_SEC = math.pi / 30.0  # RPM * π/30 = rad/s
_STRAIGHT_THRESHOLD = 0.001  # Threshold for straight movement detection

logger = logging.getLogger(__name__)


@dataclass
class Pose:
    """Robot pose (position and orientation)"""
    x: float = 0.0          # X position (meters)
    y: float = 0.0          # Y position (meters)
    heading: float = 0.0    # Heading angle (radians, 0 = +X, + = CCW)
    timestamp: float = 0.0  # Timestamp of pose


class Odometry:
    """Optimized odometry with __slots__ for memory efficiency"""
    __slots__ = (
        'wheel_base', 'wheel_radius', 'ticks_per_rev', 'pose', 'linear_velocity',
        'angular_velocity', 'left_encoder_ticks', 'right_encoder_ticks',
        'last_left_ticks', 'last_right_ticks', 'total_distance'
    )
    
    def __init__(
        self,
        wheel_base: float = 0.2,      # Distance between wheels (meters)
        wheel_radius: float = 0.05,   # Wheel radius (meters)
        ticks_per_rev: int = 0        # Encoder ticks per revolution (0 = simulated)
    ):
        """
        Initialize odometry system.
        
        Args:
            wheel_base: Distance between left and right wheels (meters)
            wheel_radius: Wheel radius (meters)
            ticks_per_rev: Encoder ticks per wheel revolution (0 = use simulated)
        """
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.ticks_per_rev = ticks_per_rev
        
        # Current pose (use monotonic time)
        self.pose = Pose(timestamp=_monotonic())
        
        # Velocity tracking
        self.linear_velocity = 0.0    # m/s
        self.angular_velocity = 0.0   # rad/s
        
        # Encoder state (if available)
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        # Distance traveled
        self.total_distance = 0.0
        
        logger.info(
            f"Odometry initialized: wheel_base={wheel_base}m, "
            f"wheel_radius={wheel_radius}m, encoders={'enabled' if ticks_per_rev > 0 else 'simulated'}"
        )
    
    def update(
        self,
        left_rpm: float = 0.0,
        right_rpm: float = 0.0,
        dt: float = 0.1,
        left_ticks: Optional[int] = None,
        right_ticks: Optional[int] = None
    ) -> None:
        """
        Update odometry based on wheel speeds or encoder ticks.
        
        Args:
            left_rpm: Left wheel RPM (from telemetry or simulated)
            right_rpm: Right wheel RPM (from telemetry or simulated)
            dt: Time delta in seconds
            left_ticks: Left encoder ticks (if encoders available)
            right_ticks: Right encoder ticks (if encoders available)
        """
        if dt <= 0:
            return
        
        # Use encoder ticks if available, otherwise use RPM
        if self.ticks_per_rev > 0 and left_ticks is not None and right_ticks is not None:
            # Encoder-based odometry
            left_delta = left_ticks - self.last_left_ticks
            right_delta = right_ticks - self.last_right_ticks
            
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            
            # Performance: Pre-compute conversion factor
            ticks_to_distance = _TWO_PI * self.wheel_radius / self.ticks_per_rev
            left_distance = left_delta * ticks_to_distance
            right_distance = right_delta * ticks_to_distance
        else:
            # RPM-based odometry (simulated) - optimized conversion
            # Performance: Pre-compute conversion factor (RPM to m/s)
            rpm_to_ms = _TWO_PI * self.wheel_radius / 60.0
            left_velocity = left_rpm * rpm_to_ms
            right_velocity = right_rpm * rpm_to_ms
            
            left_distance = left_velocity * dt
            right_distance = right_velocity * dt
        
        # Calculate linear and angular velocities
        linear_distance = (left_distance + right_distance) / 2.0
        angular_distance = (right_distance - left_distance) / self.wheel_base
        
        # Update pose
        self.linear_velocity = linear_distance / dt if dt > 0 else 0.0
        self.angular_velocity = angular_distance / dt if dt > 0 else 0.0
        
        # Update position (differential drive kinematics) - optimized
        abs_angular = abs(angular_distance)
        if abs_angular < _STRAIGHT_THRESHOLD:
            # Straight movement (optimized: cache cos/sin)
            heading = self.pose.heading
            cos_h = math.cos(heading)
            sin_h = math.sin(heading)
            self.pose.x += linear_distance * cos_h
            self.pose.y += linear_distance * sin_h
        else:
            # Arc movement (optimized: cache trigonometric values)
            heading = self.pose.heading
            radius = linear_distance / angular_distance
            sin_h = math.sin(heading)
            cos_h = math.cos(heading)
            sin_h_new = math.sin(heading + angular_distance)
            cos_h_new = math.cos(heading + angular_distance)
            self.pose.x += radius * (sin_h_new - sin_h)
            self.pose.y -= radius * (cos_h_new - cos_h)
            self.pose.heading += angular_distance
        
        # Performance: Optimize heading normalization (only if needed)
        heading = self.pose.heading
        if heading > math.pi or heading < -math.pi:
            self.pose.heading = math.atan2(math.sin(heading), math.cos(heading))
        
        # Update distance traveled
        self.total_distance += abs(linear_distance)
        
        # Update timestamp (use monotonic time)
        self.pose.timestamp = _monotonic()
    
    def get_pose(self) -> Pose:
        """Get current robot pose"""
        return Pose(
            x=self.pose.x,
            y=self.pose.y,
            heading=self.pose.heading,
            timestamp=self.pose.timestamp
        )
    
    def get_velocity(self) -> Dict[str, float]:
        """Get current velocity"""
        return {
            "linear": self.linear_velocity,   # m/s
            "angular": self.angular_velocity  # rad/s
        }
    
    def reset(self, x: float = 0.0, y: float = 0.0, heading: float = 0.0) -> None:
        """
        Reset odometry to a new pose.
        
        Args:
            x: New X position (meters)
            y: New Y position (meters)
            heading: New heading (radians)
        """
        self.pose.x = x
        self.pose.y = y
        self.pose.heading = heading
        self.pose.timestamp = _monotonic()  # Use monotonic time
        self.total_distance = 0.0
        logger.info(f"Odometry reset to: x={x:.2f}m, y={y:.2f}m, heading={math.degrees(heading):.1f}°")
    
    def get_distance_traveled(self) -> float:
        """Get total distance traveled since last reset"""
        return self.total_distance
    
    def get_status(self) -> Dict[str, Any]:
        """Get odometry status"""
        return {
            "pose": {
                "x": self.pose.x,
                "y": self.pose.y,
                "heading_rad": self.pose.heading,
                "heading_deg": math.degrees(self.pose.heading),
                "timestamp": self.pose.timestamp
            },
            "velocity": {
                "linear_ms": self.linear_velocity,
                "angular_rads": self.angular_velocity
            },
            "distance_traveled": self.total_distance,
            "encoders_enabled": self.ticks_per_rev > 0,
            "wheel_base": self.wheel_base,
            "wheel_radius": self.wheel_radius
        }


__all__ = ["Odometry", "Pose"]

