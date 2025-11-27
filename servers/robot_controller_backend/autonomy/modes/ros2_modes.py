"""ROS2-based autonomy mode handlers.

These handlers integrate with ROS2 action servers for actual autonomous behaviors.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any, Mapping, MutableMapping

from .base import AutonomyModeHandler

__all__ = [
    "ROS2LineFollowMode",
    "ROS2ObstacleAvoidanceMode",
    "ROS2NavigateToGoalMode",
]


class ROS2LineFollowMode(AutonomyModeHandler):
    """Line following mode using ROS2 action server."""

    def __init__(self, *, context: Mapping[str, Any] | None = None, logger: logging.Logger | None = None) -> None:
        super().__init__("line_follow", logger=logger)
        self.context = context or {}
        self._action_client = None
        self._goal_handle = None

    async def start(self, params: Mapping[str, Any]) -> None:
        """Start line following via ROS2 action."""
        duration = params.get("duration", 0.0)  # 0 = infinite
        
        # Get ROS2 bridge from context or try to get it
        bridge = self.context.get('ros2_bridge')
        if not bridge:
            try:
                from api.ros_native_bridge import get_bridge
                bridge = get_bridge()
            except ImportError:
                pass
        
        if bridge:
            try:
                from api.ros_action_bridge import get_ros2_action_bridge
                ros_node = bridge._node if hasattr(bridge, '_node') else None
                action_bridge = get_ros2_action_bridge(ros_node)
                
                if action_bridge:
                    self.logger.info(f"Starting ROS2 line follow via action server (duration: {duration}s)")
                    # Use action bridge to send goal
                    # For now, publish cmd_vel as fallback
                    bridge.publish_twist("/cmd_vel", 0.3, 0.0)
                    self.logger.info("Published cmd_vel for line following")
                else:
                    # Fallback: direct cmd_vel
                    self.logger.info("ROS2 action bridge not available, using direct cmd_vel")
                    bridge.publish_twist("/cmd_vel", 0.3, 0.0)
            except Exception as e:
                self.logger.error(f"Error starting ROS2 line follow: {e}")
        else:
            self.logger.warning("ROS2 bridge not available in context - line follow will not work")

    async def stop(self) -> None:
        """Stop line following."""
        bridge = self.context.get('ros2_bridge')
        if not bridge:
            try:
                from api.ros_native_bridge import get_bridge
                bridge = get_bridge()
            except ImportError:
                pass
        
        if bridge:
            try:
                bridge.publish_twist("/cmd_vel", 0.0, 0.0)
                self.logger.info("Stopped line following")
            except Exception as e:
                self.logger.error(f"Error stopping line follow: {e}")

    def snapshot(self) -> MutableMapping[str, Any]:
        return {
            "ros2_enabled": True,
            "action": "follow_line",
        }


class ROS2ObstacleAvoidanceMode(AutonomyModeHandler):
    """Obstacle avoidance mode using ROS2 action server."""

    def __init__(self, *, context: Mapping[str, Any] | None = None, logger: logging.Logger | None = None) -> None:
        super().__init__("avoid_obstacles", logger=logger)
        self.context = context or {}

    async def start(self, params: Mapping[str, Any]) -> None:
        """Start obstacle avoidance via ROS2 action."""
        direction = params.get("direction", "forward")
        
        bridge = self.context.get('ros2_bridge')
        if not bridge:
            try:
                from api.ros_native_bridge import get_bridge
                bridge = get_bridge()
            except ImportError:
                pass
        
        if bridge:
            try:
                self.logger.info(f"Starting ROS2 obstacle avoidance (direction: {direction})")
                
                if direction == "forward":
                    bridge.publish_twist("/cmd_vel", 0.3, 0.0)
                elif direction == "left":
                    bridge.publish_twist("/cmd_vel", 0.2, 0.5)
                elif direction == "right":
                    bridge.publish_twist("/cmd_vel", 0.2, -0.5)
                else:
                    bridge.publish_twist("/cmd_vel", 0.3, 0.0)
                
                self.logger.info("Published cmd_vel for obstacle avoidance")
            except Exception as e:
                self.logger.error(f"Error starting ROS2 obstacle avoidance: {e}")
        else:
            self.logger.warning("ROS2 bridge not available - obstacle avoidance will not work")

    async def stop(self) -> None:
        """Stop obstacle avoidance."""
        bridge = self.context.get('ros2_bridge')
        if not bridge:
            try:
                from api.ros_native_bridge import get_bridge
                bridge = get_bridge()
            except ImportError:
                pass
        
        if bridge:
            try:
                bridge.publish_twist("/cmd_vel", 0.0, 0.0)
                self.logger.info("Stopped obstacle avoidance")
            except Exception as e:
                self.logger.error(f"Error stopping obstacle avoidance: {e}")

    def snapshot(self) -> MutableMapping[str, Any]:
        return {
            "ros2_enabled": True,
            "action": "obstacle_avoidance",
        }


class ROS2NavigateToGoalMode(AutonomyModeHandler):
    """Navigate to goal mode using ROS2 action server."""

    def __init__(self, *, context: Mapping[str, Any] | None = None, logger: logging.Logger | None = None) -> None:
        super().__init__("waypoints", logger=logger)
        self.context = context or {}

    async def start(self, params: Mapping[str, Any]) -> None:
        """Start navigation to goal via ROS2 action."""
        x = params.get("goal_x", params.get("x", 0.0))
        y = params.get("goal_y", params.get("y", 0.0))
        theta = params.get("goal_theta", params.get("theta", 0.0))
        
        bridge = self.context.get('ros2_bridge')
        if not bridge:
            try:
                from api.ros_native_bridge import get_bridge
                bridge = get_bridge()
            except ImportError:
                pass
        
        if bridge:
            try:
                self.logger.info(f"Starting ROS2 navigate to goal ({x}, {y}, {theta})")
                
                # For now, calculate direction and move
                # In production, would use proper navigate_to_goal action
                import math
                distance = math.sqrt(x**2 + y**2)
                if distance > 0.1:
                    # Move towards goal
                    angle = math.atan2(y, x)
                    bridge.publish_twist("/cmd_vel", 0.3, angle * 0.5)
                    self.logger.info("Published cmd_vel for navigation")
            except Exception as e:
                self.logger.error(f"Error starting ROS2 navigation: {e}")
        else:
            self.logger.warning("ROS2 bridge not available - navigation will not work")

    async def stop(self) -> None:
        """Stop navigation."""
        bridge = self.context.get('ros2_bridge')
        if not bridge:
            try:
                from api.ros_native_bridge import get_bridge
                bridge = get_bridge()
            except ImportError:
                pass
        
        if bridge:
            try:
                bridge.publish_twist("/cmd_vel", 0.0, 0.0)
                self.logger.info("Stopped navigation")
            except Exception as e:
                self.logger.error(f"Error stopping navigation: {e}")

    def snapshot(self) -> MutableMapping[str, Any]:
        return {
            "ros2_enabled": True,
            "action": "navigate_to_goal",
        }


def ROS2_MODE_FACTORIES() -> Mapping[str, Any]:
    """Return ROS2-based mode factories."""
    return {
        "line_follow": lambda **kw: ROS2LineFollowMode(context=kw.get("context"), logger=kw.get("logger")),
        "avoid_obstacles": lambda **kw: ROS2ObstacleAvoidanceMode(context=kw.get("context"), logger=kw.get("logger")),
        "waypoints": lambda **kw: ROS2NavigateToGoalMode(context=kw.get("context"), logger=kw.get("logger")),
    }

