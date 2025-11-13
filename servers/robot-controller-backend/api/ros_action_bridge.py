# api/ros_action_bridge.py
"""
ROS2 Action Bridge Service

Bridges ROS2 actions to WebSocket for autonomous behavior control from web app.
Supports sending action goals, receiving feedback, and getting results.
"""

import asyncio
import json
import logging
from typing import Dict, Optional, Callable
from fastapi import WebSocket

log = logging.getLogger(__name__)

# Try to import ROS2 action client
try:
    from rclpy.action import ActionClient
    from rclpy.node import Node
    ACTION_AVAILABLE = True
except ImportError:
    ACTION_AVAILABLE = False
    ActionClient = None
    Node = None


class ROS2ActionBridge:
    """
    Bridges ROS2 actions to WebSocket connections.
    Manages action clients and goal tracking.
    """
    
    def __init__(self, ros_node: Optional[Node] = None):
        self.ros_node = ros_node
        self.active_goals: Dict[str, any] = {}  # goal_id -> action_client
        self.action_clients: Dict[str, ActionClient] = {}
        
        if not ACTION_AVAILABLE:
            log.warning("ROS2 actions not available - action bridge disabled")
    
    async def send_goal(self, action_name: str, goal_data: dict, websocket: WebSocket) -> bool:
        """Send action goal and track progress."""
        if not ACTION_AVAILABLE or not self.ros_node:
            await websocket.send_json({
                "type": "action_error",
                "action": action_name,
                "error": "ROS2 actions not available"
            })
            return False
        
        try:
            # Get or create action client
            if action_name not in self.action_clients:
                # For now, use simple action structure
                # In production, would use proper action definitions
                log.warning(f"Action {action_name} not fully implemented - using simple goal")
            
            # Create goal ID
            import uuid
            goal_id = str(uuid.uuid4())
            
            # Store goal
            self.active_goals[goal_id] = {
                "action": action_name,
                "websocket": websocket,
                "start_time": asyncio.get_event_loop().time()
            }
            
            # Send goal via cmd_vel for simple actions
            # For proper action support, would use ActionClient here
            await websocket.send_json({
                "type": "action_goal_accepted",
                "action": action_name,
                "goal_id": goal_id
            })
            
            # Simulate action execution (in production, would use real action client)
            asyncio.create_task(self._simulate_action_execution(goal_id, action_name, goal_data, websocket))
            
            return True
        
        except Exception as e:
            log.error(f"Error sending action goal: {e}")
            await websocket.send_json({
                "type": "action_error",
                "action": action_name,
                "error": str(e)
            })
            return False
    
    async def _simulate_action_execution(self, goal_id: str, action_name: str, goal_data: dict, websocket: WebSocket):
        """Simulate action execution (placeholder - replace with real action client)."""
        try:
            # This is a placeholder - in production, would use real ROS2 ActionClient
            # For now, publish cmd_vel based on action type
            
            from .ros_native_bridge import get_bridge
            bridge = get_bridge()
            
            if action_name == "navigate_to_goal":
                # Navigate to goal - would use NavigateToGoal action
                x = goal_data.get("x", 0.0)
                y = goal_data.get("y", 0.0)
                # For now, just send a simple movement command
                if bridge:
                    # Calculate direction and distance
                    distance = (x**2 + y**2)**0.5
                    # Simplified: move forward
                    bridge.publish_twist("/cmd_vel", 0.3, 0.0)
                    
                    # Send feedback
                    await websocket.send_json({
                        "type": "action_feedback",
                        "action": action_name,
                        "goal_id": goal_id,
                        "feedback": {
                            "distance_remaining": distance,
                            "status": "moving"
                        }
                    })
            
            elif action_name == "follow_line":
                # Follow line - would use FollowLine action
                duration = goal_data.get("duration", 0.0)
                if bridge:
                    bridge.publish_twist("/cmd_vel", 0.3, 0.0)
                    
                    await websocket.send_json({
                        "type": "action_feedback",
                        "action": action_name,
                        "goal_id": goal_id,
                        "feedback": {
                            "line_detected": True,
                            "status": "following"
                        }
                    })
            
            elif action_name == "obstacle_avoidance":
                # Obstacle avoidance - would use ObstacleAvoidance action
                direction = goal_data.get("direction", "forward")
                if bridge:
                    if direction == "forward":
                        bridge.publish_twist("/cmd_vel", 0.3, 0.0)
                    elif direction == "left":
                        bridge.publish_twist("/cmd_vel", 0.2, 0.5)
                    elif direction == "right":
                        bridge.publish_twist("/cmd_vel", 0.2, -0.5)
                    
                    await websocket.send_json({
                        "type": "action_feedback",
                        "action": action_name,
                        "goal_id": goal_id,
                        "feedback": {
                            "obstacle_distance": 0.5,
                            "current_direction": direction
                        }
                    })
            
            # Simulate completion after delay
            await asyncio.sleep(2.0)
            
            # Send result
            await websocket.send_json({
                "type": "action_result",
                "action": action_name,
                "goal_id": goal_id,
                "result": {
                    "success": True,
                    "status": "completed"
                }
            })
            
            # Clean up
            if goal_id in self.active_goals:
                del self.active_goals[goal_id]
        
        except Exception as e:
            log.error(f"Error in action execution: {e}")
            await websocket.send_json({
                "type": "action_error",
                "action": action_name,
                "goal_id": goal_id,
                "error": str(e)
            })
            if goal_id in self.active_goals:
                del self.active_goals[goal_id]
    
    async def cancel_goal(self, goal_id: str, websocket: WebSocket) -> bool:
        """Cancel an active action goal."""
        if goal_id not in self.active_goals:
            await websocket.send_json({
                "type": "action_error",
                "goal_id": goal_id,
                "error": "Goal not found"
            })
            return False
        
        try:
            # Cancel goal (in production, would use ActionClient.cancel_goal)
            goal_info = self.active_goals[goal_id]
            action_name = goal_info["action"]
            
            # Stop robot
            from .ros_native_bridge import get_bridge
            bridge = get_bridge()
            if bridge:
                bridge.publish_twist("/cmd_vel", 0.0, 0.0)
            
            # Remove from active goals
            del self.active_goals[goal_id]
            
            await websocket.send_json({
                "type": "action_cancelled",
                "action": action_name,
                "goal_id": goal_id
            })
            
            return True
        
        except Exception as e:
            log.error(f"Error cancelling goal: {e}")
            return False


# Global action bridge instance
_ros2_action_bridge: Optional[ROS2ActionBridge] = None


def get_ros2_action_bridge(ros_node: Optional[Node] = None) -> Optional[ROS2ActionBridge]:
    """Get global ROS2 action bridge instance."""
    global _ros2_action_bridge
    if _ros2_action_bridge is None:
        _ros2_action_bridge = ROS2ActionBridge(ros_node)
    return _ros2_action_bridge

