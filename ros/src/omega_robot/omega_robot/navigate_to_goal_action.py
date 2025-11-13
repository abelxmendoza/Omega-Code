#!/usr/bin/env python3
"""
Navigate to Goal Action Server

ROS2 action server for autonomous navigation to a goal position.
Uses cmd_vel to move robot to target coordinates.

Action: NavigateToGoal
  Goal: {x: float, y: float, theta: float}
  Feedback: {distance_remaining: float, current_pose: {...}}
  Result: {success: bool, final_pose: {...}, distance_traveled: float}
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
import math
import time

# Try to import action definition (create if doesn't exist)
try:
    from omega_robot_interfaces.action import NavigateToGoal
except ImportError:
    # Fallback: use simple goal structure
    NavigateToGoal = None


class NavigateToGoalActionServer(Node):
    """Action server for navigating to a goal position."""

    def __init__(self):
        super().__init__('navigate_to_goal_action_server')
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers for feedback (if available)
        self.ultrasonic_sub = None
        self.current_distance = None
        
        try:
            from std_msgs.msg import Float32
            self.ultrasonic_sub = self.create_subscription(
                Float32,
                '/omega/sensors/ultrasonic',
                self.ultrasonic_callback,
                10
            )
        except:
            pass
        
        # Action server
        if NavigateToGoal:
            self._action_server = ActionServer(
                self,
                NavigateToGoal,
                'navigate_to_goal',
                self.execute_callback
            )
        else:
            # Simple implementation without custom action type
            self.get_logger().warn('NavigateToGoal action type not found - using simple implementation')
            self._action_server = None
        
        # State
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.is_navigating = False
        
        self.get_logger().info('Navigate to Goal action server started')

    def ultrasonic_callback(self, msg):
        """Update current distance reading."""
        self.current_distance = msg.data

    def execute_callback(self, goal_handle):
        """Execute navigation action."""
        self.get_logger().info(f'Executing navigate to goal: {goal_handle.request}')
        
        # Extract goal
        if NavigateToGoal:
            goal_x = goal_handle.request.x
            goal_y = goal_handle.request.y
            goal_theta = goal_handle.request.theta
        else:
            # Simple fallback - use goal handle attributes
            goal_x = getattr(goal_handle.request, 'x', 0.0)
            goal_y = getattr(goal_handle.request, 'y', 0.0)
            goal_theta = getattr(goal_handle.request, 'theta', 0.0)
        
        self.is_navigating = True
        start_pose = self.current_pose.copy()
        distance_traveled = 0.0
        
        # Navigation parameters
        linear_speed = 0.3  # m/s
        angular_speed = 0.5  # rad/s
        position_tolerance = 0.1  # m
        angle_tolerance = 0.1  # rad
        
        feedback_msg = None
        if NavigateToGoal:
            feedback_msg = NavigateToGoal.Feedback()
        
        try:
            while rclpy.ok():
                # Check if goal was cancelled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Navigation cancelled')
                    self._stop_robot()
                    return NavigateToGoal.Result() if NavigateToGoal else None
                
                # Calculate distance and angle to goal
                dx = goal_x - self.current_pose['x']
                dy = goal_y - self.current_pose['y']
                distance = math.sqrt(dx*dx + dy*dy)
                target_angle = math.atan2(dy, dx)
                angle_error = self._normalize_angle(target_angle - self.current_pose['theta'])
                
                # Check if goal reached
                if distance < position_tolerance and abs(angle_error) < angle_tolerance:
                    goal_handle.succeed()
                    result = NavigateToGoal.Result() if NavigateToGoal else type('Result', (), {})()
                    if hasattr(result, 'success'):
                        result.success = True
                    if hasattr(result, 'final_pose'):
                        result.final_pose = self.current_pose
                    if hasattr(result, 'distance_traveled'):
                        result.distance_traveled = distance_traveled
                    self.get_logger().info(f'Goal reached! Distance traveled: {distance_traveled:.2f}m')
                    self._stop_robot()
                    return result
                
                # Check for obstacles (if ultrasonic available)
                if self.current_distance is not None and self.current_distance < 0.2:
                    self.get_logger().warn('Obstacle detected! Stopping.')
                    goal_handle.abort()
                    result = NavigateToGoal.Result() if NavigateToGoal else type('Result', (), {})()
                    if hasattr(result, 'success'):
                        result.success = False
                    self._stop_robot()
                    return result
                
                # Publish velocity command
                cmd = Twist()
                
                # First, align with goal direction
                if abs(angle_error) > angle_tolerance:
                    cmd.angular.z = angular_speed if angle_error > 0 else -angular_speed
                else:
                    # Move forward
                    cmd.linear.x = min(linear_speed, distance * 0.5)
                    cmd.angular.z = angle_error * 0.5  # Fine adjustment
                
                self.cmd_vel_pub.publish(cmd)
                
                # Update pose (simplified - in real implementation, use odometry)
                dt = 0.1  # Assume 10 Hz update
                self.current_pose['x'] += cmd.linear.x * math.cos(self.current_pose['theta']) * dt
                self.current_pose['y'] += cmd.linear.x * math.sin(self.current_pose['theta']) * dt
                self.current_pose['theta'] += cmd.angular.z * dt
                self.current_pose['theta'] = self._normalize_angle(self.current_pose['theta'])
                
                distance_traveled += abs(cmd.linear.x) * dt
                
                # Publish feedback
                if feedback_msg and NavigateToGoal:
                    feedback_msg.distance_remaining = distance
                    feedback_msg.current_pose = self.current_pose
                    goal_handle.publish_feedback(feedback_msg)
                
                time.sleep(0.1)  # 10 Hz control loop
        
        except Exception as e:
            self.get_logger().error(f'Error in navigation: {e}')
            goal_handle.abort()
            result = NavigateToGoal.Result() if NavigateToGoal else type('Result', (), {})()
            if hasattr(result, 'success'):
                result.success = False
            self._stop_robot()
            return result
        finally:
            self.is_navigating = False

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _stop_robot(self):
        """Stop robot movement."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().debug('Robot stopped')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = NavigateToGoalActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

