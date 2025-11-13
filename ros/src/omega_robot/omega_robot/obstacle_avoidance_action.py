#!/usr/bin/env python3
"""
Obstacle Avoidance Action Server

ROS2 action server for autonomous obstacle avoidance.
Uses ultrasonic sensor to detect and avoid obstacles while moving.

Action: ObstacleAvoidance
  Goal: {direction: string, distance: float}  # "forward", "backward", "left", "right"
  Feedback: {obstacle_distance: float, current_direction: string}
  Result: {success: bool, obstacles_avoided: int, distance_traveled: float}
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

# Try to import action definition
try:
    from omega_robot_interfaces.action import ObstacleAvoidance
except ImportError:
    ObstacleAvoidance = None


class ObstacleAvoidanceActionServer(Node):
    """Action server for obstacle avoidance."""

    def __init__(self):
        super().__init__('obstacle_avoidance_action_server')
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for ultrasonic sensor
        self.ultrasonic_sub = self.create_subscription(
            Float32,
            '/omega/sensors/ultrasonic',
            self.ultrasonic_callback,
            10
        )
        
        # Current distance reading
        self.obstacle_distance = None
        self.last_update = time.time()
        
        # Action server
        if ObstacleAvoidance:
            self._action_server = ActionServer(
                self,
                ObstacleAvoidance,
                'obstacle_avoidance',
                self.execute_callback
            )
        else:
            self.get_logger().warn('ObstacleAvoidance action type not found - using simple implementation')
            self._action_server = None
        
        # Control parameters
        self.safe_distance = 0.3  # meters
        self.obstacle_distance_threshold = 0.2  # meters
        self.base_speed = 0.3  # m/s
        self.turn_speed = 0.5  # rad/s
        
        self.get_logger().info('Obstacle Avoidance action server started')

    def ultrasonic_callback(self, msg):
        """Update obstacle distance reading."""
        self.obstacle_distance = msg.data / 100.0  # Convert cm to meters
        self.last_update = time.time()

    def execute_callback(self, goal_handle):
        """Execute obstacle avoidance action."""
        self.get_logger().info('Executing obstacle avoidance action')
        
        # Extract goal
        if ObstacleAvoidance:
            direction = goal_handle.request.direction
            target_distance = goal_handle.request.distance
        else:
            direction = getattr(goal_handle.request, 'direction', 'forward')
            target_distance = getattr(goal_handle.request, 'distance', 0.0)
        
        obstacles_avoided = 0
        distance_traveled = 0.0
        start_time = time.time()
        
        feedback_msg = None
        if ObstacleAvoidance:
            feedback_msg = ObstacleAvoidance.Feedback()
        
        try:
            while rclpy.ok():
                # Check if goal was cancelled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Obstacle avoidance cancelled')
                    self._stop_robot()
                    return ObstacleAvoidance.Result() if ObstacleAvoidance else None
                
                # Check if sensors are stale
                if time.time() - self.last_update > 1.0:
                    self.get_logger().warn('Ultrasonic sensor not updating')
                
                # Check for obstacles
                if self.obstacle_distance is not None:
                    if self.obstacle_distance < self.obstacle_distance_threshold:
                        # Obstacle detected - avoid it
                        obstacles_avoided += 1
                        self.get_logger().info(f'Obstacle detected at {self.obstacle_distance:.2f}m - avoiding')
                        
                        # Turn away from obstacle
                        cmd = Twist()
                        cmd.angular.z = self.turn_speed
                        cmd.linear.x = 0.0
                        self.cmd_vel_pub.publish(cmd)
                        
                        # Wait for turn
                        time.sleep(0.5)
                        
                        # Continue
                        continue
                
                # Normal movement
                cmd = Twist()
                
                if direction == 'forward':
                    if self.obstacle_distance is None or self.obstacle_distance > self.safe_distance:
                        cmd.linear.x = self.base_speed
                        cmd.angular.z = 0.0
                    else:
                        # Too close - turn
                        cmd.linear.x = 0.0
                        cmd.angular.z = self.turn_speed
                elif direction == 'backward':
                    cmd.linear.x = -self.base_speed * 0.5
                    cmd.angular.z = 0.0
                elif direction == 'left':
                    cmd.linear.x = self.base_speed * 0.5
                    cmd.angular.z = self.turn_speed
                elif direction == 'right':
                    cmd.linear.x = self.base_speed * 0.5
                    cmd.angular.z = -self.turn_speed
                else:
                    # Default: forward
                    cmd.linear.x = self.base_speed
                    cmd.angular.z = 0.0
                
                self.cmd_vel_pub.publish(cmd)
                
                # Update distance
                dt = 0.1
                distance_traveled += abs(cmd.linear.x) * dt
                
                # Check if target distance reached
                if target_distance > 0 and distance_traveled >= target_distance:
                    goal_handle.succeed()
                    result = ObstacleAvoidance.Result() if ObstacleAvoidance else type('Result', (), {})()
                    if hasattr(result, 'success'):
                        result.success = True
                    if hasattr(result, 'obstacles_avoided'):
                        result.obstacles_avoided = obstacles_avoided
                    if hasattr(result, 'distance_traveled'):
                        result.distance_traveled = distance_traveled
                    self.get_logger().info(f'Target distance reached. Obstacles avoided: {obstacles_avoided}')
                    self._stop_robot()
                    return result
                
                # Publish feedback
                if feedback_msg and ObstacleAvoidance:
                    feedback_msg.obstacle_distance = self.obstacle_distance if self.obstacle_distance else 0.0
                    feedback_msg.current_direction = direction
                    goal_handle.publish_feedback(feedback_msg)
                
                time.sleep(0.1)  # 10 Hz control loop
        
        except Exception as e:
            self.get_logger().error(f'Error in obstacle avoidance: {e}')
            goal_handle.abort()
            result = ObstacleAvoidance.Result() if ObstacleAvoidance else type('Result', (), {})()
            if hasattr(result, 'success'):
                result.success = False
            self._stop_robot()
            return result
        finally:
            self._stop_robot()

    def _stop_robot(self):
        """Stop robot movement."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().debug('Robot stopped')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ObstacleAvoidanceActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

