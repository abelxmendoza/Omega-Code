#!/usr/bin/env python3
"""
Follow Line Action Server

ROS2 action server for autonomous line following.
Uses line tracking sensors to follow a line on the ground.

Action: FollowLine
  Goal: {duration: float}  # seconds, 0 = infinite
  Feedback: {line_detected: bool, sensor_readings: [int, int, int]}
  Result: {success: bool, distance_traveled: float, duration: float}
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import time

# Try to import action definition
try:
    from omega_robot_interfaces.action import FollowLine
except ImportError:
    FollowLine = None


class FollowLineActionServer(Node):
    """Action server for following a line."""

    def __init__(self):
        super().__init__('follow_line_action_server')
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for line tracking sensors
        self.line_tracking_sub = self.create_subscription(
            Int32MultiArray,
            '/omega/sensors/line_tracking',
            self.line_tracking_callback,
            10
        )
        
        # Current sensor readings [left, center, right]
        self.sensor_readings = [0, 0, 0]
        self.last_update = time.time()
        
        # Action server
        if FollowLine:
            self._action_server = ActionServer(
                self,
                FollowLine,
                'follow_line',
                self.execute_callback
            )
        else:
            self.get_logger().warn('FollowLine action type not found - using simple implementation')
            self._action_server = None
        
        # Control parameters
        self.base_speed = 0.3  # m/s
        self.turn_speed = 0.4  # rad/s
        self.line_lost_timeout = 2.0  # seconds
        
        self.get_logger().info('Follow Line action server started')

    def line_tracking_callback(self, msg):
        """Update line tracking sensor readings."""
        if len(msg.data) >= 3:
            self.sensor_readings = list(msg.data[:3])
            self.last_update = time.time()

    def execute_callback(self, goal_handle):
        """Execute line following action."""
        self.get_logger().info('Executing follow line action')
        
        # Extract goal duration
        if FollowLine:
            duration = goal_handle.request.duration
        else:
            duration = getattr(goal_handle.request, 'duration', 0.0)
        
        infinite = (duration <= 0.0)
        start_time = time.time()
        distance_traveled = 0.0
        line_lost_time = None
        
        feedback_msg = None
        if FollowLine:
            feedback_msg = FollowLine.Feedback()
        
        try:
            while rclpy.ok():
                # Check if goal was cancelled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Follow line cancelled')
                    self._stop_robot()
                    return FollowLine.Result() if FollowLine else None
                
                # Check duration
                elapsed = time.time() - start_time
                if not infinite and elapsed >= duration:
                    goal_handle.succeed()
                    result = FollowLine.Result() if FollowLine else type('Result', (), {})()
                    if hasattr(result, 'success'):
                        result.success = True
                    if hasattr(result, 'distance_traveled'):
                        result.distance_traveled = distance_traveled
                    if hasattr(result, 'duration'):
                        result.duration = elapsed
                    self.get_logger().info(f'Follow line completed. Duration: {elapsed:.2f}s')
                    self._stop_robot()
                    return result
                
                # Check if sensors are stale
                if time.time() - self.last_update > 1.0:
                    self.get_logger().warn('Line tracking sensors not updating')
                
                # Determine line position
                left, center, right = self.sensor_readings
                line_detected = (left == 1 or center == 1 or right == 1)
                
                # Check if line lost
                if not line_detected:
                    if line_lost_time is None:
                        line_lost_time = time.time()
                    elif time.time() - line_lost_time > self.line_lost_timeout:
                        self.get_logger().warn('Line lost - stopping')
                        goal_handle.abort()
                        result = FollowLine.Result() if FollowLine else type('Result', (), {})()
                        if hasattr(result, 'success'):
                            result.success = False
                        self._stop_robot()
                        return result
                else:
                    line_lost_time = None
                
                # Calculate control command
                cmd = Twist()
                
                if center == 1:
                    # Line centered - go straight
                    cmd.linear.x = self.base_speed
                    cmd.angular.z = 0.0
                elif left == 1:
                    # Line on left - turn left
                    cmd.linear.x = self.base_speed * 0.7
                    cmd.angular.z = self.turn_speed
                elif right == 1:
                    # Line on right - turn right
                    cmd.linear.x = self.base_speed * 0.7
                    cmd.angular.z = -self.turn_speed
                else:
                    # No line detected - slow down and search
                    cmd.linear.x = self.base_speed * 0.3
                    cmd.angular.z = self.turn_speed * 0.5
                
                self.cmd_vel_pub.publish(cmd)
                
                # Update distance (simplified)
                dt = 0.1
                distance_traveled += abs(cmd.linear.x) * dt
                
                # Publish feedback
                if feedback_msg and FollowLine:
                    feedback_msg.line_detected = line_detected
                    feedback_msg.sensor_readings = self.sensor_readings
                    goal_handle.publish_feedback(feedback_msg)
                
                time.sleep(0.1)  # 10 Hz control loop
        
        except Exception as e:
            self.get_logger().error(f'Error in follow line: {e}')
            goal_handle.abort()
            result = FollowLine.Result() if FollowLine else type('Result', (), {})()
            if hasattr(result, 'success'):
                result.success = False
            self._stop_robot()
            return result

    def _stop_robot(self):
        """Stop robot movement."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().debug('Robot stopped')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = FollowLineActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

