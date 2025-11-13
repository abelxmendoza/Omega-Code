#!/usr/bin/env python3
"""
Odometry Publisher Node

Publishes robot odometry based on motor encoder readings or wheel odometry.
Provides position and velocity estimates for navigation.

Topics:
- /odom (nav_msgs/Odometry) - Robot odometry
- /tf (tf2_msgs/TFMessage) - Transform tree
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_msgs.msg import Float32
import math
import time

# Try to import tf2
try:
    from tf2_ros import TransformBroadcaster
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False
    TransformBroadcaster = None


class OdometryPublisher(Node):
    """Publishes robot odometry from motor encoders or wheel odometry."""

    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Transform broadcaster
        if TF2_AVAILABLE:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None
            self.get_logger().warn('tf2_ros not available - transforms disabled')
        
        # Subscribers for motor speeds
        self.left_motor_sub = self.create_subscription(
            Float32,
            '/omega/motors/left',
            self.left_motor_callback,
            10
        )
        
        self.right_motor_sub = self.create_subscription(
            Float32,
            '/omega/motors/right',
            self.right_motor_callback,
            10
        )
        
        # Robot parameters
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.05).value  # meters
        self.wheel_base = self.declare_parameter('wheel_base', 0.20).value  # meters (distance between wheels)
        self.ticks_per_revolution = self.declare_parameter('ticks_per_revolution', 360).value
        
        # State
        self.left_speed = 0.0  # m/s
        self.right_speed = 0.0  # m/s
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.theta = 0.0  # radians
        self.vx = 0.0  # m/s
        self.vtheta = 0.0  # rad/s
        
        self.last_time = self.get_clock().now()
        
        # Publishing timer
        self.timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz
        
        self.get_logger().info('Odometry publisher started')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, Base: {self.wheel_base}m')

    def left_motor_callback(self, msg):
        """Update left motor speed."""
        # Convert from [-1.0, 1.0] to m/s
        # Assuming max speed of 0.5 m/s
        max_speed = 0.5
        self.left_speed = msg.data * max_speed

    def right_motor_callback(self, msg):
        """Update right motor speed."""
        # Convert from [-1.0, 1.0] to m/s
        max_speed = 0.5
        self.right_speed = msg.data * max_speed

    def publish_odometry(self):
        """Calculate and publish odometry."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Calculate linear and angular velocities from wheel speeds
        # Differential drive kinematics
        v_left = self.left_speed
        v_right = self.right_speed
        
        # Linear velocity (average of both wheels)
        self.vx = (v_left + v_right) / 2.0
        
        # Angular velocity (difference between wheels)
        self.vtheta = (v_right - v_left) / self.wheel_base
        
        # Update pose (simple integration)
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        self.theta += self.vtheta * dt
        
        # Normalize angle
        self.theta = self._normalize_angle(self.theta)
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        q = self._yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation = q
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vtheta
        
        # Covariance (simplified - would be calculated from sensor uncertainty)
        odom.pose.covariance[0] = 0.1  # x
        odom.pose.covariance[7] = 0.1  # y
        odom.pose.covariance[35] = 0.1  # yaw
        
        odom.twist.covariance[0] = 0.1  # vx
        odom.twist.covariance[35] = 0.1  # vtheta
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Publish transform
        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)
        
        self.last_time = current_time

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

