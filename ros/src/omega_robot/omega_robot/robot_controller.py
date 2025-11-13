#!/usr/bin/env python3
"""
Robot Controller Node for Omega Robot

Subscribes to cmd_vel (geometry_msgs/Twist) and converts to motor commands.
Publishes current motor state and odometry.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
import time

# Try to import hardware libraries
try:
    import lgpio
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class RobotController(Node):
    """Controls robot motors based on cmd_vel commands."""

    def __init__(self):
        super().__init__('robot_controller')
        
        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.motor_left_pub = self.create_publisher(
            Float32, '/omega/motors/left', 10
        )
        self.motor_right_pub = self.create_publisher(
            Float32, '/omega/motors/right', 10
        )
        self.status_pub = self.create_publisher(
            String, '/omega/controller/status', 10
        )
        
        # Motor state
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        # Hardware initialization
        self.hardware_available = HARDWARE_AVAILABLE
        self.gpio_handle = None
        
        if self.hardware_available:
            try:
                self.gpio_handle = lgpio.gpiochip_open(0)
                # Initialize motor pins (adjust to your setup)
                # self.motor_left_pin = 20
                # self.motor_right_pin = 21
                self.get_logger().info('Motor hardware initialized')
            except Exception as e:
                self.get_logger().warn(f'Could not initialize motor hardware: {e}')
                self.hardware_available = False
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Robot controller started')
        self.get_logger().info(f'Hardware available: {self.hardware_available}')

    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to motor speeds."""
        linear_x = msg.linear.x  # Forward/backward speed (-1.0 to 1.0)
        angular_z = msg.angular.z  # Rotation speed (-1.0 to 1.0)
        
        # Differential drive conversion
        # Base speed from linear velocity
        base_speed = linear_x
        
        # Differential from angular velocity
        # Positive angular_z = turn right (left motor faster)
        # Negative angular_z = turn left (right motor faster)
        differential = angular_z * 0.5  # Adjust turning sensitivity
        
        self.left_speed = base_speed + differential
        self.right_speed = base_speed - differential
        
        # Clamp to [-1.0, 1.0]
        self.left_speed = max(-1.0, min(1.0, self.left_speed))
        self.right_speed = max(-1.0, min(1.0, self.right_speed))
        
        # Publish motor commands
        left_msg = Float32()
        left_msg.data = self.left_speed
        self.motor_left_pub.publish(left_msg)
        
        right_msg = Float32()
        right_msg.data = self.right_speed
        self.motor_right_pub.publish(right_msg)
        
        # Apply to hardware if available
        if self.hardware_available:
            self.apply_motor_commands()
        
        self.get_logger().debug(
            f'cmd_vel: linear={linear_x:.2f}, angular={angular_z:.2f} -> '
            f'left={self.left_speed:.2f}, right={self.right_speed:.2f}'
        )

    def apply_motor_commands(self):
        """Apply motor speeds to hardware."""
        try:
            # Implement actual motor control here
            # This is a placeholder - integrate with your motor driver
            # Example: PWM signals, motor driver commands, etc.
            pass
        except Exception as e:
            self.get_logger().error(f'Error applying motor commands: {e}')

    def publish_status(self):
        """Publish controller status."""
        status = {
            'left_speed': self.left_speed,
            'right_speed': self.right_speed,
            'hardware_available': self.hardware_available,
            'timestamp': time.time()
        }
        
        status_msg = String()
        status_msg.data = str(status)
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Stop motors on shutdown."""
        # Stop motors
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        if self.hardware_available:
            try:
                self.apply_motor_commands()
            except:
                pass
        
        if self.gpio_handle is not None:
            try:
                lgpio.gpiochip_close(self.gpio_handle)
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

