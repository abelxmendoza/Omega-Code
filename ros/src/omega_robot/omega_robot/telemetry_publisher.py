#!/usr/bin/env python3
"""
Telemetry Publisher Node for Omega Robot

Publishes heartbeat telemetry messages on /omega/telemetry topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class TelemetryPublisher(Node):
    """Publishes telemetry heartbeat messages."""

    def __init__(self):
        super().__init__('telemetry_publisher')
        self.publisher_ = self.create_publisher(String, '/omega/telemetry', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Telemetry publisher started')

    def timer_callback(self):
        """Publish telemetry message."""
        msg = String()
        msg.data = f'heartbeat:{self.counter}:{time.time()}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.counter += 1


def main(args=None):
    """Main entry point for telemetry publisher node."""
    rclpy.init(args=args)
    node = TelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

