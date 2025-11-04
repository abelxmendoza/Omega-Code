#!/usr/bin/env python3
"""
Telemetry Listener Node for Omega Robot

Subscribes to /omega/telemetry topic and logs received messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TelemetryListener(Node):
    """Listens to telemetry messages."""

    def __init__(self):
        super().__init__('telemetry_listener')
        self.subscription = self.create_subscription(
            String,
            '/omega/telemetry',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Telemetry listener started, waiting for messages...')

    def listener_callback(self, msg):
        """Process received telemetry message."""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    """Main entry point for telemetry listener node."""
    rclpy.init(args=args)
    node = TelemetryListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

