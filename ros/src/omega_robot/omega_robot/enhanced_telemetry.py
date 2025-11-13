#!/usr/bin/env python3
"""
Enhanced Telemetry Publisher Node

Publishes comprehensive robot telemetry including:
- Sensor readings (ultrasonic, line tracking, battery)
- Motor states
- System health
- Timestamp
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

# Try to import psutil (optional)
try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False


class EnhancedTelemetryPublisher(Node):
    """Publishes enhanced telemetry with actual sensor and system data."""

    def __init__(self):
        super().__init__('enhanced_telemetry_publisher')
        
        self.telemetry_pub = self.create_publisher(
            String, '/omega/telemetry', 10
        )
        
        # Subscribe to sensor topics
        from std_msgs.msg import Float32, Int32MultiArray
        from sensor_msgs.msg import BatteryState
        
        self.ultrasonic_data = None
        self.line_tracking_data = None
        self.battery_data = None
        
        self.ultrasonic_sub = self.create_subscription(
            Float32,
            '/omega/sensors/ultrasonic',
            self.ultrasonic_callback,
            10
        )
        
        self.line_tracking_sub = self.create_subscription(
            Int32MultiArray,
            '/omega/sensors/line_tracking',
            self.line_tracking_callback,
            10
        )
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/omega/sensors/battery',
            self.battery_callback,
            10
        )
        
        # Timer for publishing
        self.timer = self.create_timer(0.5, self.publish_telemetry)  # 2 Hz
        
        self.get_logger().info('Enhanced telemetry publisher started')

    def ultrasonic_callback(self, msg):
        """Store ultrasonic reading."""
        self.ultrasonic_data = msg.data

    def line_tracking_callback(self, msg):
        """Store line tracking data."""
        self.line_tracking_data = list(msg.data)

    def battery_callback(self, msg):
        """Store battery data."""
        self.battery_data = {
            'voltage': msg.voltage,
            'percentage': msg.percentage
        }

    def publish_telemetry(self):
        """Publish comprehensive telemetry."""
        # Get system info
        system_info = {}
        if PSUTIL_AVAILABLE:
            try:
                system_info = {
                    'cpu_percent': psutil.cpu_percent(interval=None),
                    'memory_percent': psutil.virtual_memory().percent,
                    'memory_available_mb': psutil.virtual_memory().available / (1024 * 1024)
                }
            except:
                system_info = {'error': 'Could not read system info'}
        else:
            system_info = {'note': 'psutil not available'}
        
        telemetry = {
            'timestamp': time.time(),
            'sensors': {
                'ultrasonic_cm': self.ultrasonic_data if self.ultrasonic_data else None,
                'line_tracking': self.line_tracking_data if self.line_tracking_data else None,
                'battery': self.battery_data if self.battery_data else None
            },
            'system': system_info,
            'status': 'active'
        }
        
        msg = String()
        msg.data = json.dumps(telemetry)
        self.telemetry_pub.publish(msg)
        
        self.get_logger().debug(f'Published telemetry: {telemetry}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = EnhancedTelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

