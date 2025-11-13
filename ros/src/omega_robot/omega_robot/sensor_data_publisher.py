#!/usr/bin/env python3
"""
Sensor Data Publisher Node for Omega Robot

Publishes comprehensive sensor data to ROS2 topics:
- Ultrasonic distance
- Line tracking sensors
- Battery voltage
- System status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray, Bool
from sensor_msgs.msg import BatteryState
import time
import json

# Try to import hardware libraries (optional - works without hardware)
try:
    import lgpio
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class SensorDataPublisher(Node):
    """Publishes sensor data from Omega robot hardware."""

    def __init__(self):
        super().__init__('sensor_data_publisher')
        
        # Publishers
        self.ultrasonic_pub = self.create_publisher(
            Float32, '/omega/sensors/ultrasonic', 10
        )
        self.line_tracking_pub = self.create_publisher(
            Int32MultiArray, '/omega/sensors/line_tracking', 10
        )
        self.battery_pub = self.create_publisher(
            BatteryState, '/omega/sensors/battery', 10
        )
        self.system_status_pub = self.create_publisher(
            Bool, '/omega/system/ready', 10
        )
        
        # Timer for sensor reading
        self.timer = self.create_timer(0.1, self.publish_sensors)  # 10 Hz
        
        # Hardware initialization
        self.hardware_available = HARDWARE_AVAILABLE
        self.gpio_handle = None
        
        if self.hardware_available:
            try:
                self.gpio_handle = lgpio.gpiochip_open(0)
                self.get_logger().info('Hardware GPIO initialized')
            except Exception as e:
                self.get_logger().warn(f'Could not initialize GPIO: {e}')
                self.hardware_available = False
        
        self.get_logger().info('Sensor data publisher started')
        self.get_logger().info(f'Hardware available: {self.hardware_available}')

    def read_ultrasonic(self) -> float:
        """Read ultrasonic sensor distance in cm."""
        if not self.hardware_available:
            # Simulated data for testing
            return 50.0
        
        try:
            # GPIO pins for ultrasonic (example - adjust to your setup)
            TRIGGER_PIN = 23
            ECHO_PIN = 24
            
            # Trigger pulse
            lgpio.gpio_write(self.gpio_handle, TRIGGER_PIN, 1)
            time.sleep(0.00001)
            lgpio.gpio_write(self.gpio_handle, TRIGGER_PIN, 0)
            
            # Read echo (simplified - implement proper timing)
            # This is a placeholder - implement actual ultrasonic reading
            return 50.0
        except Exception as e:
            self.get_logger().error(f'Error reading ultrasonic: {e}')
            return 0.0

    def read_line_tracking(self) -> list:
        """Read line tracking sensors."""
        if not self.hardware_available:
            # Simulated data
            return [0, 1, 0]
        
        try:
            # GPIO pins for line tracking (example - adjust to your setup)
            IR_PINS = [17, 27, 22]
            readings = []
            for pin in IR_PINS:
                reading = lgpio.gpio_read(self.gpio_handle, pin)
                readings.append(reading)
            return readings
        except Exception as e:
            self.get_logger().error(f'Error reading line tracking: {e}')
            return [0, 0, 0]

    def read_battery_voltage(self) -> float:
        """Read battery voltage."""
        if not self.hardware_available:
            # Simulated data
            return 7.4  # Typical 2S LiPo
        
        try:
            # Implement actual battery reading (ADC, voltage divider, etc.)
            return 7.4
        except Exception as e:
            self.get_logger().error(f'Error reading battery: {e}')
            return 0.0

    def publish_sensors(self):
        """Publish all sensor data."""
        # Ultrasonic
        ultrasonic_msg = Float32()
        ultrasonic_msg.data = self.read_ultrasonic()
        self.ultrasonic_pub.publish(ultrasonic_msg)
        
        # Line tracking
        line_data = self.read_line_tracking()
        line_msg = Int32MultiArray()
        line_msg.data = line_data
        self.line_tracking_pub.publish(line_msg)
        
        # Battery
        battery_voltage = self.read_battery_voltage()
        battery_msg = BatteryState()
        battery_msg.voltage = battery_voltage
        battery_msg.percentage = min(100.0, max(0.0, (battery_voltage - 6.0) / 1.4 * 100))
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self.battery_pub.publish(battery_msg)
        
        # System status
        status_msg = Bool()
        status_msg.data = self.hardware_available
        self.system_status_pub.publish(status_msg)

    def destroy_node(self):
        """Cleanup on shutdown."""
        if self.gpio_handle is not None:
            try:
                lgpio.gpiochip_close(self.gpio_handle)
            except:
                pass
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = SensorDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

