import unittest
from sensor_msgs.msg import Range
from std_msgs.msg import Int32MultiArray
import visualize_sensor_data

class TestVisualizeSensorData(unittest.TestCase):

    def test_ultrasonic_callback(self):
        data = Range(range=2.5)
        visualize_sensor_data.ultrasonic_callback(data)
        self.assertEqual(visualize_sensor_data.ultrasonic_data[-1], 2.5)

    def test_line_tracking_callback(self):
        data = Int32MultiArray(data=[1, 0, 1])
        visualize_sensor_data.line_tracking_callback(data)
        self.assertEqual(visualize_sensor_data.line_tracking_data[-1], [1, 0, 1])

if __name__ == '__main__':
    unittest.main()
