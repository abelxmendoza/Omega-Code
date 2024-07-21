import unittest
from sensor_msgs.msg import Range
from std_msgs.msg import Int32MultiArray
from unittest.mock import patch
import log_sensor_data
from datetime import datetime

class TestLogSensorData(unittest.TestCase):

    @patch('log_sensor_data.open')
    def test_ultrasonic_callback(self, mock_open):
        data = Range(range=2.5)
        log_sensor_data.ultrasonic_callback(data)
        mock_open.assert_called_with('/path/to/ultrasonic_data.log', 'a')
        mock_open.return_value.write.assert_called_with(f"{datetime.now()}: {data.range}\n")

    @patch('log_sensor_data.open')
    def test_line_tracking_callback(self, mock_open):
        data = Int32MultiArray(data=[1, 0, 1])
        log_sensor_data.line_tracking_callback(data)
        mock_open.assert_called_with('/path/to/line_tracking_data.log', 'a')
        mock_open.return_value.write.assert_called_with(f"{datetime.now()}: {data.data}\n")

if __name__ == '__main__':
    unittest.main()

