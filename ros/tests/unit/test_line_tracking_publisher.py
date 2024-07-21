# File: /Omega-Code/ros/tests/unit/test_line_tracking_publisher.py

import unittest
from unittest.mock import patch, MagicMock
import rospy
from scripts.line_tracking_publisher import publish_line_tracking, read_line_sensors

class TestLineTrackingPublisher(unittest.TestCase):
    @patch('scripts.line_tracking_publisher.GPIO.input', side_effect=[1, 0, 1])
    def test_read_line_sensors(self, mock_gpio_input):
        result = read_line_sensors()
        self.assertEqual(result, [1, 0, 1])

    @patch('scripts.line_tracking_publisher.GPIO.input', side_effect=[1, 0, 1])
    @patch('scripts.line_tracking_publisher.rospy.Publisher')
    def test_publish_line_tracking(self, mock_publisher):
        mock_pub_instance = MagicMock()
        mock_publisher.return_value = mock_pub_instance

        with patch('scripts.line_tracking_publisher.rospy.is_shutdown', side_effect=[False, True]):
            publish_line_tracking()

        mock_publisher.assert_called_with('line_tracking/data', Int32MultiArray, queue_size=10)
        self.assertTrue(mock_pub_instance.publish.called)

if __name__ == '__main__':
    unittest.main()

