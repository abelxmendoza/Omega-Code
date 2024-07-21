# File: /Omega-Code/ros/tests/unit/test_ultrasonic_publisher.py

import unittest
from unittest.mock import patch, MagicMock
import rospy
from scripts.ultrasonic_publisher import publish_ultrasonic, distance

class TestUltrasonicPublisher(unittest.TestCase):
    @patch('scripts.ultrasonic_publisher.GPIO.input', side_effect=[0, 1, 0, 1])
    @patch('scripts.ultrasonic_publisher.time.time', side_effect=[1, 2])
    def test_distance(self, mock_time, mock_gpio_input):
        result = distance()
        self.assertAlmostEqual(result, 17150.0, places=2)

    @patch('scripts.ultrasonic_publisher.distance', return_value=100.0)
    @patch('scripts.ultrasonic_publisher.rospy.Publisher')
    def test_publish_ultrasonic(self, mock_publisher, mock_distance):
        mock_pub_instance = MagicMock()
        mock_publisher.return_value = mock_pub_instance

        with patch('scripts.ultrasonic_publisher.rospy.is_shutdown', side_effect=[False, True]):
            publish_ultrasonic()

        mock_publisher.assert_called_with('ultrasonic/distance', Float32, queue_size=10)
        self.assertTrue(mock_pub_instance.publish.called)

if __name__ == '__main__':
    unittest.main()

