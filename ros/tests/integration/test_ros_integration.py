# File: /Omega-Code/ros/tests/integration/test_ros_integration.py

import unittest
from unittest.mock import patch, MagicMock
import rospy
from std_msgs.msg import Int32MultiArray, Float32
from sensor_msgs.msg import Image

class TestROSIntegration(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        self.camera_pub = rospy.Publisher('camera/image', Image, queue_size=10)
        self.line_pub = rospy.Publisher('line_tracking/data', Int32MultiArray, queue_size=10)
        self.ultrasonic_pub = rospy.Publisher('ultrasonic/distance', Float32, queue_size=10)

    @patch('scripts.camera_publisher.cv2.VideoCapture')
    @patch('scripts.camera_publisher.CvBridge')
    @patch('scripts.line_tracking_publisher.GPIO.input', side_effect=[1, 0, 1])
    @patch('scripts.ultrasonic_publisher.distance', return_value=100.0)
    def test_ros_integration(self, mock_distance, mock_gpio_input, mock_cv_bridge, mock_video_capture):
        # Mock camera publisher
        mock_cap_instance = MagicMock()
        mock_video_capture.return_value = mock_cap_instance
        mock_cap_instance.read.return_value = (True, MagicMock())
        mock_bridge_instance = MagicMock()
        mock_cv_bridge.return_value = mock_bridge_instance

        # Start camera publisher
        with patch('scripts.camera_publisher.rospy.is_shutdown', side_effect=[False, True]):
            import scripts.camera_publisher
            scripts.camera_publisher.publish_camera()

        # Publish test data for line tracking and ultrasonic
        self.line_pub.publish(Int32MultiArray(data=[1, 0, 1]))
        self.ultrasonic_pub.publish(Float32(data=100.0))

        # Check that the data was published correctly
        self.assertTrue(mock_bridge_instance.cv2_to_imgmsg.called)
        self.assertEqual(mock_gpio_input.call_count, 3)
        self.assertTrue(mock_distance.called)

if __name__ == '__main__':
    unittest.main()

