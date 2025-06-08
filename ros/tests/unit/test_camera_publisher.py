# File: /Omega-Code/ros/tests/unit/test_camera_publisher.py

import unittest
from unittest.mock import patch, MagicMock
try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
import rospy
from scripts.camera_publisher import publish_camera

class TestCameraPublisher(unittest.TestCase):
    @unittest.skipIf(cv2 is None, "OpenCV not installed")
    @patch('scripts.camera_publisher.cv2.VideoCapture')
    @patch('scripts.camera_publisher.CvBridge')
    @patch('scripts.camera_publisher.rospy.Publisher')
    def test_publish_camera(self, mock_publisher, mock_cv_bridge, mock_video_capture):
        mock_pub_instance = MagicMock()
        mock_publisher.return_value = mock_pub_instance
        mock_bridge_instance = MagicMock()
        mock_cv_bridge.return_value = mock_bridge_instance
        mock_cap_instance = MagicMock()
        mock_video_capture.return_value = mock_cap_instance
        mock_cap_instance.read.return_value = (True, MagicMock())

        with patch('scripts.camera_publisher.rospy.is_shutdown', side_effect=[False, True]):
            publish_camera()

        mock_publisher.assert_called_with('camera/image', Image, queue_size=10)
        self.assertTrue(mock_pub_instance.publish.called)
        self.assertTrue(mock_cap_instance.read.called)
        self.assertTrue(mock_bridge_instance.cv2_to_imgmsg.called)

if __name__ == '__main__':
    unittest.main()
