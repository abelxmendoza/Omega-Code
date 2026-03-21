# File: /Omega-Code/ros/tests/unit/test_autonomous_driving.py

import unittest
import pytest
from unittest.mock import patch, MagicMock

pytestmark = pytest.mark.skip(reason="requires rospy (ROS1) — not available on ROS2")

class TestAutonomousDriving(unittest.TestCase):
    @patch('scripts.autonomous_driving.model.predict')
    @patch('scripts.autonomous_driving.cv_bridge.CvBridge.imgmsg_to_cv2')
    def test_image_callback(self, mock_imgmsg_to_cv2, mock_predict):
        mock_imgmsg_to_cv2.return_value = MagicMock()
        mock_predict.return_value = [0.7]

        data = MagicMock()
        image_callback(data)
        mock_predict.assert_called_once()

    @patch('scripts.autonomous_driving.pub.publish')
    def test_control_robot(self, mock_publish):
        prediction = [0.7]
        control_robot(prediction)
        self.assertTrue(mock_publish.called)

if __name__ == '__main__':
    unittest.main()

