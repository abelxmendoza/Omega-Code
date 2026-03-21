# File: /Omega-Code/ros/tests/unit/test_sensor_fusion.py

import unittest
import pytest
from unittest.mock import patch, MagicMock

pytestmark = pytest.mark.skip(reason="requires rospy (ROS1) — not available on ROS2")

class TestSensorFusion(unittest.TestCase):
    @patch('scripts.sensor_fusion.SensorFusion.fuse_sensors')
    def test_image_callback(self, mock_fuse_sensors):
        sf = SensorFusion()
        data = MagicMock()
        sf.image_callback(data)
        self.assertTrue(mock_fuse_sensors.called)

    @patch('scripts.sensor_fusion.SensorFusion.fuse_sensors')
    def test_ultrasonic_callback(self, mock_fuse_sensors):
        sf = SensorFusion()
        data = MagicMock()
        sf.ultrasonic_callback(data)
        self.assertTrue(mock_fuse_sensors.called)

    @patch('scripts.sensor_fusion.SensorFusion.fuse_sensors')
    def test_line_tracking_callback(self, mock_fuse_sensors):
        sf = SensorFusion()
        data = MagicMock()
        sf.line_tracking_callback(data)
        self.assertTrue(mock_fuse_sensors.called)

if __name__ == '__main__':
    unittest.main()

