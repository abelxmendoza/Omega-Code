# File: /Omega-Code/ros/tests/unit/test_simulated_sensors.py

import unittest
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np

class TestSimulatedSensors(unittest.TestCase):

    def setUp(self):
        self.bridge = CvBridge()

    def test_publish_camera(self):
        # Simulate publishing a blank image
        image = np.zeros((480, 640, 3), np.uint8)
        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.assertEqual(msg.height, 480)
        self.assertEqual(msg.width, 640)
        self.assertEqual(msg.encoding, "bgr8")

    def test_publish_ultrasonic(self):
        # Simulate publishing a constant distance
        range_msg = Range()
        range_msg.range = 1.0
        self.assertEqual(range_msg.range, 1.0)

    def test_publish_line_tracking(self):
        # Simulate publishing a constant value
        line_msg = Float32()
        line_msg.data = 0.5
        self.assertEqual(line_msg.data, 0.5)

if __name__ == '__main__':
    unittest.main()

