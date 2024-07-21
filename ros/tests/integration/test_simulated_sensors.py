# File: /Omega-Code/ros/tests/integration/test_simulated_sensors.py

import unittest
import rospy
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import time

class TestSimulatedSensors(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_simulated_sensors', anonymous=True)
        self.camera_data = None
        self.ultrasonic_data = None
        self.line_tracking_data = None

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        rospy.Subscriber('/ultrasonic', Range, self.ultrasonic_callback)
        rospy.Subscriber('/line_tracking', Float32, self.line_tracking_callback)

        self.bridge = CvBridge()

    def camera_callback(self, data):
        self.camera_data = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def ultrasonic_callback(self, data):
        self.ultrasonic_data = data.range

    def line_tracking_callback(self, data):
        self.line_tracking_data = data.data

    def test_simulated_sensors(self):
        timeout = time.time() + 10  # 10 seconds timeout
        while not rospy.is_shutdown() and time.time() < timeout:
            if self.camera_data is not None and self.ultrasonic_data is not None and self.line_tracking_data is not None:
                break
            time.sleep(0.1)

        self.assertIsNotNone(self.camera_data, "No camera data received")
        self.assertIsNotNone(self.ultrasonic_data, "No ultrasonic data received")
        self.assertIsNotNone(self.line_tracking_data, "No line tracking data received")

        # Check if the data matches the simulated values
        self.assertTrue(np.array_equal(self.camera_data, np.zeros((480, 640, 3), np.uint8)), "Incorrect camera data")
        self.assertEqual(self.ultrasonic_data, 1.0, "Incorrect ultrasonic data")
        self.assertEqual(self.line_tracking_data, 0.5, "Incorrect line tracking data")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('test_simulated_sensors', 'TestSimulatedSensors', TestSimulatedSensors)
