# File: /Omega-Code/ros/tests/e2e/test_full_system.py

import unittest
import rospy
from std_msgs.msg import Int32MultiArray, Float32
from sensor_msgs.msg import Image
import time

class TestFullSystem(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_full_system_node', anonymous=True)
        self.camera_pub = rospy.Publisher('camera/image', Image, queue_size=10)
        self.line_pub = rospy.Publisher('line_tracking/data', Int32MultiArray, queue_size=10)
        self.ultrasonic_pub = rospy.Publisher('ultrasonic/distance', Float32, queue_size=10)

        self.camera_data = None
        self.line_data = None
        self.ultrasonic_data = None

        rospy.Subscriber('camera/image', Image, self.camera_callback)
        rospy.Subscriber('line_tracking/data', Int32MultiArray, self.line_callback)
        rospy.Subscriber('ultrasonic/distance', Float32, self.ultrasonic_callback)

    def camera_callback(self, data):
        self.camera_data = data

    def line_callback(self, data):
        self.line_data = data

    def ultrasonic_callback(self, data):
        self.ultrasonic_data = data

    def test_full_system(self):
        # Simulate publishing camera data
        self.camera_pub.publish(Image())
        time.sleep(1)
        self.assertIsNotNone(self.camera_data, "Camera data should be received")

        # Simulate publishing line tracking data
        self.line_pub.publish(Int32MultiArray(data=[1, 0, 1]))
        time.sleep(1)
        self.assertIsNotNone(self.line_data, "Line tracking data should be received")
        self.assertEqual(self.line_data.data, [1, 0, 1])

        # Simulate publishing ultrasonic data
        self.ultrasonic_pub.publish(Float32(data=100.0))
        time.sleep(1)
        self.assertIsNotNone(self.ultrasonic_data, "Ultrasonic data should be received")
        self.assertEqual(self.ultrasonic_data.data, 100.0)

if __name__ == '__main__':
    unittest.main()

