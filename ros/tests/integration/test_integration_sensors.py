import unittest
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int32MultiArray
import time

class TestIntegrationSensors(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_integration_sensors', anonymous=True)
        self.ultrasonic_data = None
        self.line_tracking_data = None

        rospy.Subscriber('/ultrasonic', Range, self.ultrasonic_callback)
        rospy.Subscriber('/line_tracking/data', Int32MultiArray, self.line_tracking_callback)

    def ultrasonic_callback(self, data):
        self.ultrasonic_data = data.range

    def line_tracking_callback(self, data):
        self.line_tracking_data = data.data

    def test_integration(self):
        timeout = time.time() + 10  # 10 seconds timeout
        while not rospy.is_shutdown() and time.time() < timeout:
            if self.ultrasonic_data is not None and self.line_tracking_data is not None:
                break
            time.sleep(0.1)

        self.assertIsNotNone(self.ultrasonic_data, "No ultrasonic data received")
        self.assertIsNotNone(self.line_tracking_data, "No line tracking data received")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('test_integration_sensors', 'TestIntegrationSensors', TestIntegrationSensors)

