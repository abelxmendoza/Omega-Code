import unittest
import rospy
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import time

class TestFullSystem(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_full_system', anonymous=True)
        self.camera_data = None
        self.ultrasonic_data = None
        self.line_tracking_data = None
        self.path = []

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        rospy.Subscriber('/ultrasonic', Range, self.ultrasonic_callback)
        rospy.Subscriber('/line_tracking', Float32, self.line_tracking_callback)
        rospy.Subscriber('/rrt_path', PoseStamped, self.path_callback)

        self.bridge = CvBridge()

    def camera_callback(self, data):
        self.camera_data = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def ultrasonic_callback(self, data):
        self.ultrasonic_data = data.range

    def line_tracking_callback(self, data):
        self.line_tracking_data = data.data

    def path_callback(self, data):
        self.path.append((data.pose.position.x, data.pose.position.y))

    def test_full_system(self):
        timeout = time.time() + 10  # 10 seconds timeout
        grid_msg = OccupancyGrid()
        grid_msg.info.width = 5
        grid_msg.info.height = 5
        grid_msg.data = [0, 1, 0, 0, 0,
                         0, 1, 0, 1, 0,
                         0, 0, 0, 1, 0,
                         1, 1, 0, 1, 0,
                         0, 0, 0, 0, 0]
        grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        while not rospy.is_shutdown() and time.time() < timeout:
            grid_pub.publish(grid_msg)
            if (self.camera_data is not None and self.ultrasonic_data is not None and
                    self.line_tracking_data is not None and len(self.path) > 0):
                break
            time.sleep(0.1)

        self.assertIsNotNone(self.camera_data, "No camera data received")
        self.assertIsNotNone(self.ultrasonic_data, "No ultrasonic data received")
        self.assertIsNotNone(self.line_tracking_data, "No line tracking data received")
        self.assertGreater(len(self.path), 0, "No path found")

        self.assertTrue(np.array_equal(self.camera_data, np.zeros((480, 640, 3), np.uint8)), "Incorrect camera data")
        self.assertEqual(self.ultrasonic_data, 1.0, "Incorrect ultrasonic data")
        self.assertEqual(self.line_tracking_data, 0.5, "Incorrect line tracking data")

        expected_path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (3, 4), (4, 4)]
        self.assertEqual(self.path, expected_path, "Incorrect path found")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('test_full_system', 'TestFullSystem', TestFullSystem)
