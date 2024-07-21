# File: /Omega-Code/ros/tests/integration/test_rrt_ros.py

import unittest
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import time

class TestRRTRos(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_rrt_ros', anonymous=True)
        self.path = []
        rospy.Subscriber('/rrt_path', PoseStamped, self.path_callback)

    def path_callback(self, data):
        self.path.append((data.pose.position.x, data.pose.position.y))

    def test_rrt_path_planning(self):
        # Simulate occupancy grid
        grid_msg = OccupancyGrid()
        grid_msg.info.width = 5
        grid_msg.info.height = 5
        grid_msg.data = [0, 1, 0, 0, 0,
                         0, 1, 0, 1, 0,
                         0, 0, 0, 1, 0,
                         1, 1, 0, 1, 0,
                         0, 0, 0, 0, 0]
        grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        timeout = time.time() + 10  # 10 seconds timeout

        while not rospy.is_shutdown() and time.time() < timeout:
            grid_pub.publish(grid_msg)
            if len(self.path) > 0:
                break
            time.sleep(0.1)

        self.assertGreater(len(self.path), 0, "No path found")
        expected_path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (3, 4), (4, 4)]
        self.assertEqual(self.path, expected_path, "Incorrect path found")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('test_rrt_ros', 'TestRRTRos', TestRRTRos)

