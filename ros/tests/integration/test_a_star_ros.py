# File: /Omega-Code/ros/tests/integration/test_a_star_ros.py

import unittest
from unittest.mock import patch, MagicMock
import rospy
from scripts.a_star_ros import main

class TestAStarROSIntegration(unittest.TestCase):
    @patch('scripts.a_star_ros.rospy')
    def test_a_star_node(self, mock_rospy):
        mock_pub = MagicMock()
        mock_rospy.Publisher.return_value = mock_pub
        mock_rospy.wait_for_message.return_value = MagicMock()
        mock_rospy.init_node.return_value = None

        main()

        mock_rospy.init_node.assert_called_with('a_star')
        mock_rospy.wait_for_message.assert_called_with('/map', OccupancyGrid)
        mock_rospy.Publisher.assert_called_with('/a_star_path', PoseStamped, queue_size=10)

if __name__ == '__main__':
    unittest.main()

