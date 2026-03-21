# File: /Omega-Code/ros/tests/unit/test_autonomous_driving_with_astar.py

import unittest
import pytest
from unittest.mock import patch, MagicMock

pytestmark = pytest.mark.skip(reason="requires rospy (ROS1) — not available on ROS2")

class TestAutonomousDrivingWithAStar(unittest.TestCase):
    @patch('scripts.autonomous_driving_with_astar.AutonomousDriving.a_star_search')
    @patch('scripts.autonomous_driving_with_astar.AutonomousDriving.move_to_position')
    def test_control_robot(self, mock_move_to_position, mock_a_star_search):
        mock_a_star_search.return_value = [(0, 0), (1, 1)]
        ad = AutonomousDriving()
        ad.latest_image = MagicMock()
        ad.latest_ultrasonic = MagicMock()
        ad.latest_line_tracking = MagicMock()
        ad.control_robot()
        self.assertTrue(mock_move_to_position.called)

if __name__ == '__main__':
    unittest.main()
