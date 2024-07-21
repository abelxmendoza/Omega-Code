# File: /Omega-Code/ros/tests/unit/test_autonomous_driving_with_astar.py

import unittest
from unittest.mock import patch, MagicMock
import rospy
from scripts.autonomous_driving_with_astar import AutonomousDriving

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
