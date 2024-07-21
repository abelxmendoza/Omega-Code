# File: /Omega-Code/servers/robot-controller-backend/tests/e2e/ros/autonomous_driving_with_astar_test.py

import unittest
from unittest.mock import patch
import autonomous_driving_with_astar as ada

class TestAutonomousDrivingWithAStar(unittest.TestCase):
    @patch('ada.rospy')
    @patch('ada.model.predict')
    def test_autonomous_driving(self, mock_predict, mock_rospy):
        mock_predict.return_value = [1.0]
        
        # Simulate image input
        ada.image_callback(mock.Mock())

        # Check if the prediction is handled correctly
        self.assertEqual(ada.cmd.linear.x, 0.5)
        self.assertEqual(ada.cmd.angular.z, 0.0)

if __name__ == '__main__':
    unittest.main()

