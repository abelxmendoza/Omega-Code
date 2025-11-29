# File: /Omega-Code/servers/robot_controller_backend/tests/e2e/ros/autonomous_driving_with_astar_test.py

import unittest
import sys
import os
from unittest.mock import patch, Mock

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))

try:
    import autonomous_driving_with_astar as ada
except ImportError:
    # Skip if module doesn't exist
    ada = None
    import pytest
    pytestmark = pytest.mark.skip(reason="autonomous_driving_with_astar module not available")

class TestAutonomousDrivingWithAStar(unittest.TestCase):
    @unittest.skipIf(ada is None, "autonomous_driving_with_astar module not available")
    @patch('autonomous_driving_with_astar.rospy')
    @patch('autonomous_driving_with_astar.model.predict')
    def test_autonomous_driving(self, mock_predict, mock_rospy):
        if ada is None:
            self.skipTest("autonomous_driving_with_astar module not available")
        
        mock_predict.return_value = [1.0]
        
        # Simulate image input
        ada.image_callback(Mock())

        # Check if the prediction is handled correctly
        self.assertEqual(ada.cmd.linear.x, 0.5)
        self.assertEqual(ada.cmd.angular.z, 0.0)

if __name__ == '__main__':
    unittest.main()

