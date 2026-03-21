import unittest
import pytest
from unittest.mock import patch

pytestmark = pytest.mark.skip(reason="uses ROS1 tools (rosrun/roslaunch) — not applicable on ROS2")

class TestStartSensors(unittest.TestCase):

    @patch('start_sensors.subprocess.Popen')
    def test_start_simulated_sensors(self, mock_popen):
        start_sensors.start_simulated_sensors()
        mock_popen.assert_called_with(["rosrun", "ros_scripts", "simulated_sensors.py"])

    @patch('start_sensors.subprocess.Popen')
    def test_start_raspberry_pi_sensors(self, mock_popen):
        start_sensors.start_raspberry_pi_sensors()
        mock_popen.assert_called_with(["ssh", "omega1@$TAILSCALE_IP_PI", "roslaunch", "omega_robot", "robot_sensors.launch"])

if __name__ == '__main__':
    unittest.main()

