import unittest
from unittest.mock import patch
import start_sensors

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

