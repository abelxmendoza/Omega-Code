# File: /Omega-Code/servers/robot_controller_backend/tests/mock_pca9685_test.py

"""
Unit tests for the mock PCA9685 module using unittest.
"""

import unittest
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

try:
    from utils.mock_pca9685 import PCA9685
except ImportError:
    # Skip if module doesn't exist
    PCA9685 = None

class TestMockPCA9685(unittest.TestCase):
    def setUp(self):
        self.pca = PCA9685(address=0x40, debug=True)

    def test_set_pwm_freq(self):
        """
        Test the setPWMFreq function to ensure it sets the frequency correctly.
        """
        self.pca.setPWMFreq(50)
        # Add assertions to verify the behavior of setPWMFreq method
        self.assertEqual(self.pca.frequency, 50, "The PWM frequency should be set to 50.")

    def test_set_servo_pulse(self):
        """
        Test the setServoPulse function to ensure it sets the pulse length correctly.
        """
        self.pca.setServoPulse(0, 1500)
        # Add assertions to verify the behavior of setServoPulse method
        self.assertIn(0, self.pca.pulse_length, "Channel 0 should have a pulse length set.")
        self.assertEqual(self.pca.pulse_length[0], 1500, "The pulse length for channel 0 should be set to 1500.")

if __name__ == '__main__':
    unittest.main()
