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
    from movement.mock_pca9685 import MockPCA9685 as PCA9685
except ImportError:
    PCA9685 = None


@unittest.skipIf(PCA9685 is None, "MockPCA9685 module not available")
class TestMockPCA9685(unittest.TestCase):
    def setUp(self):
        self.pca = PCA9685(address=0x40, debug=False)

    def test_set_pwm_freq(self):
        """setMotorPwm works without raising — no hardware call in mock."""
        self.pca.setMotorPwm(0, 1000)
        self.assertEqual(self.pca.channels[0], 1000)

    def test_set_servo_pulse(self):
        """setServoPulse converts pulse-width to a channel value without crashing."""
        self.pca.setServoPulse(8, 1500)
        # Pulse 1500 µs → PWM count = 1500 * 4096 / 20000 = 307 (non-zero)
        self.assertGreater(self.pca.channels[8], 0)

if __name__ == '__main__':
    unittest.main()
