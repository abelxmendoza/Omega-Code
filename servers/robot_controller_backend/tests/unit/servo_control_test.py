# File: /Omega-Code/servers/robot_controller_backend/tests/servo_control_test.py

"""
Unit tests for the servo control module using unittest and mock.
"""

import unittest
import sys
import os
from unittest.mock import patch, MagicMock

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

# Mock PCA9685 module before importing servo_control
import types
class MockPCA9685Class:
    def __init__(self, *args, **kwargs):
        pass
    def setPWMFreq(self, *args):
        pass
    def setServoPulse(self, *args):
        pass

mock_pca9685_module = types.ModuleType("utils.pca9685")
mock_pca9685_module.PCA9685 = MockPCA9685Class
sys.modules["utils.pca9685"] = mock_pca9685_module

from controllers.servo_control import Servo

class TestServoControl(unittest.TestCase):

    @patch('controllers.servo_control.PCA9685')
    def test_servo_initialization(self, MockPCA9685):
        """
        Test the initialization of the Servo class to ensure it sets up the PCA9685 correctly.
        """
        # Create an instance of the mock PCA9685
        mock_pca9685_instance = MockPCA9685.return_value
        
        # Instantiate the Servo class
        servo = Servo()
        
        # Check if PCA9685 was initialized with the correct address
        MockPCA9685.assert_called_with(0x40, debug=True)
        
        # Check if setPWMFreq and setServoPulse were called correctly during initialization
        mock_pca9685_instance.setPWMFreq.assert_called_with(50)
        mock_pca9685_instance.setServoPulse.assert_any_call(8, 1500)
        mock_pca9685_instance.setServoPulse.assert_any_call(9, 1500)

    @patch('controllers.servo_control.PCA9685')
    def test_set_servo_pwm_horizontal(self, MockPCA9685):
        """
        Verify horizontal servo pulse is calculated correctly and clamped to the safe range.

        Formula: pulse = 2500 - int((angle + error) / 0.09)
        Safe range: 1200–1800 µs (hardware safety clamp added with the PCA9685 50Hz fix).

        angle=90, error=10 → 2500 - int(100 / 0.09) = 2500 - 1111 = 1389 µs  (in range)
        angle=10, error=10 → 2500 - int(20  / 0.09) = 2500 - 222  = 2278 µs  (> 1800 → clamped)
        """
        mock_pca9685_instance = MockPCA9685.return_value

        servo = Servo()

        # In-range angle: raw pulse 1389 µs — should pass through unclamped
        servo.setServoPwm('horizontal', 90)
        expected_pulse = 2500 - int((90 + 10) / 0.09)   # = 1389
        mock_pca9685_instance.setServoPulse.assert_called_with(8, expected_pulse)

        # Out-of-range angle: raw pulse 2278 µs — safety clamp enforces MAX (1800 µs)
        servo.setServoPwm('horizontal', 10)
        mock_pca9685_instance.setServoPulse.assert_called_with(8, 1800)

    @patch('controllers.servo_control.PCA9685')
    def test_set_servo_pwm_vertical(self, MockPCA9685):
        """
        Verify vertical servo pulse is calculated correctly and clamped to the safe range.

        Formula: pulse = 500 + int((angle + error) / 0.09)
        Safe range: 1200–1800 µs.

        angle=90, error=10 → 500 + int(100 / 0.09) = 500 + 1111 = 1611 µs  (in range)
        angle=10, error=10 → 500 + int(20  / 0.09) = 500 + 222  =  722 µs  (< 1200 → clamped)
        """
        mock_pca9685_instance = MockPCA9685.return_value

        servo = Servo()

        # In-range angle: raw pulse 1611 µs — should pass through unclamped
        servo.setServoPwm('vertical', 90)
        expected_pulse = 500 + int((90 + 10) / 0.09)    # = 1611
        mock_pca9685_instance.setServoPulse.assert_called_with(9, expected_pulse)

        # Out-of-range angle: raw pulse 722 µs — safety clamp enforces MIN (1200 µs)
        servo.setServoPwm('vertical', 10)
        mock_pca9685_instance.setServoPulse.assert_called_with(9, 1200)

if __name__ == '__main__':
    unittest.main()
