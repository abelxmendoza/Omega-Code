# File: /Omega-Code/servers/robot-controller-backend/tests/servo_control_test.py

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
        Test the setServoPwm function for the horizontal servo to ensure it executes without errors.
        """
        # Create an instance of the mock PCA9685
        mock_pca9685_instance = MockPCA9685.return_value
        
        # Instantiate the Servo class
        servo = Servo()
        
        # Test setting PWM for horizontal servo
        servo.setServoPwm('horizontal', 10)
        
        # Check if setServoPulse was called with the correct values
        mock_pca9685_instance.setServoPulse.assert_called_with(8, 2500 - int((10 + 10) / 0.09))

    @patch('controllers.servo_control.PCA9685')
    def test_set_servo_pwm_vertical(self, MockPCA9685):
        """
        Test the setServoPwm function for the vertical servo to ensure it executes without errors.
        """
        # Create an instance of the mock PCA9685
        mock_pca9685_instance = MockPCA9685.return_value
        
        # Instantiate the Servo class
        servo = Servo()
        
        # Test setting PWM for vertical servo
        servo.setServoPwm('vertical', 10)
        
        # Check if setServoPulse was called with the correct values
        mock_pca9685_instance.setServoPulse.assert_called_with(9, 500 + int((10 + 10) / 0.09))

if __name__ == '__main__':
    unittest.main()
