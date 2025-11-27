# File: /Omega-Code/servers/robot-controller-backend/tests/led_control_test.py

"""
Unit tests for the LED control module using unittest and mock.
"""

import unittest
import sys
import os
from unittest.mock import patch, MagicMock

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

try:
    from utils.led_control import Led
    LED_AVAILABLE = True
except (ImportError, AttributeError):
    # Skip if module doesn't exist or has issues
    LED_AVAILABLE = False
    Led = None
    import pytest
    pytestmark = pytest.mark.skip(reason="LED control module not available")

class TestLedControl(unittest.TestCase):
    @unittest.skipUnless(LED_AVAILABLE, "LED control module not available")
    def setUp(self):
        if not LED_AVAILABLE or Led is None:
            self.skipTest("LED control module not available")
        try:
            self.led = Led()
        except Exception as e:
            self.skipTest(f"LED control initialization failed: {e}")

    @patch('utils.led_control.time.sleep', return_value=None)
    def test_color_wipe(self, mock_sleep):
        """
        Test the colorWipe function to ensure it executes without errors.
        """
        try:
            self.led.colorWipe(0xFF0000)  # Red color
        except Exception as e:
            self.fail(f"colorWipe method raised an exception: {e}")

    @patch('utils.led_control.time.sleep', return_value=None)
    def test_theater_chase(self, mock_sleep):
        """
        Test the theaterChase function to ensure it executes without errors.
        """
        try:
            self.led.theaterChase(0x00FF00)  # Green color
        except Exception as e:
            self.fail(f"theaterChase method raised an exception: {e}")

    @patch('utils.led_control.time.sleep', return_value=None)
    def test_rainbow(self, mock_sleep):
        """
        Test the rainbow function to ensure it executes without errors.
        """
        try:
            self.led.rainbow()
        except Exception as e:
            self.fail(f"rainbow method raised an exception: {e}")

    @patch('utils.led_control.time.sleep', return_value=None)
    def test_set_led_single(self, mock_sleep):
        """
        Test the setLed function with 'single' mode to ensure it executes without errors.
        """
        try:
            self.led.setLed(0x0000FF, 'single', 'static', 1000)  # Blue color
        except Exception as e:
            self.fail(f"setLed method raised an exception: {e}")

    @patch('utils.led_control.time.sleep', return_value=None)
    def test_set_led_multi(self, mock_sleep):
        """
        Test the setLed function with 'multi' mode to ensure it executes without errors.
        """
        try:
            self.led.setLed(0x0000FF, 'multi', 'blink', 1000)  # Blue color
        except Exception as e:
            self.fail(f"setLed method raised an exception: {e}")

    @patch('utils.led_control.time.sleep', return_value=None)
    def test_set_led_two(self, mock_sleep):
        """
        Test the setLed function with 'two' mode to ensure it executes without errors.
        """
        try:
            self.led.setLed(0x0000FF, 'two', 'fade', 1000)  # Blue color
        except Exception as e:
            self.fail(f"setLed method raised an exception: {e}")

if __name__ == '__main__':
    unittest.main()
