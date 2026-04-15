import unittest
import sys
import os
from unittest.mock import patch, MagicMock

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from controllers.lighting import led_control as lc


class TestControllersLedControl(unittest.TestCase):
    """Tests for the LedController class.

    The class uses rpi_ws281x.Adafruit_NeoPixel internally (not PixelStrip);
    the conftest.py already stubs rpi_ws281x globally so no additional patching
    is needed for basic construction tests.
    """

    def test_init_creates_stub_on_import_error(self):
        """When Adafruit_NeoPixel raises ImportError, falls back to StubPixelStrip."""
        with patch('controllers.lighting.led_control.Adafruit_NeoPixel',
                   side_effect=ImportError('no hw')):
            led = lc.LedController(num_pixels=8)
        self.assertTrue(led._is_stub)
        self.assertIsInstance(led.strip, lc.StubPixelStrip)

    def test_init_creates_stub_on_runtime_error(self):
        """When begin() raises a non-permission RuntimeError, falls back to StubPixelStrip."""
        mock_strip = MagicMock()
        mock_strip.begin.side_effect = RuntimeError('hardware failure')
        with patch('controllers.lighting.led_control.Adafruit_NeoPixel',
                   return_value=mock_strip):
            led = lc.LedController(num_pixels=4)
        self.assertTrue(led._is_stub)

    def test_num_pixels_stored(self):
        """num_pixels kwarg is stored on the controller."""
        led = lc.LedController(num_pixels=12)
        self.assertEqual(led.num_pixels, 12)

    def test_invalid_num_pixels_raises(self):
        with self.assertRaises(ValueError):
            lc.LedController(num_pixels=0)
        with self.assertRaises(ValueError):
            lc.LedController(num_pixels=9999)

    def test_invalid_pin_raises(self):
        with self.assertRaises(ValueError):
            lc.LedController(num_pixels=8, pin=100)
