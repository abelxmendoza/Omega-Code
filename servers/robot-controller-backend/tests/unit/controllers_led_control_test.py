import unittest
from unittest.mock import patch, MagicMock

from controllers.lighting import led_control as lc

class TestControllersLedControl(unittest.TestCase):
    @patch('controllers.lighting.led_control.PixelStrip')
    def test_init_calls_pixel_strip(self, MockStrip):
        instance = MockStrip.return_value
        led = lc.LedControl()
        MockStrip.assert_called_with(
            lc.LED_COUNT,
            lc.LED_PIN,
            lc.LED_FREQ_HZ,
            lc.LED_DMA,
            lc.LED_INVERT,
            lc.LED_BRIGHTNESS,
            lc.LED_CHANNEL,
        )
        instance.begin.assert_called_once()

    def test_convert_color_valid(self):
        led = lc.LedControl()
        result = led._convert_color('RGB', 0x112233)
        self.assertIsInstance(result, int)

if __name__ == '__main__':
    unittest.main()
