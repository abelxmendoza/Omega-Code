"""Unit tests covering the custom lightshow lighting pattern."""

from pathlib import Path
import sys
import types
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

if 'rpi_ws281x' not in sys.modules:
    class _MockStrip:
        def __init__(self, *_, **__):
            pass

        def begin(self):
            return None

        def numPixels(self):
            return 8

        def setPixelColor(self, *_):
            return None

        def show(self):
            return None

    mock_module = types.ModuleType('rpi_ws281x')
    mock_module.Adafruit_NeoPixel = _MockStrip
    mock_module.Color = lambda r, g, b: (r << 16) | (g << 8) | b
    mock_module.WS2811_STRIP_GRB = 0
    sys.modules['rpi_ws281x'] = mock_module

from controllers.lighting.led_control import LedController


@patch('controllers.lighting.led_control.time.sleep', return_value=None)
def test_lightshow_updates_strip(mock_sleep):
    controller = LedController(num_pixels=6)

    mock_strip = MagicMock()
    mock_strip.setPixelColor = MagicMock()
    mock_strip.show = MagicMock()
    controller.strip = mock_strip
    controller.num_pixels = 6

    controller.lightshow(0xFF3366, interval=120, brightness=0.75, cycles=1)

    assert mock_strip.setPixelColor.call_count > 0
    assert mock_strip.show.call_count > 0


@patch('controllers.lighting.led_control.time.sleep', return_value=None)
def test_set_led_routes_to_lightshow(mock_sleep):
    controller = LedController(num_pixels=6)
    controller.lightshow = MagicMock()

    controller.set_led(0x112233, mode='single', pattern='lightshow', interval=150, brightness=0.5)

    controller.lightshow.assert_called_once_with(0x112233, interval=150, brightness=0.5)


@patch('controllers.lighting.led_control.time.sleep', return_value=None)
def test_set_led_respects_mode_alias(mock_sleep):
    controller = LedController(num_pixels=6)
    controller.lightshow = MagicMock()

    controller.set_led(0x445566, mode='lightshow', pattern='static', interval=90, brightness=0.6)

    controller.lightshow.assert_called_once_with(0x445566, interval=90, brightness=0.6)
