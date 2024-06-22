# File: /Users/abel_elreaper/Desktop/Omega-Code/servers/robot-controller-backend/tests/conftest.py

import sys
import types

class MockWS281x:
    def __init__(self, *args, **kwargs):
        pass
    def begin(self):
        pass
    def numPixels(self):
        return 8
    def setPixelColor(self, i, color):
        pass
    def show(self):
        pass

mock_rpi_ws281x = types.ModuleType("rpi_ws281x")
mock_rpi_ws281x.Adafruit_NeoPixel = MockWS281x
mock_rpi_ws281x.Color = lambda r, g, b: (r << 16) | (g << 8) | b

sys.modules["rpi_ws281x"] = mock_rpi_ws281x
