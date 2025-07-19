"""
LED Control Module for WS2812/WS2811 Strips (NeoPixels)

This module provides a Python class for controlling addressable RGB LED strips
using the rpi_ws281x library. It supports color wipe animations, predefined color tests,
and can be easily extended for full RGB or hex color input.

Main Features:
- Initialize and configure the LED strip (pin, brightness, frequency, etc.)
- Perform a color wipe animation across the strip
- Run a built-in red-green-blue color test sequence

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/led_control.py
"""

from rpi_ws281x import Adafruit_NeoPixel, Color, WS2811_STRIP_GRB
import time

class LedController:
    def __init__(self, num_pixels=16, pin=18, brightness=255):
        """
        Initialize the LED strip with the given parameters.

        Args:
            num_pixels (int): Number of LEDs in the strip.
            pin (int): GPIO pin connected to the LEDs.
            brightness (int): Brightness level (0-255).
        """
        self.strip = Adafruit_NeoPixel(
            num_pixels,
            pin,
            800000,      # frequency in Hz
            10,          # DMA channel
            False,       # invert signal
            brightness,
            0,           # channel
            WS2811_STRIP_GRB
        )
        self.strip.begin()
        self.num_pixels = num_pixels

    def color_wipe(self, color, wait_ms=50):
        """
        Wipe the specified color across the strip, one LED at a time.

        Args:
            color (Color): rpi_ws281x Color object (e.g. Color(255, 0, 0) for red).
            wait_ms (int): Delay between lighting each LED, in milliseconds.
        """
        for i in range(self.num_pixels):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def test_colors(self):
        """
        Test red, green, and blue colors sequentially across the strip.
        Turns the LEDs off afterward.
        """
        print("➡️ Red")
        self.color_wipe(Color(255, 0, 0))
        time.sleep(1)
        print("➡️ Green")
        self.color_wipe(Color(0, 255, 0))
        time.sleep(1)
        print("➡️ Blue")
        self.color_wipe(Color(0, 0, 255))
        time.sleep(1)
        self.color_wipe(Color(0, 0, 0))  # Turn off
