"""
LED Test Script for WS2812/WS2811 Strips (NeoPixels)

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/basic_led_test.py

This script serves as a standalone utility to verify LED functionality for addressable RGB LED strips
driven by the rpi_ws281x library. It interfaces with the LedController module to apply solid colors
or run a predefined color test sequence.

Usage:
- Run with a named color argument (e.g., `python basic_led_test.py red`)
- Run with RGB values (e.g., `python basic_led_test.py 255 128 64`)
- Run without arguments to cycle through red, green, and blue automatically

Supported Colors:
- Named: red, green, blue, off
- RGB: three integer values from 0–255
"""

from controllers.lighting.led_control import LedController
from rpi_ws281x import Color
import sys

if __name__ == "__main__":
    led = LedController()

    if len(sys.argv) == 2:
        # Named color mode
        c = sys.argv[1].lower()
        if c == 'red':
            led.color_wipe(Color(255, 0, 0))
        elif c == 'green':
            led.color_wipe(Color(0, 255, 0))
        elif c == 'blue':
            led.color_wipe(Color(0, 0, 255))
        elif c == 'off':
            led.color_wipe(Color(0, 0, 0))
        else:
            print("Invalid color. Use: red, green, blue, or off.")
            sys.exit(1)

    elif len(sys.argv) == 4:
        # RGB mode
        try:
            r = int(sys.argv[1])
            g = int(sys.argv[2])
            b = int(sys.argv[3])
            led.color_wipe(Color(r, g, b))
        except ValueError:
            print("Invalid RGB values. Please provide integers from 0 to 255.")
            sys.exit(1)

    else:
        # No arguments: cycle through red, green, and blue (functionality test)
        print("🚦 Starting LED functionality test. Press Ctrl+C to exit.")
        led.test_colors()
        print("✅ LED test complete.")
