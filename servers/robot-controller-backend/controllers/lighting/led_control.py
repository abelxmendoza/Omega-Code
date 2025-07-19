# File: /Omega-Code/servers/robot-controller-backend/controllers/lighting/led_control.py


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
        for j in range(256 * iterations):
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, self._wheel((i + j) & 255))
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def _wheel(self, pos):
        """
        Generates rainbow colors for a given position.

        Args:
            pos (int): Position in the color wheel (0-255).

        Returns:
            int: 24-bit RGB color value.
        """
        if pos < 0 or pos > 255:
            return Color(0, 0, 0)
        elif pos < 85:
            return Color(pos * 3, 255 - pos * 3, 0)
        elif pos < 170:
            pos -= 85
            return Color(255 - pos * 3, 0, pos * 3)
        else:
            pos -= 170
            return Color(0, pos * 3, 255 - pos * 3)

    def set_led(self, color, mode, pattern, interval):
        """
        Configures LED color and pattern.

        Args:
            color (int): 24-bit RGB color value.
            mode (str): Mode ('single', 'multi', etc.).
            pattern (str): Pattern type.
            interval (int): Delay for dynamic patterns.
        """
        try:
            if mode == "single":
                self.color_wipe(color)
            elif mode == "multi":
                self.theater_chase(color, wait_ms=interval)
            elif mode == "two":
                self.rainbow(wait_ms=interval)
            else:
                print(f"Invalid mode: {mode}")
        except Exception as e:
            print(f"Failed to set LED: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python3 led_control.py <color> <mode> <pattern> <interval>")
        sys.exit(1)

    try:
        color = int(sys.argv[1], 16)
        mode = sys.argv[2]
        pattern = sys.argv[3]
        interval = int(sys.argv[4])

        led_control = LedControl()
        led_control.set_led(color, mode, pattern, interval)
    except Exception as e:
        print(f"Error: {e}")

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
