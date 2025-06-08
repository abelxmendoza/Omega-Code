# File: /Omega-Code/servers/robot-controller-backend/controllers/lighting/led_control.py

"""
LED Control Script

This script provides control for an LED strip using the rpi_ws281x library.
It allows setting static colors, dynamic lighting patterns, and advanced effects.

Key functionalities:
1. Initialize and configure the LED strip.
2. Set colors, modes, patterns, and intervals for dynamic control.
3. Execute predefined effects with error handling for robust operation.
"""

import sys
import time

try:
    from rpi_ws281x import PixelStrip, Color
except Exception:  # pragma: no cover - handle missing library gracefully
    try:
        from rpi_ws281x import Adafruit_NeoPixel as PixelStrip, Color
    except Exception:
        # Provide a minimal stub for environments without rpi_ws281x
        class PixelStrip:
            def __init__(self, num, pin, freq_hz, dma, invert, brightness, channel):
                self._num = num

            def begin(self):
                pass

            def numPixels(self):
                return self._num

            def setPixelColor(self, i, color):
                pass

            def show(self):
                pass

        def Color(r, g, b):
            return (r << 16) | (g << 8) | b

# LED strip configuration constants
LED_COUNT = 8            # Number of LED pixels
LED_PIN = 18             # GPIO pin connected to the pixels (supports PWM)
LED_FREQ_HZ = 800000     # Signal frequency (800kHz for WS281x LEDs)
LED_DMA = 10             # DMA channel to use for signal generation
LED_BRIGHTNESS = 255     # Brightness level (0-255)
LED_INVERT = False       # Invert signal if using NPN transistor level shifter
LED_CHANNEL = 0          # Channel number (usually 0 or 1)

class LedControl:
    """
    Manages LED operations, including static and dynamic effects.
    """
    def __init__(self):
        """
        Initializes the LED strip with specified configuration.
        """
        try:
            self.ORDER = "RGB"  # Default color order
            self.strip = PixelStrip(
                LED_COUNT,
                LED_PIN,
                LED_FREQ_HZ,
                LED_DMA,
                LED_INVERT,
                LED_BRIGHTNESS,
                LED_CHANNEL,
            )
            self.strip.begin()  # Initialize the LED strip
        except Exception as e:
            raise RuntimeError(f"Failed to initialize LED strip: {e}")

    def _convert_color(self, order, R_G_B):
        """
        Converts RGB color values based on the configured order.

        Args:
            order (str): Color order ('RGB', 'GRB', etc.).
            R_G_B (int): 24-bit RGB color value.

        Returns:
            int: Adjusted color value for the LED strip.
        """
        try:
            B = R_G_B & 255
            G = (R_G_B >> 8) & 255
            R = (R_G_B >> 16) & 255
            order_map = ["GRB", "GBR", "RGB", "RBG", "BRG", "BGR"]
            color_map = [Color(G, R, B), Color(G, B, R), Color(R, G, B), Color(R, B, G), Color(B, R, G), Color(B, G, R)]
            if order in order_map:
                return color_map[order_map.index(order)]
            else:
                raise ValueError(f"Invalid order: {order}")
        except Exception as e:
            raise RuntimeError(f"Color conversion error: {e}")

    def _safe_execute(self, func, *args, **kwargs):
        """
        Safely executes a pattern function and handles any errors.

        Args:
            func (callable): The function to execute.
        """
        try:
            func(*args, **kwargs)
        except Exception as e:
            print(f"Error executing pattern: {e}")

    def color_wipe(self, color, wait_ms=50):
        """
        Fills the strip with a single color, one LED at a time.

        Args:
            color (int): 24-bit RGB color value.
            wait_ms (int): Delay between lighting each LED.
        """
        color = self._convert_color(self.ORDER, color)
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def theater_chase(self, color, wait_ms=50, iterations=10):
        """
        Creates a theater chase effect with the specified color.

        Args:
            color (int): 24-bit RGB color value.
            wait_ms (int): Delay between animation frames.
            iterations (int): Number of animation cycles.
        """
        color = self._convert_color(self.ORDER, color)
        for j in range(iterations):
            for q in range(3):
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i + q, color)
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i + q, 0)

    def rainbow(self, wait_ms=20, iterations=1):
        """
        Displays a rainbow effect across the LED strip.

        Args:
            wait_ms (int): Delay between animation frames.
            iterations (int): Number of animation cycles.
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
