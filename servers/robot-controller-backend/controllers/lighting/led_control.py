"""
LED Control Module for WS2812/WS2811 Strips (NeoPixels)

This module provides a Python class for controlling addressable RGB LED strips
using the rpi_ws281x library. It supports color wipe animations, color patterns,
on/off state, and can be extended for dual-color and brightness control.

Main Features:
- Initialize and configure the LED strip
- Perform color wipe and basic animations
- On/Off toggle functionality with persistent state
- Full global brightness support for all patterns

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/led_control.py
"""

import sys
import time
from rpi_ws281x import Adafruit_NeoPixel, Color, WS2811_STRIP_GRB

class LedController:
    """
    Controller class for WS2812/WS2811 LED strips using rpi_ws281x.
    """
    def __init__(self):
        """
        Initializes the LED strip with specified configuration.
        """
        self.ORDER = "RGB"  # Default color order
        try:
          self.strip = PixelStrip(
            LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA,
             LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL
              )


                LED_COUNT,
                LED_PIN,
                LED_FREQ_HZ,
                LED_DMA,
                LED_INVERT,
                LED_BRIGHTNESS,
                LED_CHANNEL,
            )
            strip.begin()  # Initialize the LED strip
            self.strip = strip
        except Exception as e:
            print(f"Warning: Failed to initialize LED strip: {e}")

            try:
                del strip
            except UnboundLocalError:
                pass

            self.strip = StubPixelStrip(
                LED_COUNT,
                LED_PIN,
                LED_FREQ_HZ,
                LED_DMA,
                LED_INVERT,
                LED_BRIGHTNESS,
                LED_CHANNEL,
            )
            print("Using stub LED strip; no hardware output will occur")

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
    def __init__(self, num_pixels=16, pin=18, brightness=255):
  master
        """
        Initialize the LED strip.

        Args:
            num_pixels (int): Number of LEDs.
            pin (int): GPIO pin (default: 18).
            brightness (int): Brightness (0–255).
        """
        self.strip = Adafruit_NeoPixel(
            num_pixels,
            pin,
            800000,      # Frequency (Hz)
            10,          # DMA channel
            False,       # Invert signal
            brightness,
            0,
            WS2811_STRIP_GRB
        )
        self.strip.begin()
        self.num_pixels = num_pixels
        self.is_on = False

    def color_wipe(self, color, wait_ms=50):
        for i in range(self.num_pixels):
            self.strip.setPixelColor(i, color)
        self.strip.show()
        self.is_on = True

    def clear_strip(self):
        for i in range(self.num_pixels):
            self.strip.setPixelColor(i, Color(0, 0, 0))
        self.strip.show()
        self.is_on = False

    def test_colors(self, wait_ms=500):
        self.color_wipe(Color(255, 0, 0), wait_ms)
        time.sleep(1)
        self.color_wipe(Color(0, 255, 0), wait_ms)
        time.sleep(1)
        self.color_wipe(Color(0, 0, 255), wait_ms)
        time.sleep(1)
        self.clear_strip()

    def _wheel(self, pos):
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

    def _apply_brightness(self, base_color, brightness):
        r = int(((base_color >> 16) & 255) * brightness)
        g = int(((base_color >> 8) & 255) * brightness)
        b = int((base_color & 255) * brightness)
        for i in range(self.num_pixels):
            self.strip.setPixelColor(i, Color(r, g, b))
        self.strip.show()

    def rainbow(self, wait_ms=20, brightness=1.0):
        for j in range(256):
            for i in range(self.num_pixels):
                base_color = self._wheel((i + j) & 255)
                r = int(((base_color >> 16) & 255) * brightness)
                g = int(((base_color >> 8) & 255) * brightness)
                b = int((base_color & 255) * brightness)
                self.strip.setPixelColor(i, Color(r, g, b))
            self.strip.show()
            time.sleep(wait_ms / 1000.0)
        self.is_on = True

    def set_led(self, color, mode="single", pattern="static", interval=500, brightness=1.0):
        """
        Set LED strip color/pattern/mode/brightness.

        Args:
            color (int): 24-bit RGB integer.
            mode (str): Lighting mode.
            pattern (str): Animation pattern.
            interval (int): Timing for dynamic patterns (ms).
            brightness (float): 0.0–1.0, global brightness.
        """
        try:
            if pattern == "off":
                self.clear_strip()
                return

            # Always apply requested pattern!
            r = int(((color >> 16) & 255) * brightness)
            g = int(((color >> 8) & 255) * brightness)
            b = int((color & 255) * brightness)

            if mode == "rainbow" or pattern == "rainbow":
                self.rainbow(interval, brightness)
            elif pattern == "static":
                self.color_wipe(Color(r, g, b), wait_ms=10)
            elif pattern == "blink":
                for _ in range(5):
                    self.color_wipe(Color(r, g, b), wait_ms=10)
                    time.sleep(interval / 1000)
                    self.clear_strip()
                    time.sleep(interval / 1000)
                self.is_on = True
            elif pattern == "pulse":
                for _ in range(5):
                    # Fade in
                    for i in range(0, 256, 5):
                        step_brightness = brightness * (i / 255.0)
                        self._apply_brightness(color, step_brightness)
                        time.sleep(interval / 1000 / 50)
                    # Fade out
                    for i in range(255, 0, -5):
                        step_brightness = brightness * (i / 255.0)
                        self._apply_brightness(color, step_brightness)
                        time.sleep(interval / 1000 / 50)
                self.is_on = True
            else:
                print(f"Unknown pattern: {pattern}")

            self.is_on = True

        except Exception as e:
            print(f"LED error: {e}")

    def toggle_light(self):
        if self.is_on:
            self.clear_strip()
            print("LEDs turned OFF")
        else:
            self.color_wipe(Color(255, 255, 255), wait_ms=20)
            print("LEDs turned ON")

    def get_status(self):
        return "ON" if self.is_on else "OFF"


if __name__ == "__main__":
    # CLI usage: python3 led_control.py <hexcolor> <mode> <pattern> <interval> <brightness>
    #           python3 led_control.py off
    #           python3 led_control.py toggle
    if len(sys.argv) == 2 and sys.argv[1] == "off":
        led = LedController()
        led.clear_strip()
        print("LEDs turned OFF")
        sys.exit(0)

    if len(sys.argv) == 2 and sys.argv[1] == "toggle":
        led = LedController()
        led.toggle_light()
        sys.exit(0)

    if len(sys.argv) not in (5, 6):
        print("Usage: python3 led_control.py <hexcolor> <mode> <pattern> <interval> <brightness>")
        print("   or: python3 led_control.py off")
        print("   or: python3 led_control.py toggle")
        sys.exit(1)

    try:
        color = int(sys.argv[1], 16)
        mode = sys.argv[2]
        pattern = sys.argv[3]
        interval = int(sys.argv[4])
        brightness = float(sys.argv[5]) if len(sys.argv) == 6 else 1.0

        led_control = LedController()
        led_control.set_led(color, mode, pattern, interval, brightness)
    except Exception as e:
        print(f"Startup error: {e}")
