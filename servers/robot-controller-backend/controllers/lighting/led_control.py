# File: /Omega-Code/servers/robot-controller-backend/controllers/lighting/led_control.py

"""
LED Control Module for WS2812/WS2811 Strips (NeoPixels)

This module provides a Python class for controlling addressable RGB LED strips
using the rpi_ws281x library. It supports color wipe animations, color patterns,
on/off state, and can be extended for dual-color and brightness control.

Main Features:
- Initialize and configure the LED strip
- Perform color wipe and basic animations
- On/Off toggle functionality with persistent state
- Prepares for future dual color + brightness enhancements

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/led_control.py
"""

import sys
import time
from rpi_ws281x import Adafruit_NeoPixel, Color, WS2811_STRIP_GRB

class LedController:
    def __init__(self, num_pixels=16, pin=18, brightness=255):
        self.strip = Adafruit_NeoPixel(
            num_pixels,
            pin,
            800000,
            10,
            False,
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

    def rainbow(self, wait_ms=20):
        for j in range(256):
            for i in range(self.num_pixels):
                self.strip.setPixelColor(i, self._wheel((i + j) & 255))
            self.strip.show()
            time.sleep(wait_ms / 1000.0)
        self.is_on = True

    def set_led(self, color, mode="single", pattern="static", interval=500):
        try:
            if pattern == "off":
                self.clear_strip()
                return

            if not self.is_on:
                print("LEDs are OFF. Turn them ON to apply effects.")
                return

            r = (color >> 16) & 255
            g = (color >> 8) & 255
            b = color & 255

            if mode == "rainbow":
                self.rainbow(interval)
            elif pattern == "static":
                self.color_wipe(Color(r, g, b), wait_ms=10)
            elif pattern == "blink":
                for _ in range(5):
                    self.color_wipe(Color(r, g, b), wait_ms=10)
                    time.sleep(interval / 1000)
                    self.clear_strip()
                    time.sleep(interval / 1000)
            elif pattern == "pulse":
                for _ in range(5):
                    for i in range(0, 256, 5):
                        brightness = i / 255.0
                        self._apply_brightness(color, brightness)
                        time.sleep(interval / 1000 / 50)
                    for i in range(255, 0, -5):
                        brightness = i / 255.0
                        self._apply_brightness(color, brightness)
                        time.sleep(interval / 1000 / 50)
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
    if len(sys.argv) == 2 and sys.argv[1] == "toggle":
        led = LedController()
        led.toggle_light()
        sys.exit(0)

    if len(sys.argv) != 5:
        print("Usage: python3 led_control.py <hexcolor> <mode> <pattern> <interval>")
        print("Or: python3 led_control.py toggle")
        sys.exit(1)

    try:
        color = int(sys.argv[1], 16)
        mode = sys.argv[2]
        pattern = sys.argv[3]
        interval = int(sys.argv[4])

        led_control = LedController()
        led_control.set_led(color, mode, pattern, interval)
    except Exception as e:
        print(f"Startup error: {e}")

