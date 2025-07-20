"""
Lighting Patterns Module for WS2812/WS2811 LED Strips

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/patterns.py

This module contains pattern functions for LED effects using the rpi_ws281x library.
It supports basic patterns like static color wipe, dual color alternation, fade, blink,
chase, and rainbow effects.

Functions assume the caller passes a pre-initialized NeoPixel strip object.
"""

from time import sleep
from rpi_ws281x import Color


def color_wipe(strip, color):
    """Fill all LEDs with a single solid color."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
    strip.show()


def dual_color(strip, color1, color2):
    """Alternate between two colors along the strip."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color1 if i % 2 == 0 else color2)
    strip.show()


def fade(strip, color1, color2=None, steps=50, delay=0.01):
    """
    Fade from black to color1, or from color1 to color2 if provided.
    Uses linear interpolation.
    """
    def lerp(a, b, t):
        return int(a + (b - a) * t)

    for step in range(steps):
        t = step / steps
        r = lerp(0, color1[0], t)
        g = lerp(0, color1[1], t)
        b = lerp(0, color1[2], t)
        c = Color(r, g, b)
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, c)
        strip.show()
        sleep(delay)


def blink(strip, color1, color2=None, delay=0.5):
    """Blink all LEDs with color1, or alternate with color2 if given."""
    for _ in range(10):
        color_wipe(strip, color1)
        sleep(delay)
        color_wipe(strip, color2 if color2 else Color(0, 0, 0))
        sleep(delay)


def chase(strip, color1, color2=None, delay=0.05):
    """Create a moving chase effect with optional secondary background color."""
    for i in range(strip.numPixels()):
        for j in range(strip.numPixels()):
            if j == i:
                strip.setPixelColor(j, color1)
            else:
                strip.setPixelColor(j, color2 if color2 else Color(0, 0, 0))
        strip.show()
        sleep(delay)


def rainbow(strip, wait_ms=20, iterations=1):
    """Display a rainbow across the strip."""
    def wheel(pos):
        if pos < 85:
            return Color(pos * 3, 255 - pos * 3, 0)
        elif pos < 170:
            pos -= 85
            return Color(255 - pos * 3, 0, pos * 3)
        else:
            pos -= 170
            return Color(0, pos * 3, 255 - pos * 3)

    for j in range(256 * iterations):
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, wheel((i + j) & 255))
        strip.show()
        sleep(wait_ms / 1000.0)
# Lighting patterns for NeoPixels

