"""
Lighting Patterns Module for WS2812/WS2811 LED Strips

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/patterns.py

This module contains pattern functions for LED effects using the rpi_ws281x library.
It supports basic patterns like static color wipe, dual color alternation, fade, blink,
chase, and rainbow effects.

Usage:
- All functions assume the caller passes a pre-initialized NeoPixel strip object as the first argument.
"""

from time import sleep
from rpi_ws281x import Color


def _scale_rgb(rgb, scale, brightness):
    """Scale an (r,g,b) tuple by a factor and brightness clamp to 0..255."""
    r, g, b = rgb
    return (
        max(0, min(255, int(r * scale * brightness))),
        max(0, min(255, int(g * scale * brightness))),
        max(0, min(255, int(b * scale * brightness))),
    )

def color_wipe(strip, color):
    """
    Fill all LEDs with a single solid color.
    Args:
        strip: The initialized NeoPixel strip object.
        color: The Color object for all LEDs.
    """
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
    strip.show()

def dual_color(strip, color1, color2=Color(0,0,0)):
    """
    Alternate between two colors along the strip.
    Args:
        strip: The initialized NeoPixel strip object.
        color1: Color object for even LEDs.
        color2: Color object for odd LEDs.
    """
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color1 if i % 2 == 0 else color2)
    strip.show()

def fade(strip, color1, color2=None, steps=50, delay=0.01):
    """
    Fade from black to color1, or from color1 to color2 if provided.
    Args:
        strip: The initialized NeoPixel strip object.
        color1: Tuple (r,g,b) as the first color.
        color2: Tuple (r,g,b) as the second color (optional).
        steps: Number of fade steps.
        delay: Time delay between steps.
    """
    def lerp(a, b, t):
        return int(a + (b - a) * t)
    for step in range(steps):
        t = step / steps
        if color2:
            # Fade between color1 and color2
            r = lerp(color1[0], color2[0], t)
            g = lerp(color1[1], color2[1], t)
            b = lerp(color1[2], color2[2], t)
        else:
            # Fade from black to color1
            r = lerp(0, color1[0], t)
            g = lerp(0, color1[1], t)
            b = lerp(0, color1[2], t)
        c = Color(r, g, b)
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, c)
        strip.show()
        sleep(delay)

def blink(strip, color1, color2=Color(0,0,0), delay=0.5):
    """
    Blink all LEDs with color1, then color2 (or off if color2 not given).
    Args:
        strip: The initialized NeoPixel strip object.
        color1: First Color object.
        color2: Second Color object (or off).
        delay: Time for each color.
    """
    for _ in range(10):
        color_wipe(strip, color1)
        sleep(delay)
        color_wipe(strip, color2)
        sleep(delay)

def chase(strip, color1, color2=Color(0,0,0), delay=0.05):
    """
    Create a moving chase effect with optional secondary background color.
    Args:
        strip: The initialized NeoPixel strip object.
        color1: Color object for chasing pixel.
        color2: Background Color object.
        delay: Time for each move.
    """
    for i in range(strip.numPixels()):
        for j in range(strip.numPixels()):
            if j == i:
                strip.setPixelColor(j, color1)
            else:
                strip.setPixelColor(j, color2)
        strip.show()
        sleep(delay)

def rainbow(strip, wait_ms=20, iterations=1):
    """
    Display a rainbow across the strip.
    Args:
        strip: The initialized NeoPixel strip object.
        wait_ms: Delay between color changes.
        iterations: Number of rainbow cycles.
    """
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

def lightshow(strip, base_rgb, interval_ms=100, brightness=1.0, cycles=3):
    """Create a multi-stage lightshow using the provided base color.

    The effect blends rotating color bands derived from ``base_rgb`` with
    white sparkles to create a lively showcase without requiring random
    numbers (making it deterministic for testing).

    Args:
        strip: NeoPixel strip instance.
        base_rgb: Tuple of the primary color from the UI payload.
        interval_ms: Base interval from the payload in milliseconds.
        brightness: Float multiplier (0–1) applied to all palette colors.
        cycles: Number of rotations through the effect.
    """

    if not isinstance(base_rgb, tuple) or len(base_rgb) != 3:
        raise ValueError("base_rgb must be an (r, g, b) tuple")

    # Build a palette that mixes the requested color with softer accents.
    palette = [
        Color(*_scale_rgb(base_rgb, 1.0, brightness)),
        Color(*_scale_rgb(base_rgb, 0.6, brightness)),
        Color(*_scale_rgb(base_rgb, 0.25, brightness)),
        Color(*_scale_rgb((255, 255, 255), 0.5, brightness)),
    ]

    sparkle = Color(*_scale_rgb((255, 255, 255), 1.0, brightness))
    delay = max(interval_ms / 1000.0, 0.02)
    sparkle_delay = max(delay / 2.0, 0.01)
    num_pixels = strip.numPixels()
    total_cycles = max(1, cycles)

    for cycle in range(total_cycles):
        # Rotating bands derived from the palette
        for offset in range(num_pixels):
            for i in range(num_pixels):
                strip.setPixelColor(i, palette[(i + offset + cycle) % len(palette)])
            strip.show()
            sleep(delay)

        # Sparkle sweep to add extra movement/highlights
        for offset in range(num_pixels):
            for i in range(num_pixels):
                strip.setPixelColor(i, palette[(i + offset + cycle) % len(palette)])
            strip.setPixelColor((offset * 2) % num_pixels, sparkle)
            strip.show()
            sleep(sparkle_delay)

# Lighting patterns for NeoPixels
