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

import math
import time
from time import sleep
from typing import Sequence, Tuple

import numpy as np
from rpi_ws281x import Color

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

# --- Advanced patterns -----------------------------------------------------

def _clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp *value* to the inclusive range [minimum, maximum]."""

    return max(minimum, min(maximum, value))


def _mix_colors(color_a: Tuple[int, int, int], color_b: Tuple[int, int, int], ratio: float) -> Tuple[int, int, int]:
    """Blend two RGB tuples using *ratio* (0.0 → A, 1.0 → B)."""

    ratio = _clamp(ratio, 0.0, 1.0)
    return (
        int(color_a[0] * (1.0 - ratio) + color_b[0] * ratio),
        int(color_a[1] * (1.0 - ratio) + color_b[1] * ratio),
        int(color_a[2] * (1.0 - ratio) + color_b[2] * ratio),
    )


def _color_with_scale(color: Tuple[int, int, int], scale: float) -> Color:
    """Return an rpi_ws281x Color from an RGB tuple scaled by *scale*."""

    scale = _clamp(scale, 0.0, 1.0)
    r = int(_clamp(color[0] * scale, 0, 255))
    g = int(_clamp(color[1] * scale, 0, 255))
    b = int(_clamp(color[2] * scale, 0, 255))
    return Color(r, g, b)


def music_visualizer(
    strip,
    base_color: Sequence[int] = (255, 255, 255),
    brightness: float = 1.0,
    update_ms: int = 80,
    duration_s: float = 8.0,
    sample_rate: int = 44100,
) -> None:
    """Animate LEDs so they "dance" in response to music captured from the microphone.

    The function attempts to read short frames from the default input device using
    :mod:`sounddevice`. When the dependency or an input device is unavailable it
    gracefully falls back to a synthetic waveform so the pattern still produces a
    dynamic animation instead of failing outright.

    Args:
        strip: Initialised NeoPixel strip.
        base_color: RGB triple that defines the core hue for the animation.
        brightness: Global brightness multiplier ``0.0`` – ``1.0``.
        update_ms: Update cadence in milliseconds (controls responsiveness).
        duration_s: Total duration of the animation in seconds.
        sample_rate: Sample rate used when capturing audio.
    """

    try:  # Optional dependency; fall back to synthetic data when unavailable.
        import sounddevice as sd  # type: ignore
    except Exception:  # pragma: no cover - absence is fine on CI/containers
        sd = None  # type: ignore[assignment]

    brightness = _clamp(float(brightness), 0.0, 1.0)
    update_sec = max(update_ms / 1000.0, 0.05)
    duration = max(duration_s, update_sec)

    num_pixels = getattr(strip, "numPixels", lambda: 0)()
    if num_pixels <= 0:
        return

    base = tuple(int(_clamp(c, 0, 255)) for c in base_color)
    complement = tuple(255 - c for c in base)
    background = tuple(int(c * 0.05) for c in base)

    palette = [_mix_colors(base, complement, i / max(num_pixels - 1, 1)) for i in range(num_pixels)]

    smoothing = 0.65
    level = 0.0
    fallback_phase = 0.0
    offset = 0

    def capture_level() -> float:
        nonlocal sd

        if sd is None:
            return -1.0

        frames = max(int(sample_rate * update_sec), 256)
        try:
            audio = sd.rec(frames, samplerate=sample_rate, channels=1, dtype="float32")
            sd.wait()
        except Exception as exc:  # pragma: no cover - depends on runtime hardware
            print(f"Music visualizer audio capture unavailable ({exc}); using synthetic waveform.")
            sd = None  # type: ignore[assignment]
            return -1.0

        if not len(audio):
            return 0.0

        value = float(np.sqrt(np.mean(np.square(audio))))
        if math.isnan(value):
            value = 0.0
        return _clamp(value * 8.0, 0.0, 1.0)

    end_time = time.time() + duration
    while time.time() < end_time:
        amplitude = capture_level()
        if amplitude < 0:
            fallback_phase += update_sec * 2.0
            amplitude = (math.sin(fallback_phase) + 1.0) / 2.0

        level = smoothing * level + (1.0 - smoothing) * amplitude
        active_pixels = max(1, int(level * num_pixels))
        trail = max(1, int(num_pixels * 0.15))

        for idx in range(num_pixels):
            if idx < active_pixels:
                color = palette[(idx + offset) % num_pixels]
                strip.setPixelColor(idx, _color_with_scale(color, brightness))
            elif idx < active_pixels + trail:
                fade = 1.0 - ((idx - active_pixels) / max(trail, 1))
                strip.setPixelColor(idx, _color_with_scale(base, brightness * 0.35 * fade))
            else:
                strip.setPixelColor(idx, _color_with_scale(background, brightness))

        strip.show()
        offset = (offset + 1) % num_pixels
        time.sleep(update_sec)

# Lighting patterns for NeoPixels
