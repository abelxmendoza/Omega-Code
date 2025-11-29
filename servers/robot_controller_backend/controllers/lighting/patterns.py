"""
Lighting Patterns Module for WS2812/WS2811 LED Strips

File Location:
~/Omega-Code/servers/robot_controller_backend/controllers/lighting/patterns.py

This module contains pattern functions for LED effects using the rpi_ws281x library.
It supports basic patterns like static color wipe, dual color alternation, fade, blink,
chase, and rainbow effects.

Usage:
- All functions assume the caller passes a pre-initialized NeoPixel strip object as the first argument.
"""

import math
import time
import sys
from time import sleep
from typing import Sequence, Tuple
from functools import lru_cache

import numpy as np
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
    Optimized color wipe - fill all LEDs with a single solid color.
    Args:
        strip: The initialized NeoPixel strip object.
        color: The Color object for all LEDs.
    """
    try:
        num_pixels = strip.numPixels()
        for i in range(num_pixels):
            strip.setPixelColor(i, color)
        strip.show()
    except Exception as e:
        print(f"❌ [ERROR] color_wipe failed: {e}", file=sys.stderr)
        raise

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
    Optimized chase effect - create a moving chase with optional background color.
    Args:
        strip: The initialized NeoPixel strip object.
        color1: Color object for chasing pixel.
        color2: Background Color object.
        delay: Time for each move.
    """
    try:
        num_pixels = strip.numPixels()
        # Pre-fill with background color, then update chasing pixel
        for i in range(num_pixels):
            # Fill all with background first
            for j in range(num_pixels):
                strip.setPixelColor(j, color2)
            # Set chasing pixel
            strip.setPixelColor(i, color1)
            strip.show()
            sleep(delay)
    except Exception as e:
        print(f"❌ [ERROR] chase failed: {e}", file=sys.stderr)
        raise

def rainbow(strip, wait_ms=20, iterations=1):
    """
    Optimized rainbow - display a rainbow across the strip with cached calculations.
    Args:
        strip: The initialized NeoPixel strip object.
        wait_ms: Delay between color changes.
        iterations: Number of rainbow cycles.
    """
    @lru_cache(maxsize=256)
    def wheel(pos):
        """Cached wheel function for rainbow colors."""
        pos = pos & 255  # Ensure 0-255 range
        if pos < 85:
            return Color(pos * 3, 255 - pos * 3, 0)
        elif pos < 170:
            pos -= 85
            return Color(255 - pos * 3, 0, pos * 3)
        else:
            pos -= 170
            return Color(0, pos * 3, 255 - pos * 3)
    
    try:
        num_pixels = strip.numPixels()
        wait_sec = max(0.01, wait_ms / 1000.0)  # Pre-compute wait time
        
        for j in range(256 * iterations):
            for i in range(num_pixels):
                strip.setPixelColor(i, wheel((i + j) & 255))
            strip.show()
            sleep(wait_sec)
    except Exception as e:
        print(f"❌ [ERROR] rainbow failed: {e}", file=sys.stderr)
        raise


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


def rave_mode(
    strip,
    base_rgb: Tuple[int, int, int] = (255, 0, 255),
    brightness: float = 1.0,
    interval_ms: int = 50,
    duration_s: float = 30.0,
) -> None:
    """Create an energetic "rave mode" dancing lights effect.
    
    This pattern creates a high-energy, party-like lighting effect with:
    - Fast color cycling through vibrant colors
    - Strobing effects
    - Wave patterns
    - Pulsing beats
    - Multiple simultaneous effects
    
    No audio input required - creates synthetic "beat" patterns that simulate
    dancing to music. Perfect for party mode!
    
    Args:
        strip: Initialized NeoPixel strip.
        base_rgb: Base color tuple (r, g, b) - influences color palette.
        brightness: Global brightness multiplier (0.0-1.0).
        interval_ms: Base update interval in milliseconds (lower = faster, recommended 30-100ms).
        duration_s: Duration to run the effect in seconds.
    """
    
    brightness = _clamp(float(brightness), 0.0, 1.0)
    update_sec = max(0.01, interval_ms / 1000.0)
    duration = max(duration_s, update_sec)
    num_pixels = strip.numPixels()
    
    if num_pixels <= 0:
        return
    
    # Create vibrant rave color palette
    # Mix base color with classic rave colors (magenta, cyan, yellow, green, red, blue)
    rave_colors = [
        base_rgb,
        (255, 0, 255),  # Magenta
        (0, 255, 255),  # Cyan
        (255, 255, 0),  # Yellow
        (0, 255, 0),    # Green
        (255, 0, 0),    # Red
        (0, 0, 255),    # Blue
        (255, 128, 0),  # Orange
    ]
    
    # Scale colors by brightness
    palette = [tuple(int(c * brightness) for c in color) for color in rave_colors]
    palette_colors = [Color(*rgb) for rgb in palette]
    
    # State variables for various effects
    phase = 0.0
    wave_offset = 0
    strobe_counter = 0
    color_index = 0
    
    end_time = time.time() + duration
    
    while time.time() < end_time:
        phase += update_sec * 4.0  # Fast phase progression
        strobe_counter += 1
        
        # Multiple simultaneous effects
        for i in range(num_pixels):
            # Effect 1: Wave pattern (sine wave moving across strip)
            wave_pos = (i + wave_offset) % num_pixels
            wave_value = (math.sin(phase + wave_pos * 0.5) + 1.0) / 2.0
            
            # Effect 2: Strobing (every 3rd frame, flash white)
            if strobe_counter % 3 == 0 and i % 3 == 0:
                strip.setPixelColor(i, Color(*_scale_rgb((255, 255, 255), 1.0, brightness)))
            else:
                # Effect 3: Color cycling with wave modulation
                color_idx = int((phase * 2 + i * 0.3) % len(palette_colors))
                base_color = palette[color_idx]
                
                # Apply wave modulation to brightness
                modulated_brightness = wave_value * 0.7 + 0.3  # 30-100% brightness
                r = int(base_color[0] * modulated_brightness)
                g = int(base_color[1] * modulated_brightness)
                b = int(base_color[2] * modulated_brightness)
                
                strip.setPixelColor(i, Color(r, g, b))
        
        strip.show()
        
        # Update offsets for next frame
        wave_offset = (wave_offset + 1) % num_pixels
        color_index = (color_index + 1) % len(palette_colors)
        
        time.sleep(update_sec)


def breathing(
    strip,
    base_rgb: Tuple[int, int, int] = (0, 255, 255),
    brightness: float = 1.0,
    interval_ms: int = 50,
    cycles: int = 5,
) -> None:
    """Smooth breathing effect - like a gentle pulse.
    
    Creates a calming, breathing-like effect perfect for idle/standby mode.
    Energy-efficient with smooth transitions.
    
    Args:
        strip: Initialized NeoPixel strip.
        base_rgb: Base color tuple (r, g, b).
        brightness: Global brightness multiplier (0.0-1.0).
        interval_ms: Update interval in milliseconds.
        cycles: Number of breath cycles.
    """
    brightness = _clamp(float(brightness), 0.0, 1.0)
    update_sec = max(0.01, interval_ms / 1000.0)
    num_pixels = strip.numPixels()
    
    if num_pixels <= 0:
        return
    
    base_color = tuple(int(c * brightness) for c in base_rgb)
    
    for cycle in range(cycles):
        # Breathe in (fade up)
        for step in range(0, 101, 2):
            scale = step / 100.0
            # Smooth curve using ease-in-out
            eased = scale * scale * (3.0 - 2.0 * scale)
            r = int(base_color[0] * eased)
            g = int(base_color[1] * eased)
            b = int(base_color[2] * eased)
            color = Color(r, g, b)
            for i in range(num_pixels):
                strip.setPixelColor(i, color)
            strip.show()
            time.sleep(update_sec * 2)
        
        # Breathe out (fade down)
        for step in range(100, -1, -2):
            scale = step / 100.0
            eased = scale * scale * (3.0 - 2.0 * scale)
            r = int(base_color[0] * eased)
            g = int(base_color[1] * eased)
            b = int(base_color[2] * eased)
            color = Color(r, g, b)
            for i in range(num_pixels):
                strip.setPixelColor(i, color)
            strip.show()
            time.sleep(update_sec * 2)


def aurora(
    strip,
    base_rgb: Tuple[int, int, int] = (0, 150, 255),
    brightness: float = 1.0,
    interval_ms: int = 80,
    duration_s: float = 20.0,
) -> None:
    """Aurora effect - flowing, wave-like colors like northern lights.
    
    Creates mesmerizing, flowing color waves perfect for ambient lighting.
    Energy-efficient with smooth, organic movements.
    
    Args:
        strip: Initialized NeoPixel strip.
        base_rgb: Base color tuple (r, g, b) - typically cool colors.
        brightness: Global brightness multiplier (0.0-1.0).
        interval_ms: Update interval in milliseconds.
        duration_s: Duration to run the effect.
    """
    brightness = _clamp(float(brightness), 0.0, 1.0)
    update_sec = max(0.02, interval_ms / 1000.0)
    duration = max(duration_s, update_sec)
    num_pixels = strip.numPixels()
    
    if num_pixels <= 0:
        return
    
    # Create aurora color palette (cool colors)
    aurora_colors = [
        base_rgb,
        (0, 100, 200),   # Deep blue
        (50, 200, 255),  # Cyan
        (100, 150, 255), # Light blue
        (150, 100, 255), # Purple-blue
    ]
    
    palette = [tuple(int(c * brightness) for c in color) for color in aurora_colors]
    
    phase = 0.0
    end_time = time.time() + duration
    
    while time.time() < end_time:
        phase += update_sec * 0.5  # Slow, flowing movement
        
        for i in range(num_pixels):
            # Multiple sine waves for organic flow
            wave1 = math.sin(phase + i * 0.1) * 0.5 + 0.5
            wave2 = math.sin(phase * 1.3 + i * 0.15) * 0.5 + 0.5
            wave3 = math.cos(phase * 0.7 + i * 0.2) * 0.5 + 0.5
            
            # Combine waves for complex pattern
            combined = (wave1 + wave2 + wave3) / 3.0
            
            # Select color based on position and phase
            color_idx = int((i * 0.1 + phase * 0.5) % len(palette))
            base_color = palette[color_idx]
            
            # Apply wave modulation
            r = int(base_color[0] * combined)
            g = int(base_color[1] * combined)
            b = int(base_color[2] * combined)
            
            strip.setPixelColor(i, Color(r, g, b))
        
        strip.show()
        time.sleep(update_sec)


def matrix_rain(
    strip,
    base_rgb: Tuple[int, int, int] = (0, 255, 0),
    brightness: float = 1.0,
    interval_ms: int = 100,
    duration_s: float = 15.0,
) -> None:
    """Matrix-style rain effect - falling columns of light.
    
    Creates a cool "Matrix" style effect with falling light trails.
    Perfect for tech/cyber aesthetic.
    
    Args:
        strip: Initialized NeoPixel strip.
        base_rgb: Base color tuple (r, g, b) - typically green for Matrix look.
        brightness: Global brightness multiplier (0.0-1.0).
        interval_ms: Update interval in milliseconds (lower = faster rain).
        duration_s: Duration to run the effect.
    """
    brightness = _clamp(float(brightness), 0.0, 1.0)
    update_sec = max(0.05, interval_ms / 1000.0)
    duration = max(duration_s, update_sec)
    num_pixels = strip.numPixels()
    
    if num_pixels <= 0:
        return
    
    # Create matrix color with brightness
    matrix_color = tuple(int(c * brightness) for c in base_rgb)
    trail_color = tuple(int(c * brightness * 0.3) for c in base_rgb)
    
    # Track falling drops
    drops = []
    drop_counter = 0
    
    end_time = time.time() + duration
    
    while time.time() < end_time:
        # Clear strip
        for i in range(num_pixels):
            strip.setPixelColor(i, Color(0, 0, 0))
        
        # Add new drops occasionally
        drop_counter += 1
        if drop_counter % 3 == 0:
            drops.append({"pos": 0, "speed": 1 + (drop_counter % 3)})
        
        # Update and draw drops
        active_drops = []
        for drop in drops:
            pos = drop["pos"]
            speed = drop["speed"]
            
            # Draw head (bright)
            if 0 <= pos < num_pixels:
                strip.setPixelColor(int(pos), Color(*matrix_color))
            
            # Draw trail (fading)
            trail_length = 3
            for t in range(1, trail_length + 1):
                trail_pos = pos - t * speed
                if 0 <= trail_pos < num_pixels:
                    fade = 1.0 - (t / trail_length)
                    r = int(trail_color[0] * fade)
                    g = int(trail_color[1] * fade)
                    b = int(trail_color[2] * fade)
                    strip.setPixelColor(int(trail_pos), Color(r, g, b))
            
            # Update position
            drop["pos"] += speed
            
            # Keep if still visible
            if drop["pos"] < num_pixels + trail_length * speed:
                active_drops.append(drop)
        
        drops = active_drops
        strip.show()
        time.sleep(update_sec)


def fire_effect(
    strip,
    brightness: float = 1.0,
    interval_ms: int = 50,
    duration_s: float = 20.0,
) -> None:
    """Fire effect - flickering flames with orange/red/yellow colors.
    
    Creates a realistic fire effect perfect for ambient lighting.
    Uses cooling algorithm for natural flame behavior.
    
    Args:
        strip: Initialized NeoPixel strip.
        brightness: Global brightness multiplier (0.0-1.0).
        interval_ms: Update interval in milliseconds.
        duration_s: Duration to run the effect.
    """
    brightness = _clamp(float(brightness), 0.0, 1.0)
    update_sec = max(0.02, interval_ms / 1000.0)
    duration = max(duration_s, update_sec)
    num_pixels = strip.numPixels()
    
    if num_pixels <= 0:
        return
    
    # Fire heat array (0-255)
    heat = [0] * num_pixels
    
    end_time = time.time() + duration
    
    while time.time() < end_time:
        # Cool down every cell a little
        for i in range(num_pixels):
            cooling = (i * 7) / num_pixels + 2
            heat[i] = max(0, heat[i] - cooling)
        
        # Heat from each cell drifts up and diffuses
        for i in range(num_pixels - 1, 2, -1):
            heat[i] = (heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3
        
        # Randomly ignite new 'sparks' at the bottom
        if time.time() % 0.1 < update_sec:
            spark = int(num_pixels * 0.8 + (num_pixels * 0.2 * (time.time() % 1.0)))
            if 0 <= spark < num_pixels:
                heat[spark] = min(255, heat[spark] + 160)
        
        # Convert heat to LED colors
        for i in range(num_pixels):
            # Scale heat value to color
            t192 = int((heat[i] / 255.0) * 191)
            
            # Calculate brightness of this frame
            heatramp = t192 & 0x3F  # 0-63
            heatramp <<= 2  # Scale up to 0-252
            
            # Color based on heat
            if t192 > 0x80:  # Hottest
                r = 255
                g = 255
                b = heatramp
            elif t192 > 0x40:  # Hot
                r = 255
                g = heatramp
                b = 0
            else:  # Cool
                r = heatramp
                g = 0
                b = 0
            
            # Apply brightness
            r = int(r * brightness)
            g = int(g * brightness)
            b = int(b * brightness)
            
            strip.setPixelColor(i, Color(r, g, b))
        
        strip.show()
        time.sleep(update_sec)


def status_indicator(
    strip,
    status: str = "idle",
    brightness: float = 0.5,
) -> None:
    """Status indicator - shows robot state through lighting.
    
    Uses different colors/patterns to indicate robot status:
    - idle: Soft blue breathing
    - moving: Green pulse
    - error: Red blink
    - low_battery: Orange warning
    - charging: Yellow pulse
    
    Args:
        strip: Initialized NeoPixel strip.
        status: Robot status string.
        brightness: Global brightness multiplier (0.0-1.0).
    """
    brightness = _clamp(float(brightness), 0.0, 1.0)
    num_pixels = strip.numPixels()
    
    if num_pixels <= 0:
        return
    
    status_colors = {
        "idle": (0, 100, 255),      # Soft blue
        "moving": (0, 255, 0),      # Green
        "error": (255, 0, 0),       # Red
        "low_battery": (255, 128, 0),  # Orange
        "charging": (255, 255, 0),  # Yellow
        "ready": (0, 255, 255),     # Cyan
    }
    
    color = status_colors.get(status.lower(), (128, 128, 128))
    color = tuple(int(c * brightness) for c in color)
    
    if status.lower() == "idle":
        # Breathing effect for idle
        breathing(strip, color, brightness, 50, 1)
    elif status.lower() == "error":
        # Blink for errors
        blink(strip, Color(*color), Color(0, 0, 0), delay=0.3)
    elif status.lower() == "moving":
        # Pulse for movement
        for _ in range(3):
            color_wipe(strip, Color(*color))
            time.sleep(0.2)
            color_wipe(strip, Color(0, 0, 0))
            time.sleep(0.2)
    else:
        # Static color for other statuses
        color_wipe(strip, Color(*color))


# Lighting patterns for NeoPixels
