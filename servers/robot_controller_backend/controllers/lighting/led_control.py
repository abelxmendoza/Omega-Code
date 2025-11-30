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
~/Omega-Code/servers/robot_controller_backend/controllers/lighting/led_control.py
"""

import sys
import os
import time
import traceback
from functools import lru_cache

# Add parent directory to Python path if not already set
# This allows the script to be run from any directory
if 'PYTHONPATH' not in os.environ or 'robot_controller_backend' not in os.environ.get('PYTHONPATH', ''):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.abspath(os.path.join(script_dir, '../..'))
    if parent_dir not in sys.path:
        sys.path.insert(0, parent_dir)

from rpi_ws281x import Adafruit_NeoPixel, Color, WS2811_STRIP_GRB

from controllers.lighting.patterns import (
    music_visualizer,
    fade,
    chase,
    blink,
    color_wipe as pattern_color_wipe,
    dual_color,
    rave_mode,
    breathing,
    aurora,
    matrix_rain,
    fire_effect,
    status_indicator,
)

# Cache common Color objects to reduce allocations
_COLOR_CACHE = {
    'black': Color(0, 0, 0),
    'white': Color(255, 255, 255),
    'red': Color(255, 0, 0),
    'green': Color(0, 255, 0),
    'blue': Color(0, 0, 255),
}


class StubPixelStrip:
    """Minimal stand-in used when the hardware strip cannot be initialized."""

    def __init__(self, num_pixels=16):
        self._num_pixels = num_pixels
        self._pixels = [0] * num_pixels

    def begin(self):  # pragma: no cover - trivial stub
        return None

    def numPixels(self):
        return self._num_pixels

    def setPixelColor(self, index, color):
        if 0 <= index < self._num_pixels:
            self._pixels[index] = color

    def show(self):  # pragma: no cover - trivial stub
        return None

class LedController:
    """
    Controller class for WS2812/WS2811 LED strips using rpi_ws281x.
    """
    def __init__(self, num_pixels=16, pin=18, brightness=255):
        """
        Initialize the LED strip with comprehensive error handling.

        Args:
            num_pixels (int): Number of LEDs (1-1024).
            pin (int): GPIO pin (default: 18).
            brightness (int): Brightness (0–255).
        """
        # Validate inputs
        if not isinstance(num_pixels, int) or num_pixels < 1 or num_pixels > 1024:
            raise ValueError(f"num_pixels must be 1-1024, got {num_pixels}")
        if not isinstance(pin, int) or pin < 0 or pin > 27:
            raise ValueError(f"pin must be 0-27 (GPIO), got {pin}")
        if not isinstance(brightness, int) or brightness < 0 or brightness > 255:
            raise ValueError(f"brightness must be 0-255, got {brightness}")
        
        self.num_pixels = num_pixels
        self.is_on = False
        self._is_stub = False
        
        try:
            print(f"🔧 [INIT] Initializing LED strip: {num_pixels} pixels, pin {pin}, brightness {brightness}")
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
            print(f"✅ [SUCCESS] LED strip initialized successfully")
            self._is_stub = False
        except ImportError as e:
            print(f"❌ [ERROR] rpi_ws281x library not available: {e}")
            print(f"   Falling back to StubPixelStrip (no hardware output)")
            self.strip = StubPixelStrip(num_pixels)
            self._is_stub = True
        except Exception as exc:
            # Check if it's a permission error (expected when not running as root)
            if "Permission denied" in str(exc) or "mmap() failed" in str(exc) or "code -5" in str(exc):
                print(f"⚠️  [WARN] Hardware access denied (need sudo for hardware control)")
                print(f"   Falling back to StubPixelStrip (no hardware output)")
                print(f"   To control hardware LEDs, run with: sudo python3 ...")
            else:
                print(f"❌ [ERROR] Failed to initialize LED strip: {exc}")
                print(f"   Error type: {type(exc).__name__}")
                print(f"   Falling back to StubPixelStrip (no hardware output)")
            self.strip = StubPixelStrip(num_pixels)
            self._is_stub = True

    def color_wipe(self, color, wait_ms=50):
        """Optimized color wipe using batch operations."""
        try:
            # Batch set all pixels (more efficient than individual calls)
            for i in range(self.num_pixels):
                self.strip.setPixelColor(i, color)
            self.strip.show()
            self.is_on = True
        except Exception as e:
            print(f"❌ [ERROR] color_wipe failed: {e}")
            raise

    def clear_strip(self):
        """Optimized strip clearing using cached black color."""
        try:
            black = _COLOR_CACHE['black']
            for i in range(self.num_pixels):
                self.strip.setPixelColor(i, black)
            self.strip.show()
            self.is_on = False
        except Exception as e:
            print(f"❌ [ERROR] clear_strip failed: {e}")
            raise

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
        """Optimized brightness application with pre-computed color."""
        try:
            # Pre-compute color once, then apply to all pixels
            r = int(((base_color >> 16) & 255) * brightness)
            g = int(((base_color >> 8) & 255) * brightness)
            b = int((base_color & 255) * brightness)
            color = Color(r, g, b)
            for i in range(self.num_pixels):
                self.strip.setPixelColor(i, color)
            self.strip.show()
        except Exception as e:
            print(f"❌ [ERROR] _apply_brightness failed: {e}")
            raise

    @staticmethod
    def _int_to_rgb(color):
        return (
            (color >> 16) & 255,
            (color >> 8) & 255,
            color & 255,
        )

    @staticmethod
    def _scale_rgb(rgb, scale, brightness):
        r, g, b = rgb
        return (
            max(0, min(255, int(r * scale * brightness))),
            max(0, min(255, int(g * scale * brightness))),
            max(0, min(255, int(b * scale * brightness))),
        )

    def rainbow(self, wait_ms=20, brightness=1.0, iterations=1):
        """Optimized rainbow with pre-computed brightness scaling and configurable iterations."""
        try:
            # Pre-compute wait time to avoid repeated division
            wait_sec = max(0.01, wait_ms / 1000.0)
            # Cache brightness multiplication
            brightness_r = brightness
            brightness_g = brightness
            brightness_b = brightness
            
            for iteration in range(iterations):
                for j in range(256):
                    for i in range(self.num_pixels):
                        base_color = self._wheel((i + j) & 255)
                        # Optimize: extract RGB once, apply brightness, create Color once
                        r = int(((base_color >> 16) & 255) * brightness_r)
                        g = int(((base_color >> 8) & 255) * brightness_g)
                        b = int((base_color & 255) * brightness_b)
                        self.strip.setPixelColor(i, Color(r, g, b))
                    self.strip.show()
                    time.sleep(wait_sec)
            self.is_on = True
        except Exception as e:
            print(f"❌ [ERROR] rainbow failed: {e}")
            raise

    def lightshow(self, color, interval=100, brightness=1.0, cycles=3):
        base_rgb = self._int_to_rgb(color)
        palette_rgb = [
            self._scale_rgb(base_rgb, 1.0, brightness),
            self._scale_rgb(base_rgb, 0.6, brightness),
            self._scale_rgb(base_rgb, 0.25, brightness),
            self._scale_rgb((255, 255, 255), 0.5, brightness),
        ]
        palette = [Color(*rgb) for rgb in palette_rgb]
        sparkle = Color(*self._scale_rgb((255, 255, 255), 1.0, brightness))

        delay = max(interval / 1000.0, 0.02)
        sparkle_delay = max(delay / 2.0, 0.01)
        total_cycles = max(1, cycles)

        for cycle in range(total_cycles):
            for offset in range(self.num_pixels):
                for i in range(self.num_pixels):
                    self.strip.setPixelColor(i, palette[(i + offset + cycle) % len(palette)])
                self.strip.show()
                time.sleep(delay)

            for offset in range(self.num_pixels):
                for i in range(self.num_pixels):
                    self.strip.setPixelColor(i, palette[(i + offset + cycle) % len(palette)])
                self.strip.setPixelColor((offset * 2) % self.num_pixels, sparkle)
                self.strip.show()
                time.sleep(sparkle_delay)

        self.is_on = True

    def set_led(self, color, mode="single", pattern="static", interval=500, brightness=1.0, color2=0x000000):
        """
        Set LED strip color/pattern/mode/brightness with comprehensive validation.

        Args:
            color (int): 24-bit RGB integer (0-16777215) for primary color.
            mode (str): Lighting mode ("single", "dual", "rainbow").
            pattern (str): Animation pattern.
            interval (int): Timing for dynamic patterns (ms).
            brightness (float): 0.0–1.0, global brightness.
            color2 (int): 24-bit RGB integer (0-16777215) for secondary color (used in dual mode).
        """
        try:
            # Validate inputs
            if not isinstance(color, int) or color < 0 or color > 0xFFFFFF:
                raise ValueError(f"color must be 0-16777215 (0xFFFFFF), got {color}")
            if not isinstance(brightness, (int, float)) or brightness < 0 or brightness > 1:
                raise ValueError(f"brightness must be 0.0-1.0, got {brightness}")
            if not isinstance(interval, int) or interval < 0:
                raise ValueError(f"interval must be >= 0, got {interval}")
            
            brightness = float(brightness)
            interval = max(0, min(60000, interval))  # Clamp to reasonable range
            
            if pattern == "off":
                self.clear_strip()
                return

            # Pre-compute RGB values once
            raw_r = (color >> 16) & 255
            raw_g = (color >> 8) & 255
            raw_b = color & 255

            r = int(raw_r * brightness)
            g = int(raw_g * brightness)
            b = int(raw_b * brightness)

            # Pre-compute RGB values for color2 (used in dual mode)
            raw_r2 = (color2 >> 16) & 255
            raw_g2 = (color2 >> 8) & 255
            raw_b2 = color2 & 255

            r2 = int(raw_r2 * brightness)
            g2 = int(raw_g2 * brightness)
            b2 = int(raw_b2 * brightness)

            # Pattern routing - check pattern first, then mode as fallback
            if pattern == "rainbow" or mode == "rainbow":
                # Rainbow runs continuously - use iterations for duration control
                iterations = max(1, int(interval / 20)) if interval > 0 else 1
                self.rainbow(interval if interval > 0 else 20, brightness, iterations)
                self.is_on = True
            elif pattern == "lightshow":
                # Lightshow pattern - multi-stage animation
                self.lightshow(color, interval=interval, brightness=brightness)
                self.is_on = True
            elif pattern == "static":
                # Static solid color
                self.color_wipe(Color(r, g, b), wait_ms=10)
                self.is_on = True
            elif pattern == "blink":
                # Blink pattern - on/off blinking (or between two colors in dual mode)
                from rpi_ws281x import Color as WsColor
                blink_delay = max(0.05, interval / 1000.0) if interval > 0 else 0.5
                if mode == "dual":
                    blink(self.strip, WsColor(r, g, b), WsColor(r2, g2, b2), delay=blink_delay)
                else:
                    blink(self.strip, WsColor(r, g, b), WsColor(0, 0, 0), delay=blink_delay)
                self.is_on = True
            elif pattern == "fade":
                # Fade pattern - smooth color transitions (between two colors in dual mode)
                fade_delay = max(0.01, interval / 1000.0 / 50) if interval > 0 else 0.02
                if mode == "dual":
                    fade(self.strip, (r, g, b), (r2, g2, b2), steps=50, delay=fade_delay)
                else:
                    fade(self.strip, (r, g, b), None, steps=50, delay=fade_delay)
                self.is_on = True
            elif pattern == "chase":
                # Chase pattern - moving chase effect (between two colors in dual mode)
                from rpi_ws281x import Color as WsColor
                chase_delay = max(0.01, interval / 1000.0 / self.num_pixels) if interval > 0 else 0.05
                if mode == "dual":
                    chase(self.strip, WsColor(r, g, b), WsColor(r2, g2, b2), delay=chase_delay)
                else:
                    chase(self.strip, WsColor(r, g, b), WsColor(0, 0, 0), delay=chase_delay)
                self.is_on = True
            elif pattern == "pulse":
                # Pulse pattern - fade in/out
                pulse_cycles = max(3, int(interval / 200)) if interval > 0 else 5
                pulse_delay = max(0.01, interval / 1000.0 / 50) if interval > 0 else 0.02
                for _ in range(pulse_cycles):
                    # Fade in
                    for i in range(0, 256, 5):
                        step_brightness = brightness * (i / 255.0)
                        self._apply_brightness(color, step_brightness)
                        time.sleep(pulse_delay)
                    # Fade out
                    for i in range(255, 0, -5):
                        step_brightness = brightness * (i / 255.0)
                        self._apply_brightness(color, step_brightness)
                        time.sleep(pulse_delay)
                self.is_on = True
            elif pattern in ("music", "music-reactive"):
                # Music reactive pattern - audio visualization
                update_ms = max(50, interval) if interval > 0 else 80
                duration = max(8.0, update_ms / 1000.0 * 80)
                music_visualizer(
                    self.strip,
                    base_color=(raw_r, raw_g, raw_b),
                    brightness=brightness,
                    update_ms=int(update_ms),
                    duration_s=duration,
                )
                self.is_on = True
            elif pattern == "rave":
                # Rave mode - energetic dancing lights (no audio required)
                update_ms = max(30, interval) if interval > 0 else 50
                duration = max(10.0, update_ms / 1000.0 * 200)  # Run longer for rave mode
                rave_mode(
                    self.strip,
                    base_rgb=(raw_r, raw_g, raw_b),
                    brightness=brightness,
                    interval_ms=int(update_ms),
                    duration_s=duration,
                )
                self.is_on = True
            elif pattern == "breathing":
                # Breathing effect - smooth pulse like breathing
                breathing_interval = max(50, interval) if interval > 0 else 100
                breathing(
                    self.strip,
                    base_rgb=(raw_r, raw_g, raw_b),
                    brightness=brightness,
                    interval_ms=int(breathing_interval),
                    cycles=5,
                )
                self.is_on = True
            elif pattern == "aurora":
                # Aurora effect - flowing northern lights
                update_ms = max(50, interval) if interval > 0 else 80
                duration = max(10.0, update_ms / 1000.0 * 250)
                aurora(
                    self.strip,
                    base_rgb=(raw_r, raw_g, raw_b),
                    brightness=brightness,
                    interval_ms=int(update_ms),
                    duration_s=duration,
                )
                self.is_on = True
            elif pattern == "matrix":
                # Matrix rain effect
                update_ms = max(50, interval) if interval > 0 else 100
                duration = max(10.0, update_ms / 1000.0 * 150)
                matrix_rain(
                    self.strip,
                    base_rgb=(raw_r, raw_g, raw_b),
                    brightness=brightness,
                    interval_ms=int(update_ms),
                    duration_s=duration,
                )
                self.is_on = True
            elif pattern == "fire":
                # Fire effect - flickering flames (uses predefined fire colors)
                # Note: Fire effect uses orange/red/yellow colors by design for realism
                # The selected color is ignored to maintain authentic fire appearance
                update_ms = max(30, interval) if interval > 0 else 50
                duration = max(10.0, update_ms / 1000.0 * 400)
                fire_effect(
                    self.strip,
                    brightness=brightness,
                    interval_ms=int(update_ms),
                    duration_s=duration,
                )
                self.is_on = True
            else:
                # Unknown pattern - log error and fall back to static
                print(f"⚠️ [WARN] Unknown pattern: {pattern}, falling back to static", file=sys.stderr)
                self.color_wipe(Color(r, g, b), wait_ms=10)
                self.is_on = True

        except ValueError as e:
            print(f"❌ [ERROR] Invalid LED command: {e}")
            raise
        except Exception as e:
            print(f"❌ [ERROR] LED operation failed: {e}")
            print(f"   Pattern: {pattern}, Mode: {mode}, Color: {color:06x}")
            traceback.print_exc()
            raise

    def toggle_light(self):
        if self.is_on:
            self.clear_strip()
            print("LEDs turned OFF")
        else:
            self.color_wipe(Color(255, 255, 255), wait_ms=20)
            print("LEDs turned ON")

    def get_status(self):
        return "ON" if self.is_on else "OFF"
    
    def cleanup(self):
        """Clean up resources. Safe to call multiple times."""
        try:
            if not self._is_stub and hasattr(self.strip, 'cleanup'):
                self.strip.cleanup()
        except Exception:
            pass  # Ignore cleanup errors
    
    def __del__(self):
        """Destructor - ensure cleanup on deletion."""
        try:
            self.cleanup()
        except Exception:
            pass  # Ignore errors during destruction


# Backwards compatible aliases expected by older modules/tests
LedControl = LedController

if __name__ == "__main__":
    # CLI usage: python3 led_control.py <hexcolor> <hexcolor2> <mode> <pattern> <interval> <brightness>
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

    if len(sys.argv) not in (6, 7):
        print("Usage: python3 led_control.py <hexcolor> <hexcolor2> <mode> <pattern> <interval> <brightness>")
        print("   or: python3 led_control.py off")
        print("   or: python3 led_control.py toggle")
        sys.exit(1)

    try:
        color = int(sys.argv[1], 16)
        color2 = int(sys.argv[2], 16) if len(sys.argv) >= 7 else 0x000000
        mode = sys.argv[3]
        pattern = sys.argv[4]
        interval = int(sys.argv[5])
        brightness = float(sys.argv[6]) if len(sys.argv) == 7 else 1.0

        print(f"🎨 [LED] Setting: color=#{color:06x}, color2=#{color2:06x}, mode={mode}, pattern={pattern}, interval={interval}ms, brightness={brightness}")
        led_control = LedController()
        led_control.set_led(color, mode, pattern, interval, brightness, color2=color2)
        print(f"✅ [SUCCESS] LED command completed")
    except ValueError as e:
        print(f"❌ [ERROR] Invalid input: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ [ERROR] LED command failed: {e}")
        traceback.print_exc()
        sys.exit(1)
