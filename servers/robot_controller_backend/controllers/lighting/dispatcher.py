"""
Lighting Dispatcher Module

File Location:
~/Omega-Code/servers/robot_controller_backend/controllers/lighting/dispatcher.py

Receives lighting control commands and routes them to the appropriate LED pattern functions
based on UI input or WebSocket messages.
"""

import sys
from functools import lru_cache
from controllers.lighting.patterns import (
    blink,
    chase,
    color_wipe,
    dual_color,
    fade,
    music_visualizer,
    lightshow,
    rainbow,
    rave_mode,
    breathing,
    aurora,
    matrix_rain,
    fire_effect,
    status_indicator,
)
from rpi_ws281x import Color

# Cache hex to RGB conversions (common colors)
@lru_cache(maxsize=128)
def hex_to_rgb(hex_color: str):
    """
    Optimized hex to RGB conversion with caching.
    Convert a hex string like '#ff0000' to an RGB tuple.
    """
    hex_color = hex_color.lstrip('#').upper()
    if len(hex_color) != 6:
        raise ValueError(f"Color hex string must be in the format #RRGGBB, got: {hex_color}")
    try:
        return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    except ValueError as e:
        raise ValueError(f"Invalid hex color: {hex_color} - {e}")

# Pattern routing dictionary for O(1) lookup instead of if/elif chain
_PATTERN_HANDLERS = {
    "static": lambda strip, c1, c2, i, b, m: (
        dual_color(strip, Color(*c1), Color(*c2)) if m == "dual"
        else color_wipe(strip, Color(*c1))
    ),
    "fade": lambda strip, c1, c2, i, b, m: fade(strip, c1, c2, delay=i / 1000.0),
    "blink": lambda strip, c1, c2, i, b, m: blink(strip, Color(*c1), Color(*c2), delay=i / 1000.0),
    "chase": lambda strip, c1, c2, i, b, m: chase(strip, Color(*c1), Color(*c2), delay=i / 1000.0),
}

def apply_lighting_mode(payload: dict, led_controller):
    """
    Optimized dispatcher with comprehensive error handling and validation.
    Routes the payload from the UI or API to the appropriate LED pattern function.

    Args:
        payload (dict): Lighting settings from frontend or API.
        led_controller: Instance of LedController.
    """
    try:
        # Extract and validate inputs
        pattern = payload.get("pattern", "static").lower()
        mode = payload.get("mode", "single").lower()
        interval = payload.get("interval", 1000)
        color_hex = payload.get("color", "#ffffff")
        brightness = payload.get("brightness", 1.0)
        
        # Validate and clamp brightness
        try:
            brightness = max(0.0, min(1.0, float(brightness)))
        except (TypeError, ValueError):
            print(f"⚠️ [WARN] Invalid brightness value: {payload.get('brightness')}, using 1.0", file=sys.stderr)
            brightness = 1.0
        
        # Validate interval
        if not isinstance(interval, (int, float)) or interval < 0:
            print(f"⚠️ [WARN] Invalid interval: {interval}, using 1000ms", file=sys.stderr)
            interval = 1000
        interval = max(0, min(60000, int(interval)))  # Clamp to 0-60s
        
        # Convert hex to RGB (cached)
        try:
            color1 = hex_to_rgb(color_hex)
        except ValueError as e:
            print(f"❌ [ERROR] Invalid color hex: {color_hex} - {e}", file=sys.stderr)
            raise
        
        # Pre-compute scaled colors
        color1_scaled = tuple(int(channel * brightness) for channel in color1)
        
        # Placeholder for dual color (future UI support)
        color2 = (0, 0, 0)
        color2_scaled = tuple(int(channel * brightness) for channel in color2)
        
        strip = led_controller.strip
        
        # Optimized pattern routing with dictionary lookup
        if pattern == "lightshow" or mode == "lightshow":
            lightshow(strip, color1, interval_ms=interval, brightness=brightness)
        elif pattern == "rainbow" or mode == "rainbow":
            rainbow(strip, wait_ms=interval)
        elif pattern in {"music", "music-reactive"}:
            update_ms = interval if interval > 0 else 80
            duration = max(8.0, update_ms / 1000.0 * 80)
            music_visualizer(
                strip,
                base_color=color1,
                brightness=brightness,
                update_ms=int(update_ms),
                duration_s=duration,
            )
        elif pattern == "rave":
            # Rave mode - energetic dancing lights (no audio required)
            update_ms = interval if interval > 0 else 50
            duration = max(10.0, update_ms / 1000.0 * 200)
            rave_mode(
                strip,
                base_rgb=color1,
                brightness=brightness,
                interval_ms=int(update_ms),
                duration_s=duration,
            )
        elif pattern == "breathing":
            # Breathing effect - smooth pulse
            breathing(
                strip,
                base_rgb=color1,
                brightness=brightness,
                interval_ms=interval if interval > 0 else 50,
                cycles=5,
            )
        elif pattern == "aurora":
            # Aurora effect - flowing northern lights
            update_ms = interval if interval > 0 else 80
            duration = max(10.0, update_ms / 1000.0 * 250)
            aurora(
                strip,
                base_rgb=color1,
                brightness=brightness,
                interval_ms=int(update_ms),
                duration_s=duration,
            )
        elif pattern == "matrix":
            # Matrix rain effect
            update_ms = interval if interval > 0 else 100
            duration = max(10.0, update_ms / 1000.0 * 150)
            matrix_rain(
                strip,
                base_rgb=color1,
                brightness=brightness,
                interval_ms=int(update_ms),
                duration_s=duration,
            )
        elif pattern == "fire":
            # Fire effect - flickering flames
            update_ms = interval if interval > 0 else 50
            duration = max(10.0, update_ms / 1000.0 * 400)
            fire_effect(
                strip,
                brightness=brightness,
                interval_ms=int(update_ms),
                duration_s=duration,
            )
        elif pattern in _PATTERN_HANDLERS:
            _PATTERN_HANDLERS[pattern](strip, color1_scaled, color2_scaled, interval, brightness, mode)
        else:
            raise ValueError(f"Unknown pattern: {pattern} (supported: static, fade, blink, chase, rainbow, lightshow, music, rave, breathing, aurora, matrix, fire)")
            
    except ValueError as e:
        print(f"❌ [ERROR] Invalid lighting command: {e}", file=sys.stderr)
        raise
    except Exception as e:
        print(f"❌ [ERROR] Lighting dispatcher failed: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        raise
