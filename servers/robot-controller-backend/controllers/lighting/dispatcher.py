"""
Lighting Dispatcher Module

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/dispatcher.py

Receives lighting control commands and routes them to the appropriate LED pattern functions
based on UI input or WebSocket messages.
"""

from controllers.lighting.patterns import (
    blink,
    chase,
    color_wipe,
    dual_color,
    fade,
    music_visualizer,
    lightshow,
    rainbow,
)
from rpi_ws281x import Color

def hex_to_rgb(hex_color: str):
    """
    Convert a hex string like '#ff0000' to an RGB tuple.
    """
    hex_color = hex_color.lstrip('#')
    if len(hex_color) != 6:
        raise ValueError("Color hex string must be in the format #RRGGBB")
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

def apply_lighting_mode(payload: dict, led_controller):
    """
    Routes the payload from the UI or API to the appropriate LED pattern function.

    Args:
        payload (dict): Lighting settings from frontend or API.
        led_controller: Instance of LedController.
    """
    pattern = payload.get("pattern", "static")
    mode = payload.get("mode", "single")
    interval = payload.get("interval", 1000)
    color_hex = payload.get("color", "#ffffff")
    brightness = payload.get("brightness", 1.0)  # optional, float 0.0–1.0
    try:
        brightness = max(0.0, min(1.0, float(brightness)))
    except (TypeError, ValueError):
        brightness = 1.0

    color1 = hex_to_rgb(color_hex)
    color1_scaled = tuple(int(channel * brightness) for channel in color1)

    # Placeholder for dual color (future UI support)
    color2 = (0, 0, 0)
    color2_scaled = tuple(int(channel * brightness) for channel in color2)
    # If UI provides a second color: color2 = hex_to_rgb(payload['color2'])

    strip = led_controller.strip

    try:
        if pattern == "static":
            if mode == "dual":
                dual_color(strip, Color(*color1_scaled), Color(*color2_scaled))
            else:
                color_wipe(strip, Color(*color1_scaled))
        elif pattern == "fade":
            fade(strip, color1_scaled, color2_scaled, delay=interval / 1000.0)
        elif pattern == "blink":
            blink(strip, Color(*color1_scaled), Color(*color2_scaled), delay=interval / 1000.0)
        elif pattern == "chase":

            chase(strip, Color(*color1_scaled), Color(*color2_scaled), delay=interval / 1000.0)

            chase(strip, Color(*color1), Color(*color2), delay=interval / 1000.0)
        elif pattern == "lightshow" or mode == "lightshow":
            lightshow(strip, color1, interval_ms=interval, brightness=brightness)
     master
        elif pattern == "rainbow" or mode == "rainbow":
            # Accept both "rainbow" as pattern or mode
            rainbow(strip, wait_ms=interval)
        elif pattern in {"music", "music-reactive"}:
            update_ms = interval if interval and interval > 0 else 80
            duration = max(8.0, update_ms / 1000.0 * 80)
            music_visualizer(
                strip,
                base_color=color1,
                brightness=brightness,
                update_ms=int(update_ms),
                duration_s=duration,
            )
        else:
            print(f"⚠️ Unknown pattern: {pattern}")
    except Exception as e:
        print(f"Lighting dispatcher error: {e}")
