"""
Lighting Dispatcher Module

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/dispatcher.py

Receives lighting control commands and routes them to the appropriate LED pattern functions
based on UI input or WebSocket messages.
"""

from controllers.lighting.patterns import color_wipe, dual_color, fade, blink, chase, rainbow
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

    color1 = hex_to_rgb(color_hex)
    # brightness is not used in patterns yet, but can be applied if needed

    # Placeholder for dual color (future UI support)
    color2 = (0, 0, 0)
    # If UI provides a second color: color2 = hex_to_rgb(payload['color2'])

    strip = led_controller.strip

    try:
        if pattern == "static":
            if mode == "dual":
                dual_color(strip, Color(*color1), Color(*color2))
            else:
                color_wipe(strip, Color(*color1))
        elif pattern == "fade":
            fade(strip, color1, color2, delay=interval / 1000.0)
        elif pattern == "blink":
            blink(strip, Color(*color1), Color(*color2), delay=interval / 1000.0)
        elif pattern == "chase":
            chase(strip, Color(*color1), Color(*color2), delay=interval / 1000.0)
        elif pattern == "rainbow" or mode == "rainbow":
            # Accept both "rainbow" as pattern or mode
            rainbow(strip, wait_ms=interval)
        else:
            print(f"⚠️ Unknown pattern: {pattern}")
    except Exception as e:
        print(f"Lighting dispatcher error: {e}")
