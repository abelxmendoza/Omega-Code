"""
Lighting Dispatcher Module

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/dispatcher.py

Receives lighting control commands and routes them to the appropriate LED pattern functions
based on UI input or WebSocket messages.
"""

from patterns import color_wipe, dual_color, fade, blink, chase, rainbow
from rpi_ws281x import Color
import re

def hex_to_rgb(hex_color):
    """Convert hex string like '#ff0000' to RGB tuple."""
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

def apply_lighting_mode(payload: dict, led_controller):
    pattern = payload.get("pattern", "static")
    mode = payload.get("mode", "single")
    interval = payload.get("interval", 1000)

    # Convert hex string to RGB tuple
    color1 = hex_to_rgb(payload.get("color", "#ffffff"))
    color2 = (0, 0, 0)  # default second color

    # TODO: support dual color input if UI sends it later

    strip = led_controller.strip

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

    elif pattern == "rainbow":
        rainbow(strip, wait_ms=interval)

    else:
        print(f"⚠️ Unknown pattern: {pattern}")
