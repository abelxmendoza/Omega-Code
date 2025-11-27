from controllers.lighting.led_control import LedControl

class Led(LedControl):
    """Backward-compatible wrapper around :class:`LedControl`."""

    # Alias methods to maintain legacy API
    colorWipe = LedControl.color_wipe
    theaterChase = LedControl.theater_chase
    setLed = LedControl.set_led
    # Preserve legacy attribute name for compatibility
    LED_TYPE = LedControl._convert_color

