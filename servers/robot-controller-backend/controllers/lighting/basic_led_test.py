"""Basic LED functionality test script."""

import time

import os
import sys

if __package__ in (None, ""):
    # Support running as a standalone script
    sys.path.append(os.path.dirname(__file__))
    from led_control import LedControl
else:  # pragma: no cover - imported as package
    from .led_control import LedControl


def run_test():
    """Run a simple sequence of LED patterns to verify functionality."""
    led = LedControl()

    print("Starting LED functionality test. Press Ctrl+C to exit.")

    # Cycle through primary colors with a color wipe effect
    colors = [0xFF0000, 0x00FF00, 0x0000FF]
    for color in colors:
        print(f"Color wipe: {color:#06x}")
        led.color_wipe(color, wait_ms=100)
        time.sleep(0.5)

    # Theater chase effect with white color
    print("Theater chase")
    led.theater_chase(0xFFFFFF, wait_ms=50, iterations=5)

    # Rainbow cycle
    print("Rainbow")
    led.rainbow(wait_ms=20, iterations=1)

    print("LED test complete.")
    # Explicitly delete the LED controller to ensure any underlying resources
    # are cleaned up before exiting. This helps avoid segfaults on some
    # platforms when the rpi_ws281x library fails to initialize.
    del led



if __name__ == "__main__":
    try:
        run_test()
    except KeyboardInterrupt:
        print("Test interrupted by user.")
