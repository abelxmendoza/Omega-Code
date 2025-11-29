# File: /Omega-Code/servers/robot_controller_backend/commands/buzzer_control.py

import lgpio
import sys
import time

# Configure GPIO settings
Buzzer_Pin = 17  # Define the GPIO pin connected to the buzzer

class Buzzer:
    def __init__(self):
        """Initialize the buzzer GPIO using lgpio."""
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, Buzzer_Pin, 0)

    def activate(self):
        """Activate the buzzer."""
        lgpio.gpio_write(self.h, Buzzer_Pin, 1)

    def deactivate(self):
        """Deactivate the buzzer."""
        lgpio.gpio_write(self.h, Buzzer_Pin, 0)

    def close(self):
        """Release the GPIO handle."""
        lgpio.gpiochip_close(self.h)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 buzzer_control.py <action>")
        sys.exit(1)

    action = sys.argv[1].lower()
    buzzer = Buzzer()

    try:
        if action == "on":
            buzzer.activate()
        elif action == "off":
            buzzer.deactivate()
        else:
            print("Invalid action. Use 'on' or 'off'.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        buzzer.close()
