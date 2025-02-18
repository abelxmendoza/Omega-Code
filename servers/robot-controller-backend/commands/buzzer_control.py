# File: /Omega-Code/servers/robot-controller-backend/commands/buzzer_control.py

import RPi.GPIO as GPIO
import sys
import time

# Configure GPIO settings
GPIO.setwarnings(False)
Buzzer_Pin = 17  # Define the GPIO pin connected to the buzzer
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_Pin, GPIO.OUT)

class Buzzer:
    def activate(self):
        """Activate the buzzer."""
        GPIO.output(Buzzer_Pin, True)

    def deactivate(self):
        """Deactivate the buzzer."""
        GPIO.output(Buzzer_Pin, False)


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
        GPIO.cleanup()
