# File: /Omega-Code/servers/robot-controller-backend/gpio/gpio_mock.py

"""
This script simulates GPIO actions for testing purposes.
It provides a mock implementation of GPIO operations without requiring actual hardware.
"""

class GPIO:
    BCM = "BCM"
    OUT = "OUT"
    HIGH = True
    LOW = False

    @staticmethod
    def setmode(mode):
        print(f"GPIO setmode({mode})")

    @staticmethod
    def setup(pin, mode):
        print(f"GPIO setup({pin}, {mode})")

    @staticmethod
    def output(pin, state):
        print(f"GPIO output({pin}, {state})")

    @staticmethod
    def cleanup():
        print("GPIO cleanup()")

# Simulating the GPIO actions
def main():
    import sys
    pin = int(sys.argv[1])
    state = sys.argv[2].upper()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT)

    if state == "HIGH":
        GPIO.output(pin, GPIO.HIGH)
    else:
        GPIO.output(pin, GPIO.LOW)

    GPIO.cleanup()

if __name__ == "__main__":
    main()
