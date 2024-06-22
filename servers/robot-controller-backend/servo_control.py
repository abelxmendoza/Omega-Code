import sys
from Adafruit_PCA9685 import PCA9685

class Servo:
    def __init__(self):
        self.pwm = PCA9685()
        self.pwm.set_pwm_freq(50)
        self.set_servo_angle(8, 90)
        self.set_servo_angle(9, 90)

    def set_servo_angle(self, channel, angle):
        pulse_length = 1000000.0    # 1,000,000 us per second
        pulse_length /= 60.0        # 60 Hz
        pulse_length /= 4096.0      # 12 bits of resolution
        pulse = int((angle * 1000.0 / 90.0) / pulse_length)
        self.pwm.set_pwm(channel, 0, pulse)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 servo_control.py <servo-type> <angle>")
        sys.exit(1)

    servo_type = sys.argv[1]
    angle = int(sys.argv[2])

    servo = Servo()
    if servo_type == "horizontal":
        servo.set_servo_angle(8, angle)
    elif servo_type == "vertical":
        servo.set_servo_angle(9, angle)
