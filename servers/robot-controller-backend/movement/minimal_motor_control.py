# /Omega-Code/servers/robot-controller-backend/movement/minimal_motor_control.py

from PCA9685 import PCA9685
import time

class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)

    def setMotors(self, duty):
        """Move all 4 wheels in the same direction"""
        self.pwm.setMotorPwm(0, duty if duty > 0 else 0)
        self.pwm.setMotorPwm(1, 0 if duty > 0 else abs(duty))
        self.pwm.setMotorPwm(2, duty if duty > 0 else 0)
        self.pwm.setMotorPwm(3, 0 if duty > 0 else abs(duty))
        self.pwm.setMotorPwm(4, duty if duty > 0 else 0)
        self.pwm.setMotorPwm(5, 0 if duty > 0 else abs(duty))
        self.pwm.setMotorPwm(6, duty if duty > 0 else 0)
        self.pwm.setMotorPwm(7, 0 if duty > 0 else abs(duty))

    def forward(self, speed=2000):
        print("Moving forward")
        self.setMotors(speed)

    def backward(self, speed=2000):
        print("Moving backward")
        self.setMotors(-speed)

    def stop(self):
        print("Stopping")
        self.setMotors(0)

if __name__ == "__main__":
    motor = Motor()
    try:
        motor.forward()
        time.sleep(2)
        motor.backward()
        time.sleep(2)
        motor.stop()
    except KeyboardInterrupt:
        motor.stop()


