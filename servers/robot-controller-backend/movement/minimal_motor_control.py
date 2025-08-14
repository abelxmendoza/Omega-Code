# /Omega-Code/servers/robot-controller-backend/movement/minimal_motor_control.py
from PCA9685 import PCA9685
import time

class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)

    def setMotors(self, duty):
        """All 4 wheels same direction (tank forward/back)."""
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

    # --- NEW: turning helpers ---

    def pivot_left(self, speed=1500):
        """Left side backward, right side forward (in-place turn)."""
        print("Pivot left")
        L = -abs(speed)
        R = +abs(speed)
        self._set_lr(L, R)

    def pivot_right(self, speed=1500):
        """Right side backward, left side forward (in-place turn)."""
        print("Pivot right")
        L = +abs(speed)
        R = -abs(speed)
        self._set_lr(L, R)

    def left(self, speed=1500, ratio=0.5):
        """Gentle left: slow left side, full right side forward."""
        print("Turn left")
        L = int(abs(speed) * ratio)
        R = +abs(speed)
        self._set_lr(L, R)

    def right(self, speed=1500, ratio=0.5):
        """Gentle right: slow right side, full left side forward."""
        print("Turn right")
        L = +abs(speed)
        R = int(abs(speed) * ratio)
        self._set_lr(L, R)

    # Low-level helper: set left/right sides with signed duty
    def _set_lr(self, left_duty, right_duty):
        # Left pair: channels 0/1 and 4/5; Right pair: 2/3 and 6/7 (adjust if different)
        for ch_fwd, ch_rev, duty in [(0,1,left_duty), (4,5,left_duty),
                                     (2,3,right_duty), (6,7,right_duty)]:
            if duty >= 0:
                self.pwm.setMotorPwm(ch_fwd, duty)
                self.pwm.setMotorPwm(ch_rev, 0)
            else:
                self.pwm.setMotorPwm(ch_fwd, 0)
                self.pwm.setMotorPwm(ch_rev, -duty)

if __name__ == "__main__":
    m = Motor()
    try:
        m.forward(1800); time.sleep(1)
        m.left(1600);    time.sleep(1)
        m.right(1600);   time.sleep(1)
        m.pivot_left(1500);  time.sleep(0.8)
        m.pivot_right(1500); time.sleep(0.8)
        m.backward(1800); time.sleep(1)
    finally:
        m.stop()
