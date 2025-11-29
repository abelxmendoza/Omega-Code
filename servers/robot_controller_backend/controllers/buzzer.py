"""
# File: /Omega-Code/servers/robot_controller_backend/controllers/buzzer.py
# Summary:
Hardware controller for robot buzzer (horn).
Call buzz_on() to activate (sound), buzz_off() to deactivate (silent).
"""
import time
import RPi.GPIO as GPIO

BUZZER_PIN = 17  # BCM numbering

def setup_buzzer():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.output(BUZZER_PIN, GPIO.LOW)  # Ensure off

def buzz_on():
    print("[BUZZER] ON")
    GPIO.output(BUZZER_PIN, GPIO.HIGH)

def buzz_off():
    print("[BUZZER] OFF")
    GPIO.output(BUZZER_PIN, GPIO.LOW)

# Allow CLI testing (optional)
if __name__ == '__main__':
    setup_buzzer()
    buzz_on()
    time.sleep(3)
    buzz_off()
