# tests/servo_control_test.py
import pytest
from servo_control import ServoController

def test_servo_initialization():
    servo = ServoController()
    assert servo.position == 0, "Initial position should be 0"

def test_servo_move():
    servo = ServoController()
    servo.move(90)
    assert servo.position == 90, "Servo should move to 90 degrees"
