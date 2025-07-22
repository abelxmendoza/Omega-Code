# /Omega-Code/servers/robot-controller-backend/movement/keyboard_forward_backward.py

from minimal_motor_control import Motor
from pynput import keyboard

motor = Motor()

def on_press(key):
    try:
        if key.char == 'w':
            motor.forward()
        elif key.char == 's':
            motor.backward()
        elif key.char == ' ':
            motor.stop()
    except AttributeError:
        pass

print("Controls: W = forward | S = backward | SPACE = stop | ESC to exit")

with keyboard.Listener(on_press=on_press) as listener:
    listener.join()

