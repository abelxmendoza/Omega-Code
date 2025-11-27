# /Omega-Code/servers/robot-controller-backend/movement/keyboard_forward_backward.py

from minimal_motor_control import Motor
from pynput import keyboard

motor = Motor()

def on_press(key):
    try:
        c = key.char.lower()
        if c == 'w':
            motor.forward()
        elif c == 's':
            motor.backward()
    except Exception:
        if getattr(key, "name", "") == "space":
            motor.stop()
        elif getattr(key, "name", "") == "esc":
            print("Bye"); motor.stop(); return False

print("W forward | S backward | SPACE stop | ESC exit")
with keyboard.Listener(on_press=on_press) as listener:
    listener.join()
