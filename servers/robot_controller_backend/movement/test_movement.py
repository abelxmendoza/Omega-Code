import websocket
import json
import time
from evdev import InputDevice, categorize, ecodes

# WebSocket connection
ws = websocket.create_connection("ws://127.0.0.1:8081/movement")
speed = 0  # Track current speed
buzzer_active = False  # Track buzzer state

# Find the keyboard event file (adjust if necessary)
device_path = "/dev/input/event0"  # Change if needed, check `ls /dev/input/`
try:
    dev = InputDevice(device_path)
    print(f"✅ Listening for input on {device_path}")
except FileNotFoundError:
    print(f"❌ No input device found at {device_path}")
    exit(1)

def send_command(command):
    """ Sends a command to the WebSocket server and prints it. """
    message = json.dumps({"command": command})
    ws.send(message)
    print(f"✅ Sent: {command}")

print("🔹 Controls: W (Forward), S (Backward), P (Speed+), O (Speed-), SPACE (Stop), 0 (Buzzer), I (LED)")

# Key mapping (modify if necessary)
key_map = {
    ecodes.KEY_W: "move-up",
    ecodes.KEY_S: "move-down",
    ecodes.KEY_P: "increase-speed",
    ecodes.KEY_O: "decrease-speed",
    ecodes.KEY_SPACE: "emergency-stop",
    ecodes.KEY_0: "buzz",
    ecodes.KEY_I: "led-control"
}

for event in dev.read_loop():
    if event.type == ecodes.EV_KEY and event.value == 1:  # Key press event
        key = event.code
        if key in key_map:
            send_command(key_map[key])

        if key == ecodes.KEY_P:
            speed = min(speed + 10, 100)
            print(f"⚡ Speed increased to {speed}%")
        elif key == ecodes.KEY_O:
            speed = max(speed - 10, 0)
            print(f"🐢 Speed decreased to {speed}%")
        elif key == ecodes.KEY_0:
            buzzer_active = not buzzer_active
            print("🔊 Buzzer ON" if buzzer_active else "🔇 Buzzer OFF")
    elif event.type == ecodes.EV_KEY and event.value == 0:  # Key release event
        if event.code == ecodes.KEY_ESC:
            print("🚪 Exiting...")
            ws.close()
            break
