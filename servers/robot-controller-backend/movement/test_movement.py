import websocket
import json
import time
from pynput import keyboard  # Alternative to keyboard that does not require root

ws = websocket.create_connection("ws://127.0.0.1:8081/movement")
speed = 0  # Track current speed
buzzer_active = False  # Track buzzer state

def send_command(command):
    """ Sends a command to the WebSocket server and prints it. """
    message = json.dumps({"command": command})
    ws.send(message)
    print(f"✅ Sent: {command}")

print("🔹 Controls: W (Forward), S (Backward), P (Speed+), O (Speed-), SPACE (Stop), 0 (Buzzer), I (LED)")

def on_press(key):
    global speed, buzzer_active

    try:
        if key.char == "w":
            send_command("move-up")
        elif key.char == "s":
            send_command("move-down")
        elif key.char == "p":
            speed = min(speed + 10, 100)  # Limit speed to 100%
            send_command("increase-speed")
            print(f"⚡ Speed increased to {speed}%")
        elif key.char == "o":
            speed = max(speed - 10, 0)  # Minimum speed is 0%
            send_command("decrease-speed")
            print(f"🐢 Speed decreased to {speed}%")
        elif key.char == "0":
            buzzer_active = not buzzer_active  # Toggle buzzer state
            send_command("buzz" if buzzer_active else "buzz-stop")
            print("🔊 Buzzer ON" if buzzer_active else "🔇 Buzzer OFF")
        elif key.char == "i":
            send_command("led-control")  # LED control command
            print("💡 Toggled LED")
    except AttributeError:
        if key == keyboard.Key.space:
            send_command("emergency-stop")
            speed = 0  # Reset speed on emergency stop
            print("🛑 Emergency Stop! Speed reset to 0%")

def on_release(key):
    if key == keyboard.Key.esc:
        print("🚪 Exiting...")
        ws.close()
        return False  # Stop listener

# Start listening for keyboard events
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
