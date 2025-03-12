import websocket
import json
import keyboard  # Requires the `keyboard` library
import time

ws = websocket.create_connection("ws://127.0.0.1:8081/movement")
speed = 0  # Track current speed
buzzer_active = False  # Track buzzer state

def send_command(command):
    """ Sends a command to the WebSocket server and prints it. """
    message = json.dumps({"command": command})
    ws.send(message)
    print(f"✅ Sent: {command}")

print("🔹 Controls: W (Forward), S (Backward), P (Speed+), O (Speed-), SPACE (Stop), 0 (Buzzer), I (LED)")

while True:
    try:
        if keyboard.is_pressed("w"):
            send_command("move-up")
        elif keyboard.is_pressed("s"):
            send_command("move-down")
        elif keyboard.is_pressed("p"):
            speed = min(speed + 10, 100)  # Limit speed to 100%
            send_command("increase-speed")
            print(f"⚡ Speed increased to {speed}%")
            time.sleep(0.2)  # Prevent spamming
        elif keyboard.is_pressed("o"):
            speed = max(speed - 10, 0)  # Minimum speed is 0%
            send_command("decrease-speed")
            print(f"🐢 Speed decreased to {speed}%")
            time.sleep(0.2)  # Prevent spamming
        elif keyboard.is_pressed("space"):
            send_command("emergency-stop")
            speed = 0  # Reset speed on emergency stop
            print("🛑 Emergency Stop! Speed reset to 0%")
            time.sleep(0.2)
        elif keyboard.is_pressed("0"):
            buzzer_active = not buzzer_active  # Toggle buzzer state
            send_command("buzz" if buzzer_active else "buzz-stop")
            print("🔊 Buzzer ON" if buzzer_active else "🔇 Buzzer OFF")
            time.sleep(0.2)  # Prevent spamming
        elif keyboard.is_pressed("i"):
            send_command("led-control")  # LED control command
            print("💡 Toggled LED")
            time.sleep(0.2)  # Prevent spamming
    except KeyboardInterrupt:
        print("🚪 Exiting...")
        ws.close()
        break
