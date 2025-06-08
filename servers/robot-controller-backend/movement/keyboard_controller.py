"""
# File: /Omega-Code/servers/robot-controller-backend/movement/keyboard_controller.py
# Summary:
# Allows controlling the Raspberry Pi bot using a MacBook keyboard over WebSockets.
# Handles WebSocket disconnections and reconnects automatically.
"""

import websocket
import json
import time
import os
from dotenv import load_dotenv
from pynput import keyboard  # Allows capturing key presses

# Load environment variables from .env
load_dotenv()
PI_IP = os.getenv("PI_IP", "192.168.1.134")  # Default to last known IP

# WebSocket connection setup
ws = None


def connect_ws():
    """ Connects to the Raspberry Pi WebSocket server. Handles reconnection. """
    global ws
    try:
        print(f"🔌 Connecting to Raspberry Pi at ws://{PI_IP}:8081/movement...")
        ws = websocket.create_connection(f"ws://{PI_IP}:8081/movement")
        print("✅ Connected successfully!")
    except Exception as e:
        print(f"❌ WebSocket connection failed: {e}")
        ws = None


def send_command(command):
    """ Sends a command to the WebSocket server and handles reconnection if needed. """
    global ws
    try:
        if ws:
            message = json.dumps({"command": command})
            ws.send(message)
            print(f"✅ Sent: {command}")
        else:
            raise Exception("WebSocket is not connected")
    except Exception as e:
        print(f"⚠️ WebSocket error: {e}. Reconnecting...")
        connect_ws()


# Keyboard event handling
key_mapping = {
    "w": "move-up",
    "s": "move-down",
    "p": "increase-speed",
    "o": "decrease-speed",
    "space": "emergency-stop",
    "0": "buzz",
    "i": "led-control",
}


def on_press(key):
    """ Handles key presses and sends commands. """
    try:
        key_name = key.char if hasattr(key, "char") else key.name
        if key_name in key_mapping:
            send_command(key_mapping[key_name])
    except AttributeError:
        pass


# Initialize WebSocket
connect_ws()

# Keyboard listener
print("🔹 Use your MacBook keyboard to control the robot.")
print("🔹 Controls: W (Forward), S (Backward), P (Speed+), O (Speed-), SPACE (Stop), 0 (Buzzer), I (LED)")
print("🔹 Press ESC to exit.")

with keyboard.Listener(on_press=on_press) as listener:
    listener.join()
