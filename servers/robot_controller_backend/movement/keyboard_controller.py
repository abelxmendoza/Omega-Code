# /Omega-Code/servers/robot_controller_backend/movement/keyboard_controller.py
# Control the bot from your Mac keyboard over WebSocket.

import os, json, time
from dotenv import load_dotenv
from pynput import keyboard
import websocket  # pip install websocket-client

load_dotenv()
PI_IP = os.getenv("PI_IP", "192.168.1.134")
WS_URL = f"ws://{PI_IP}:8081"  # no /movement

ws = None
buzzer_on = False
speed_pct = 40  # 0..100 UI-ish; sent to server as 0..4095

def pct_to_pwm(pct: int) -> int:
    pct = max(0, min(100, pct))
    return int(pct * 4095 / 100)

def connect_ws():
    global ws
    try:
        print(f"🔌 Connecting {WS_URL} …")
        ws = websocket.create_connection(WS_URL, timeout=3)
        ws.settimeout(0.5)
        # read welcome if sent
        try:
            print("👋", ws.recv())
        except Exception:
            pass
        print("✅ Connected")
    except Exception as e:
        print(f"❌ WS connect failed: {e}")
        ws = None

def send(payload: dict):
    global ws
    if ws is None:
        connect_ws()
    try:
        ws.send(json.dumps(payload))
        try:
            print("⬅", ws.recv())
        except Exception:
            pass
    except Exception as e:
        print(f"⚠️ WS send error: {e} (reconnecting)")
        try:
            if ws: ws.close()
        except Exception:
            pass
        ws = None
        connect_ws()

def on_press(key):
    global speed_pct, buzzer_on
    name = getattr(key, "char", None)
    special = getattr(key, "name", None)

    if name:
        c = name.lower()
        if c == "w":
            send({"command": "move-up", "speed": pct_to_pwm(speed_pct)})
        elif c == "s":
            send({"command": "move-down", "speed": pct_to_pwm(speed_pct)})
        elif c == "a":
            send({"command": "move-left", "speed": pct_to_pwm(speed_pct)})
        elif c == "d":
            send({"command": "move-right", "speed": pct_to_pwm(speed_pct)})
        elif c == "p":
            speed_pct = min(speed_pct + 5, 100)
            print(f"⚡ Speed {speed_pct}%")
            send({"command": "increase-speed"})
        elif c == "o":
            speed_pct = max(speed_pct - 5, 0)
            print(f"🐢 Speed {speed_pct}%")
            send({"command": "decrease-speed"})
        elif c == "0":
            buzzer_on = not buzzer_on
            send({"command": "buzz" if buzzer_on else "buzz-stop"})
            print("🔊 Buzzer ON" if buzzer_on else "🔇 Buzzer OFF")

    if special == "space":
        send({"command": "move-stop"})
    elif special == "esc":
        print("🚪 Exiting…")
        try:
            send({"command": "move-stop"})
            send({"command": "buzz-stop"})
        finally:
            if ws:
                try: ws.close()
                except: pass
        return False  # stop listener

if __name__ == "__main__":
    print("Keys: W/S forward/back • A/D left/right • SPACE stop • P/O speed +/- • 0 buzzer toggle • ESC quit")
    connect_ws()
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()
