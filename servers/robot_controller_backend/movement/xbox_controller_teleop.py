#!/usr/bin/env python3
"""
Xbox Controller Teleoperation for Robot (GTA-Style Controls)
============================================================
Reads Xbox controller input and sends commands to robot over network.

GTA-Style Controls:
- Right Trigger: Forward (gas)
- Left Trigger: Backward (brake/reverse)
- Left Stick X: Pivot turns (left/right)
- D-Pad: Camera control (up/down/left/right)
- B Button: Horn/Buzzer toggle
- X Button: Lights on/off toggle
- A Button: Emergency stop

Features:
- GTA-style driving controls (triggers for gas/brake)
- Pivot turns with left stick
- Camera control with D-Pad
- Sends commands over UDP to robot at 192.168.50.2
- Safe default: zero velocity if triggers released or connection lost
- Automatic controller detection

Usage:
    # Auto-detect mode (recommended)
    python3 xbox_controller_teleop.py
    
    # Force local mode (controller on robot)
    python3 xbox_controller_teleop.py --mode local
    
    # Force remote mode (controller on laptop)
    python3 xbox_controller_teleop.py --mode remote --robot-ip 192.168.50.2
    
    # Custom robot IP and port
    python3 xbox_controller_teleop.py --robot-ip 192.168.1.100 --port 8888

Configuration Modes:
    auto (default): Auto-detect based on network connectivity
    local: Controller on robot, process commands locally (no network needed)
    remote: Controller on laptop, send commands over network to robot

Requirements:
    pip install evdev
    pip install websocket-client  # For local mode
"""

import os
import sys
import time
import json
import socket
import select
import argparse
import signal
from typing import Optional, Tuple
from evdev import InputDevice, list_devices, categorize, ecodes

# Default configuration
DEFAULT_ROBOT_IP = "192.168.50.2"
DEFAULT_PORT = 8888
DEFAULT_DEADMAN = "trigger"  # "trigger" or "bumper"

# Xbox controller axis mappings (evdev codes)
# Left stick: ABS_Y (vertical), ABS_X (horizontal)
# Right stick: ABS_RY (vertical), ABS_RX (horizontal)
# Triggers: ABS_Z (left), ABS_RZ (right)
# Buttons: BTN_TL (left bumper), BTN_TR (right bumper)
# D-Pad: ABS_HAT0X (horizontal), ABS_HAT0Y (vertical)

AXIS_LEFT_STICK_Y = ecodes.ABS_Y
AXIS_LEFT_STICK_X = ecodes.ABS_X
AXIS_RIGHT_STICK_X = ecodes.ABS_RX
AXIS_LEFT_TRIGGER = ecodes.ABS_Z
AXIS_RIGHT_TRIGGER = ecodes.ABS_RZ
AXIS_DPAD_X = ecodes.ABS_HAT0X
AXIS_DPAD_Y = ecodes.ABS_HAT0Y
BTN_LEFT_BUMPER = ecodes.BTN_TL
BTN_RIGHT_BUMPER = ecodes.BTN_TR
BTN_A = ecodes.BTN_SOUTH  # A button (green)
BTN_B = ecodes.BTN_EAST   # B button (red)
BTN_X = ecodes.BTN_NORTH  # X button (blue)
BTN_Y = ecodes.BTN_WEST   # Y button (yellow)

# Velocity limits
MAX_LINEAR_VELOCITY = 1.0   # m/s (adjust based on robot capabilities)
MAX_ANGULAR_VELOCITY = 1.0   # rad/s (adjust based on robot capabilities)

# Dead zone threshold (ignore small stick movements)
DEAD_ZONE = 0.1

# Command send rate (Hz)
COMMAND_RATE = 20  # Send commands 20 times per second


class XboxControllerTeleop:
    """Xbox controller teleoperation handler."""
    
    def __init__(self, robot_ip: str, port: int, deadman_type: str, mode: str = "auto"):
        self.robot_ip = robot_ip
        self.port = port
        self.deadman_type = deadman_type
        self.mode = mode
        self.sock: Optional[socket.socket] = None
        self.ws_client = None  # WebSocket client for local mode
        self.device: Optional[InputDevice] = None
        self.running = True
        
        # Current controller state
        self.left_stick_y = 0.0
        self.left_stick_x = 0.0
        self.right_stick_x = 0.0
        self.left_trigger = 0.0
        self.right_trigger = 0.0
        self.left_bumper = False
        self.dpad_x = 0
        self.dpad_y = 0
        
        # Button states
        self.horn_active = False
        self.lights_on = False
        
        # Last sent velocity (for safety)
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_pivot_command = None
        self.last_pivot_time = 0.0
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        print("\n🛑 Shutting down...")
        self.stop()
        sys.exit(0)
    
    def detect_mode(self) -> str:
        """Auto-detect operation mode based on network connectivity."""
        if self.mode != "auto":
            return self.mode
        
        # Try to reach robot IP
        try:
            test_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            test_sock.settimeout(0.5)
            test_sock.connect((self.robot_ip, self.port))
            test_sock.close()
            print(f"🌐 Network reachable - using REMOTE mode")
            return "remote"
        except:
            print(f"🏠 Robot IP not reachable - using LOCAL mode")
            return "local"
    
    def find_xbox_controller(self) -> Optional[str]:
        """Find Xbox controller event device path.

        Strategy:
        1. evdev list_devices() — needs input group, gives richest info.
        2. /proc/bus/input/devices — world-readable; maps name → event node.
        3. First /dev/input/event* that can be opened (last resort).
        """
        import glob as _glob
        import re as _re

        # --- Strategy 1: evdev (needs input group) ---
        try:
            devices = [InputDevice(path) for path in list_devices()]
            for device in devices:
                caps = device.capabilities()
                if ecodes.EV_ABS in caps:
                    abs_caps = caps[ecodes.EV_ABS]
                    has_stick = any(axis in abs_caps for axis in [
                        AXIS_LEFT_STICK_Y, AXIS_RIGHT_STICK_X
                    ])
                    if has_stick:
                        print(f"✅ Found controller (evdev): {device.name} at {device.path}")
                        return device.path
        except Exception:
            pass

        # --- Strategy 2: /proc/bus/input/devices (world-readable) ---
        try:
            with open("/proc/bus/input/devices") as fh:
                content = fh.read()
            name = None
            handlers: list = []
            for line in content.splitlines():
                if line.startswith("N: Name="):
                    name = line.split("=", 1)[1].strip('"')
                    handlers = []
                elif line.startswith("H: Handlers="):
                    handlers = line.split("=", 1)[1].split()
                elif line == "" and name:
                    keywords = ("xbox", "gamepad", "joystick", "controller", "pad")
                    if any(k in name.lower() for k in keywords):
                        event_nodes = [h for h in handlers if h.startswith("event")]
                        if event_nodes:
                            path = f"/dev/input/{event_nodes[0]}"
                            print(f"✅ Found controller (proc): {name} at {path}")
                            return path
                    name = None
                    handlers = []
        except Exception:
            pass

        # --- Strategy 3: first readable event node ---
        for path in sorted(_glob.glob("/dev/input/event*")):
            try:
                dev = InputDevice(path)
                caps = dev.capabilities()
                if ecodes.EV_ABS in caps:
                    print(f"✅ Found controller (fallback): {dev.name} at {path}")
                    return path
            except Exception:
                continue

        return None
    
    def connect_network(self) -> bool:
        """Connect to robot over UDP (remote mode)."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(0.1)  # Non-blocking with timeout
            print(f"✅ Network socket created (UDP to {self.robot_ip}:{self.port})")
            return True
        except Exception as e:
            print(f"❌ Failed to create socket: {e}")
            return False
    
    def connect_local(self) -> bool:
        """Connect to local WebSocket movement server (local mode)."""
        try:
            import websocket
            ws_url = "ws://127.0.0.1:8081"
            print(f"🔌 Connecting to local movement server: {ws_url}")
            self.ws_client = websocket.create_connection(ws_url, timeout=2)
            print(f"✅ Connected to local movement server")
            return True
        except ImportError:
            print(f"⚠️ websocket-client not installed. Install with: pip install websocket-client")
            return False
        except Exception as e:
            print(f"❌ Failed to connect to local server: {e}")
            print(f"💡 Make sure movement_ws_server.py is running on this machine")
            return False
    
    def send_velocity_command(self, linear: float, angular: float):
        """Send velocity command to robot."""
        # Clamp velocities
        linear = max(-MAX_LINEAR_VELOCITY, min(MAX_LINEAR_VELOCITY, linear))
        angular = max(-MAX_ANGULAR_VELOCITY, min(MAX_ANGULAR_VELOCITY, angular))

        if self.mode == "local":
            # Local mode: send twist command to movement_ws_server.py
            if self.ws_client:
                try:
                    command = {
                        "command": "twist",
                        "linear_x": linear,
                        "angular_z": angular,
                    }
                    self.ws_client.send(json.dumps(command))
                    self.last_linear = linear
                    self.last_angular = angular
                    return
                except Exception:
                    # Try to reconnect once
                    if self.connect_local():
                        try:
                            self.ws_client.send(json.dumps(command))
                        except Exception:
                            pass
        else:
            # Remote mode: send over UDP
            if not self.sock:
                return
            
            # Create command message
            command = {
                "type": "cmd_vel",
                "linear": {
                    "x": linear,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": angular
                },
                "timestamp": time.time()
            }
            
            try:
                message = json.dumps(command).encode('utf-8')
                self.sock.sendto(message, (self.robot_ip, self.port))
                self.last_linear = linear
                self.last_angular = angular
            except Exception as e:
                print(f"⚠️ Failed to send command: {e}")
                # On error, send stop command
                self.send_stop()
    
    def send_stop(self):
        """Send zero velocity command (safe stop)."""
        self.send_velocity_command(0.0, 0.0)
        # Also send discrete stop command
        self.send_command("stop")
    
    def send_command(self, command: str, payload: dict = None):
        """Send discrete command to robot (for pivot, camera, horn, lights, etc.)."""
        if self.mode == "local":
            # Send directly to local WebSocket server
            if not self.ws_client:
                return
            try:
                cmd_data = {"command": command}
                if payload:
                    cmd_data.update(payload)
                self.ws_client.send(json.dumps(cmd_data))
            except Exception as e:
                print(f"⚠️ Failed to send command {command}: {e}")
                # Try to reconnect
                if not self.connect_local():
                    return
        else:
            # Send over UDP network
            if not self.sock:
                return
            
            message = {
                "command": command,
                "timestamp": time.time()
            }
            if payload:
                message.update(payload)
            
            try:
                data = json.dumps(message).encode('utf-8')
                self.sock.sendto(data, (self.robot_ip, self.port))
            except Exception as e:
                print(f"⚠️ Failed to send command {command}: {e}")
    
    def is_deadman_active(self) -> bool:
        """Check if deadman switch is active (GTA-style: any trigger pressed)."""
        # GTA-style: right trigger (gas) or left trigger (brake) acts as deadman
        return self.right_trigger > 0.1 or self.left_trigger > 0.1
    
    def normalize_axis(self, value: int, axis_info: dict) -> float:
        """Normalize axis value from raw integer to [-1.0, 1.0]."""
        if not axis_info:
            return 0.0
        
        min_val = axis_info.min
        max_val = axis_info.max
        center = (min_val + max_val) // 2
        
        # Normalize around center
        if value < center:
            normalized = (value - center) / (center - min_val)
        else:
            normalized = (value - center) / (max_val - center)
        
        # Apply dead zone
        if abs(normalized) < DEAD_ZONE:
            return 0.0
        
        # Scale to account for dead zone
        if normalized > 0:
            normalized = (normalized - DEAD_ZONE) / (1.0 - DEAD_ZONE)
        else:
            normalized = (normalized + DEAD_ZONE) / (1.0 - DEAD_ZONE)
        
        return max(-1.0, min(1.0, normalized))
    
    def _abs_info(self, axis_code: int):
        """Return the AbsInfo for an axis code, or None.

        capabilities()[EV_ABS] is a list of (code, AbsInfo) tuples — NOT a dict.
        Build a lookup dict once and cache it on the instance.
        """
        if not hasattr(self, '_abs_cache'):
            caps = self.device.capabilities()
            abs_list = caps.get(ecodes.EV_ABS, [])
            self._abs_cache = {code: info for code, info in abs_list}
        return self._abs_cache.get(axis_code)

    def process_event(self, event):
        """Process input event from controller."""
        if event.type == ecodes.EV_ABS:
            # Left stick
            if event.code == AXIS_LEFT_STICK_Y:
                self.left_stick_y = -self.normalize_axis(event.value, self._abs_info(AXIS_LEFT_STICK_Y))
            elif event.code == AXIS_LEFT_STICK_X:
                self.left_stick_x = self.normalize_axis(event.value, self._abs_info(AXIS_LEFT_STICK_X))

            # Triggers (GTA-style gas/brake)
            elif event.code == AXIS_RIGHT_TRIGGER:
                axis_info = self._abs_info(AXIS_RIGHT_TRIGGER)
                self.right_trigger = event.value / axis_info.max if axis_info else event.value / 255.0
            elif event.code == AXIS_LEFT_TRIGGER:
                axis_info = self._abs_info(AXIS_LEFT_TRIGGER)
                self.left_trigger = event.value / axis_info.max if axis_info else event.value / 255.0
            
            # D-Pad (Camera control)
            elif event.code == AXIS_DPAD_X:
                old_value = self.dpad_x
                self.dpad_x = event.value
                # Send camera commands on change
                if event.value == -1 and old_value != -1:  # Left pressed
                    self.send_command("camera-servo-left")
                    print("📷 Camera: Left")
                elif event.value == 1 and old_value != 1:  # Right pressed
                    self.send_command("camera-servo-right")
                    print("📷 Camera: Right")
            elif event.code == AXIS_DPAD_Y:
                old_value = self.dpad_y
                self.dpad_y = event.value
                # Send camera commands on change
                if event.value == -1 and old_value != -1:  # Up pressed
                    self.send_command("camera-servo-up")
                    print("📷 Camera: Up")
                elif event.value == 1 and old_value != 1:  # Down pressed
                    self.send_command("camera-servo-down")
                    print("📷 Camera: Down")
        
        elif event.type == ecodes.EV_KEY:
            # Buttons
            if event.code == BTN_B:
                if event.value == 1:  # Press
                    self.horn_active = not self.horn_active
                    self.send_command("buzz" if self.horn_active else "buzz-stop")
                    print(f"🔊 Horn: {'ON' if self.horn_active else 'OFF'}")
            elif event.code == BTN_X:
                if event.value == 1:  # Press
                    self.lights_on = not self.lights_on
                    cmd = "led-control" if self.lights_on else "led-off"
                    self.send_command(cmd)
                    print(f"💡 Lights: {'ON' if self.lights_on else 'OFF'}")
            elif event.code == BTN_A:
                if event.value == 1:  # Press
                    self.send_stop()
                    self.send_command("stop")
                    print("🛑 Emergency stop")
            elif event.code == BTN_LEFT_BUMPER:
                self.left_bumper = event.value == 1
    
    def run(self):
        """Main control loop."""
        # Detect or use specified mode
        self.mode = self.detect_mode()
        
        # Find controller
        device_path = self.find_xbox_controller()
        if not device_path:
            print("❌ No Xbox controller found!")
            print("💡 Connect controller (USB or Bluetooth) and check with: ls /dev/input/")
            return False
        
        # Open device
        try:
            self.device = InputDevice(device_path)
            print(f"✅ Controller opened: {self.device.name}")
        except Exception as e:
            print(f"❌ Failed to open controller: {e}")
            return False

        # Grab exclusive access so desktop compositors don't steal events.
        # Non-fatal: works fine without grab (events shared with kernel).
        try:
            self.device.grab()
            print("   (exclusive grab: ok)")
        except Exception as e:
            print(f"   (exclusive grab failed — running shared mode: {e})")
            print("   To fix: sudo usermod -a -G input $USER  then re-login")
        
        # Connect based on mode
        if self.mode == "local":
            if not self.connect_local():
                print("💡 Tip: Start movement_ws_server.py on this machine for local mode")
                return False
        else:
            if not self.connect_network():
                return False
        
        print(f"\n🎮 GTA-Style Controller ready!")
        if self.mode == "local":
            print(f"🏠 Mode: LOCAL (controller on robot, processing locally)")
        else:
            print(f"🌐 Mode: REMOTE (sending to {self.robot_ip}:{self.port})")
        print(f"📊 Max velocities: linear={MAX_LINEAR_VELOCITY} m/s, angular={MAX_ANGULAR_VELOCITY} rad/s")
        print(f"\n🎯 GTA-Style Controls:")
        print(f"   Right Trigger: Forward (gas) - hold to move")
        print(f"   Left Trigger: Backward (brake/reverse)")
        print(f"   Left Stick X: Pivot turns (left/right)")
        print(f"   D-Pad: Camera control (↑↓←→)")
        print(f"   B Button: Horn/Buzzer toggle")
        print(f"   X Button: Lights on/off toggle")
        print(f"   A Button: Emergency stop")
        print(f"\nPress Ctrl+C to stop\n")
        
        # Main loop — select-based so we send at a fixed rate even when
        # the trigger is held steady (no new axis events).  The watchdog
        # in movement_ws_server times out after 2s without a command;
        # we send at 20 Hz so the watchdog is always kept alive.
        last_send_time = 0.0
        send_interval = 1.0 / COMMAND_RATE  # 50 ms
        fd = self.device.fileno()

        try:
            while self.running:
                # Wait up to send_interval for device events
                ready, _, _ = select.select([fd], [], [], send_interval)
                if not self.running:
                    break

                # Drain all pending events from the device
                if ready:
                    try:
                        for event in self.device.read():
                            self.process_event(event)
                    except OSError:
                        print("⚠️ Controller disconnected")
                        break

                # Send velocity command at fixed rate (fires every ~50 ms
                # regardless of whether a new event arrived)
                current_time = time.time()
                if current_time - last_send_time >= send_interval:
                    # GTA-style movement: Triggers for forward/backward
                    linear = 0.0
                    if self.right_trigger > 0.1:
                        linear = self.right_trigger * MAX_LINEAR_VELOCITY
                    elif self.left_trigger > 0.1:
                        linear = -self.left_trigger * MAX_LINEAR_VELOCITY

                    # Pivot turns with left stick X
                    pivot_active = False
                    if abs(self.left_stick_x) > DEAD_ZONE:
                        angular = -self.left_stick_x  # left stick right → negative angular (turn right)
                        self.send_velocity_command(linear, angular)
                        pivot_active = True

                    # Forward/backward (or stop) when not pivoting
                    if not pivot_active:
                        if abs(linear) > 0.1:
                            self.send_velocity_command(linear, 0.0)
                        elif self.last_linear != 0.0:
                            self.send_velocity_command(0.0, 0.0)

                    last_send_time = current_time

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
        
        return True
    
    def stop(self):
        """Stop teleoperation and cleanup."""
        self.running = False
        
        # Send final stop command
        if self.mode == "local":
            if self.ws_client:
                self.send_stop()
                time.sleep(0.1)
        else:
            if self.sock:
                self.send_stop()
                time.sleep(0.1)  # Give time for message to send
        
        # Release controller
        if self.device:
            try:
                self.device.ungrab()
            except:
                pass
        
        # Close connections
        if self.sock:
            self.sock.close()
        if self.ws_client:
            try:
                self.ws_client.close()
            except:
                pass
        
        print("✅ Cleanup complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Xbox controller teleoperation for robot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use defaults (192.168.50.2:8888, trigger deadman)
  python3 xbox_controller_teleop.py
  
  # Custom robot IP and port
  python3 xbox_controller_teleop.py --robot-ip 192.168.1.100 --port 9999
  
  # Use bumper instead of trigger for deadman
  python3 xbox_controller_teleop.py --deadman bumper
        """
    )
    parser.add_argument(
        "--robot-ip",
        default=DEFAULT_ROBOT_IP,
        help=f"Robot IP address (default: {DEFAULT_ROBOT_IP})"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"UDP port for remote mode (default: {DEFAULT_PORT})"
    )
    parser.add_argument(
        "--mode",
        choices=["auto", "local", "remote"],
        default="auto",
        help="Operation mode: auto (detect), local (controller on robot), remote (controller on laptop)"
    )
    # Note: Deadman is now automatic (triggers act as deadman in GTA-style)
    
    args = parser.parse_args()
    
    # Create and run teleop (deadman is automatic in GTA-style)
    teleop = XboxControllerTeleop(
        robot_ip=args.robot_ip,
        port=args.port,
        deadman_type="trigger",  # Always trigger-based in GTA-style
        mode=args.mode
    )
    
    success = teleop.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
