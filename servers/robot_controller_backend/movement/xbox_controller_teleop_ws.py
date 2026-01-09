#!/usr/bin/env python3
"""
Xbox Controller Teleoperation via WebSocket
============================================
Alternative version that sends commands via WebSocket instead of UDP.

This version connects to the existing WebSocket movement server and sends
velocity commands in a format compatible with the ROS2 bridge.

Usage:
    python3 xbox_controller_teleop_ws.py [--robot-ip 192.168.50.2] [--port 8081] [--deadman trigger|bumper]
"""

import os
import sys
import time
import json
import argparse
import signal
import websocket
from typing import Optional
from evdev import InputDevice, list_devices, ecodes

# Import shared constants and utilities from UDP version
# For simplicity, we'll duplicate the essential parts

DEFAULT_ROBOT_IP = "192.168.50.2"
DEFAULT_WS_PORT = 8081
DEFAULT_DEADMAN = "trigger"

AXIS_LEFT_STICK_Y = ecodes.ABS_Y
AXIS_RIGHT_STICK_X = ecodes.ABS_RX
AXIS_LEFT_TRIGGER = ecodes.ABS_Z
BTN_LEFT_BUMPER = ecodes.BTN_TL

MAX_LINEAR_VELOCITY = 1.0
MAX_ANGULAR_VELOCITY = 1.0
DEAD_ZONE = 0.1
COMMAND_RATE = 20


class XboxControllerTeleopWS:
    """Xbox controller teleoperation via WebSocket."""
    
    def __init__(self, robot_ip: str, port: int, deadman_type: str):
        self.robot_ip = robot_ip
        self.port = port
        self.deadman_type = deadman_type
        self.ws: Optional[websocket.WebSocket] = None
        self.device: Optional[InputDevice] = None
        self.running = True
        
        # Controller state
        self.left_stick_y = 0.0
        self.right_stick_x = 0.0
        self.left_trigger = 0.0
        self.left_bumper = False
        
        self.last_linear = 0.0
        self.last_angular = 0.0
        
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        print("\n🛑 Shutting down...")
        self.stop()
        sys.exit(0)
    
    def find_xbox_controller(self) -> Optional[str]:
        """Find Xbox controller device path."""
        devices = [InputDevice(path) for path in list_devices()]
        
        for device in devices:
            caps = device.capabilities()
            if ecodes.EV_ABS in caps:
                abs_caps = caps[ecodes.EV_ABS]
                has_stick = any(axis in abs_caps for axis in [
                    AXIS_LEFT_STICK_Y, AXIS_RIGHT_STICK_X
                ])
                if has_stick:
                    print(f"✅ Found controller: {device.name} at {device.path}")
                    return device.path
        
        return None
    
    def connect_websocket(self) -> bool:
        """Connect to robot WebSocket server."""
        try:
            ws_url = f"ws://{self.robot_ip}:{self.port}"
            print(f"🔌 Connecting to {ws_url}...")
            self.ws = websocket.create_connection(ws_url, timeout=3)
            print("✅ WebSocket connected")
            return True
        except Exception as e:
            print(f"❌ Failed to connect: {e}")
            return False
    
    def send_velocity_command(self, linear: float, angular: float):
        """Send velocity command via WebSocket."""
        if not self.ws:
            return
        
        linear = max(-MAX_LINEAR_VELOCITY, min(MAX_LINEAR_VELOCITY, linear))
        angular = max(-MAX_ANGULAR_VELOCITY, min(MAX_ANGULAR_VELOCITY, angular))
        
        # Format for ROS2 bridge (if available) or movement server
        # Try ROS2 bridge format first
        command = {
            "type": "publish",
            "topic": "/cmd_vel",
            "msg_type": "Twist",
            "command": {
                "linear": {"x": linear, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": angular}
            }
        }
        
        try:
            self.ws.send(json.dumps(command))
            self.last_linear = linear
            self.last_angular = angular
        except Exception as e:
            print(f"⚠️ Failed to send: {e}")
            # Try to reconnect
            if not self.connect_websocket():
                self.send_stop()
    
    def send_stop(self):
        """Send zero velocity command."""
        self.send_velocity_command(0.0, 0.0)
    
    def is_deadman_active(self) -> bool:
        """Check if deadman switch is active."""
        if self.deadman_type == "trigger":
            return self.left_trigger > 0.5
        else:
            return self.left_bumper
    
    def normalize_axis(self, value: int, axis_info: dict) -> float:
        """Normalize axis value to [-1.0, 1.0]."""
        if not axis_info:
            return 0.0
        
        min_val = axis_info.min
        max_val = axis_info.max
        center = (min_val + max_val) // 2
        
        if value < center:
            normalized = (value - center) / (center - min_val)
        else:
            normalized = (value - center) / (max_val - center)
        
        if abs(normalized) < DEAD_ZONE:
            return 0.0
        
        if normalized > 0:
            normalized = (normalized - DEAD_ZONE) / (1.0 - DEAD_ZONE)
        else:
            normalized = (normalized + DEAD_ZONE) / (1.0 - DEAD_ZONE)
        
        return max(-1.0, min(1.0, normalized))
    
    def process_event(self, event):
        """Process input event."""
        if event.type == ecodes.EV_ABS:
            if event.code == AXIS_LEFT_STICK_Y:
                axis_info = self.device.capabilities()[ecodes.EV_ABS][AXIS_LEFT_STICK_Y]
                self.left_stick_y = -self.normalize_axis(event.value, axis_info)
            elif event.code == AXIS_RIGHT_STICK_X:
                axis_info = self.device.capabilities()[ecodes.EV_ABS][AXIS_RIGHT_STICK_X]
                self.right_stick_x = self.normalize_axis(event.value, axis_info)
            elif event.code == AXIS_LEFT_TRIGGER:
                axis_info = self.device.capabilities()[ecodes.EV_ABS][AXIS_LEFT_TRIGGER]
                if axis_info:
                    self.left_trigger = event.value / axis_info.max
                else:
                    self.left_trigger = event.value / 255.0
        elif event.type == ecodes.EV_KEY:
            if event.code == BTN_LEFT_BUMPER:
                self.left_bumper = event.value == 1
    
    def run(self):
        """Main control loop."""
        device_path = self.find_xbox_controller()
        if not device_path:
            print("❌ No Xbox controller found!")
            return False
        
        try:
            self.device = InputDevice(device_path)
            self.device.grab()
            print(f"✅ Controller opened: {self.device.name}")
        except Exception as e:
            print(f"❌ Failed to open controller: {e}")
            return False
        
        if not self.connect_websocket():
            return False
        
        print(f"\n🎮 Controller ready!")
        print(f"📡 Connected to {self.robot_ip}:{self.port}")
        print(f"🛡️ Deadman switch: {self.deadman_type}")
        print(f"\nPress Ctrl+C to stop\n")
        
        last_send_time = 0.0
        send_interval = 1.0 / COMMAND_RATE
        
        try:
            for event in self.device.read_loop():
                if not self.running:
                    break
                
                self.process_event(event)
                
                current_time = time.time()
                if current_time - last_send_time >= send_interval:
                    if self.is_deadman_active():
                        linear = self.left_stick_y * MAX_LINEAR_VELOCITY
                        angular = self.right_stick_x * MAX_ANGULAR_VELOCITY
                        self.send_velocity_command(linear, angular)
                    else:
                        if self.last_linear != 0.0 or self.last_angular != 0.0:
                            self.send_stop()
                    
                    last_send_time = current_time
        
        except OSError:
            print("⚠️ Controller disconnected")
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
        
        return True
    
    def stop(self):
        """Stop and cleanup."""
        self.running = False
        
        if self.ws:
            self.send_stop()
            time.sleep(0.1)
            try:
                self.ws.close()
            except:
                pass
        
        if self.device:
            try:
                self.device.ungrab()
            except:
                pass
        
        print("✅ Cleanup complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Xbox controller teleoperation via WebSocket"
    )
    parser.add_argument(
        "--robot-ip",
        default=DEFAULT_ROBOT_IP,
        help=f"Robot IP address (default: {DEFAULT_ROBOT_IP})"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_WS_PORT,
        help=f"WebSocket port (default: {DEFAULT_WS_PORT})"
    )
    parser.add_argument(
        "--deadman",
        choices=["trigger", "bumper"],
        default=DEFAULT_DEADMAN,
        help="Deadman switch type"
    )
    
    args = parser.parse_args()
    
    teleop = XboxControllerTeleopWS(
        robot_ip=args.robot_ip,
        port=args.port,
        deadman_type=args.deadman
    )
    
    success = teleop.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
