#!/usr/bin/env python3
"""
UDP Velocity Command Receiver for Robot
========================================
Receives velocity commands over UDP and forwards them to movement system.

This script runs on the robot (Raspberry Pi) and listens for cmd_vel commands
from the teleoperation client.

Usage:
    python3 udp_velocity_receiver.py [--port 8888] [--forward-ws] [--forward-ros]

Options:
    --forward-ws: Forward commands to WebSocket movement server
    --forward-ros: Forward commands to ROS2 /cmd_vel topic (if ROS2 available)
"""

import os
import sys
import json
import socket
import argparse
import signal
import time
from typing import Optional

# Add parent directory to path for imports
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.append(ROOT)

DEFAULT_PORT = 8888


class UDPVelocityReceiver:
    """Receives velocity commands over UDP and forwards them."""
    
    def __init__(self, port: int, forward_ws: bool, forward_ros: bool):
        self.port = port
        self.forward_ws = forward_ws
        self.forward_ros = forward_ros
        self.sock: Optional[socket.socket] = None
        self.running = True
        
        # Last received command time (for timeout)
        self.last_command_time = 0.0
        self.timeout_seconds = 0.5  # Stop if no command received for 0.5s
        
        # Current velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        print("\n🛑 Shutting down...")
        self.stop()
        sys.exit(0)
    
    def forward_to_websocket(self, linear: float, angular: float):
        """Forward command to WebSocket movement server."""
        try:
            import websocket
            import json as json_lib
            
            # Connect to local movement server
            ws_url = "ws://127.0.0.1:8081"
            try:
                ws = websocket.create_connection(ws_url, timeout=0.1)
                
                # Convert velocity to movement commands
                # For now, use a simple mapping
                # TODO: Better integration with movement_ws_server.py
                
                # Calculate differential drive speeds
                # left = linear + angular, right = linear - angular
                left_speed = linear + angular
                right_speed = linear - angular
                
                # Clamp to [-1, 1] and convert to PWM (0-4095)
                left_pwm = int((left_speed + 1.0) / 2.0 * 4095)
                right_pwm = int((right_speed + 1.0) / 2.0 * 4095)
                
                # Send as movement command (simplified)
                if abs(linear) > 0.1 or abs(angular) > 0.1:
                    if linear > 0:
                        cmd = {"command": "forward", "speed": left_pwm}
                    else:
                        cmd = {"command": "backward", "speed": abs(left_pwm)}
                else:
                    cmd = {"command": "stop"}
                
                ws.send(json_lib.dumps(cmd))
                ws.close()
            except Exception:
                pass  # WebSocket not available, skip
        except ImportError:
            pass  # websocket-client not installed
    
    def forward_to_ros2(self, linear: float, angular: float):
        """Forward command to ROS2 /cmd_vel topic."""
        try:
            from api.ros_native_bridge import ROS2NativeBridge
            
            bridge = ROS2NativeBridge("udp_velocity_receiver")
            bridge.publish_twist("/cmd_vel", linear, angular)
        except Exception:
            pass  # ROS2 not available
    
    def forward_discrete_command(self, command: str, payload: dict = None):
        """Forward discrete command to WebSocket movement server."""
        try:
            import websocket
            import json as json_lib
            
            ws_url = "ws://127.0.0.1:8081"
            try:
                ws = websocket.create_connection(ws_url, timeout=0.1)
                cmd_data = {"command": command}
                if payload:
                    cmd_data.update(payload)
                ws.send(json_lib.dumps(cmd_data))
                ws.close()
            except Exception:
                pass  # WebSocket not available
        except ImportError:
            pass  # websocket-client not installed
    
    def process_command(self, command: dict):
        """Process received command (velocity or discrete)."""
        try:
            cmd_type = command.get("type")
            cmd_name = command.get("command")
            
            # Handle discrete commands (pivot, camera, horn, lights, etc.)
            if cmd_name:
                self.last_command_time = time.time()
                
                # Extract payload
                payload = {k: v for k, v in command.items() if k not in ["command", "type", "timestamp"]}
                
                # Forward to WebSocket
                if self.forward_ws:
                    self.forward_discrete_command(cmd_name, payload)
                
                # Print status
                print(f"📥 Command: {cmd_name}" + (f" {payload}" if payload else ""), end='\r')
                return
            
            # Handle velocity commands (cmd_vel)
            if cmd_type == "cmd_vel":
                linear = command.get("linear", {}).get("x", 0.0)
                angular = command.get("angular", {}).get("z", 0.0)
                
                self.current_linear = linear
                self.current_angular = angular
                self.last_command_time = time.time()
                
                # Forward to configured destinations
                if self.forward_ws:
                    self.forward_to_websocket(linear, angular)
                
                if self.forward_ros:
                    self.forward_to_ros2(linear, angular)
                
                # Print status
                print(f"📥 linear={linear:+.2f} m/s, angular={angular:+.2f} rad/s", end='\r')
            
        except Exception as e:
            print(f"⚠️ Error processing command: {e}")
    
    def check_timeout(self):
        """Check if command timeout occurred and send stop."""
        if time.time() - self.last_command_time > self.timeout_seconds:
            if self.current_linear != 0.0 or self.current_angular != 0.0:
                print("\n⏱️ Timeout - sending stop command")
                self.current_linear = 0.0
                self.current_angular = 0.0
                
                if self.forward_ws:
                    self.forward_to_websocket(0.0, 0.0)
                if self.forward_ros:
                    self.forward_to_ros2(0.0, 0.0)
    
    def run(self):
        """Main receive loop."""
        # Create UDP socket
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("0.0.0.0", self.port))
            self.sock.settimeout(1.0)  # 1 second timeout for checking
            print(f"✅ Listening on UDP port {self.port}")
        except Exception as e:
            print(f"❌ Failed to create socket: {e}")
            return False
        
        print(f"📡 Waiting for velocity commands...")
        if self.forward_ws:
            print(f"   → Forwarding to WebSocket (ws://127.0.0.1:8081)")
        if self.forward_ros:
            print(f"   → Forwarding to ROS2 (/cmd_vel)")
        print(f"⏱️ Timeout: {self.timeout_seconds}s (auto-stop if no commands)")
        print(f"\nPress Ctrl+C to stop\n")
        
        self.last_command_time = time.time()
        
        try:
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(1024)
                    command = json.loads(data.decode('utf-8'))
                    self.process_command(command)
                except socket.timeout:
                    # Check for timeout
                    self.check_timeout()
                except json.JSONDecodeError:
                    print(f"⚠️ Invalid JSON from {addr}")
                except Exception as e:
                    print(f"⚠️ Error receiving: {e}")
        
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
        
        return True
    
    def stop(self):
        """Stop receiver and cleanup."""
        self.running = False
        
        # Send final stop
        if self.current_linear != 0.0 or self.current_angular != 0.0:
            print("\n🛑 Sending stop command")
            if self.forward_ws:
                self.forward_to_websocket(0.0, 0.0)
            if self.forward_ros:
                self.forward_to_ros2(0.0, 0.0)
        
        if self.sock:
            self.sock.close()
        
        print("✅ Cleanup complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="UDP velocity command receiver for robot",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"UDP port to listen on (default: {DEFAULT_PORT})"
    )
    parser.add_argument(
        "--forward-ws",
        action="store_true",
        help="Forward commands to WebSocket movement server"
    )
    parser.add_argument(
        "--forward-ros",
        action="store_true",
        help="Forward commands to ROS2 /cmd_vel topic"
    )
    
    args = parser.parse_args()
    
    # At least one forwarding method should be enabled
    if not args.forward_ws and not args.forward_ros:
        print("⚠️ Warning: No forwarding enabled. Use --forward-ws or --forward-ros")
        print("   Commands will be received but not forwarded.")
    
    # Create and run receiver
    receiver = UDPVelocityReceiver(
        port=args.port,
        forward_ws=args.forward_ws,
        forward_ros=args.forward_ros
    )
    
    success = receiver.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
