"""
Health Check Functions for OmegaOS Services

Each service can have an optional health check function.
Returns: {"healthy": bool, "message": str}
"""

import subprocess
import socket
import os
import logging
from typing import Dict, Any

log = logging.getLogger(__name__)


def check_port(host: str, port: int, timeout: float = 2.0) -> bool:
    """Check if a port is open and accepting connections"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except Exception:
        return False


def movement_check() -> Dict[str, Any]:
    """Health check for Movement WebSocket Server"""
    try:
        # Check if port 8081 is listening
        if check_port("localhost", 8081):
            return {"healthy": True, "message": "Movement server is responding"}
        return {"healthy": False, "message": "Movement server port not accessible"}
    except Exception as e:
        return {"healthy": False, "message": f"Health check failed: {e}"}


def video_check() -> Dict[str, Any]:
    """Health check for Video Server"""
    try:
        # Check if port 5000 is listening
        if check_port("localhost", 5000):
            return {"healthy": True, "message": "Video server is responding"}
        return {"healthy": False, "message": "Video server port not accessible"}
    except Exception as e:
        return {"healthy": False, "message": f"Health check failed: {e}"}


def api_check() -> Dict[str, Any]:
    """Health check for Main API Server"""
    try:
        # Check if port 8000 is listening
        if check_port("localhost", 8000):
            return {"healthy": True, "message": "API server is responding"}
        return {"healthy": False, "message": "API server port not accessible"}
    except Exception as e:
        return {"healthy": False, "message": f"Health check failed: {e}"}


def ultrasonic_check() -> Dict[str, Any]:
    """Health check for Ultrasonic Sensor Server"""
    try:
        # Check if port 8080 is listening
        if check_port("localhost", 8080):
            return {"healthy": True, "message": "Ultrasonic server is responding"}
        return {"healthy": False, "message": "Ultrasonic server port not accessible"}
    except Exception as e:
        return {"healthy": False, "message": f"Health check failed: {e}"}


def line_tracker_check() -> Dict[str, Any]:
    """Health check for Line Tracker Server"""
    try:
        # Check if port 8090 is listening
        if check_port("localhost", 8090):
            return {"healthy": True, "message": "Line tracker server is responding"}
        return {"healthy": False, "message": "Line tracker server port not accessible"}
    except Exception as e:
        return {"healthy": False, "message": f"Health check failed: {e}"}


def gateway_check() -> Dict[str, Any]:
    """Health check for Gateway API"""
    try:
        # Check if port 7070 is listening
        if check_port("localhost", 7070):
            return {"healthy": True, "message": "Gateway API is responding"}
        return {"healthy": False, "message": "Gateway API port not accessible"}
    except Exception as e:
        return {"healthy": False, "message": f"Health check failed: {e}"}


def ros_core_check() -> Dict[str, Any]:
    """Health check for ROS 2 Core"""
    try:
        # Check if ROS 2 is available
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            return {"healthy": True, "message": "ROS 2 core is responding"}
        return {"healthy": False, "message": "ROS 2 core not responding"}
    except FileNotFoundError:
        return {"healthy": False, "message": "ROS 2 not installed"}
    except Exception as e:
        return {"healthy": False, "message": f"ROS health check failed: {e}"}


def sensor_check() -> Dict[str, Any]:
    """Generic sensor health check - checks I2C bus"""
    try:
        # Check if I2C device exists
        if os.path.exists("/dev/i2c-1"):
            return {"healthy": True, "message": "I2C bus is available"}
        return {"healthy": False, "message": "I2C bus not available"}
    except Exception as e:
        return {"healthy": False, "message": f"Sensor check failed: {e}"}


def telemetry_check() -> Dict[str, Any]:
    """Health check for Telemetry Server"""
    try:
        # Check if telemetry endpoint is accessible
        # This would check your telemetry server port
        # For now, check if main API is up (which serves telemetry)
        if check_port("localhost", 8000):
            return {"healthy": True, "message": "Telemetry server is responding"}
        return {"healthy": False, "message": "Telemetry server not accessible"}
    except Exception as e:
        return {"healthy": False, "message": f"Telemetry check failed: {e}"}


# Health check registry
HEALTH_CHECKS = {
    "movement_check": movement_check,
    "video_check": video_check,
    "api_check": api_check,
    "ultrasonic_check": ultrasonic_check,
    "line_tracker_check": line_tracker_check,
    "gateway_check": gateway_check,
    "ros_core_check": ros_core_check,
    "sensor_check": sensor_check,
    "telemetry_check": telemetry_check,
}


def run_health_check(check_name: str) -> Dict[str, Any]:
    """Run a health check by name"""
    if check_name is None:
        return {"healthy": True, "message": "No health check defined"}
    
    check_func = HEALTH_CHECKS.get(check_name)
    if check_func is None:
        return {"healthy": False, "message": f"Unknown health check: {check_name}"}
    
    try:
        return check_func()
    except Exception as e:
        log.error(f"Health check {check_name} raised exception: {e}")
        return {"healthy": False, "message": f"Health check exception: {e}"}

