"""
Wi-Fi Client Connection Management
==================================

Provides Wi-Fi connection functionality using nmcli.
"""

import subprocess
from typing import Dict, Any, Optional


def connect_to_wifi(ssid: str, password: Optional[str] = None) -> Dict[str, Any]:
    """
    Connect to a Wi-Fi network.
    
    Args:
        ssid: Network SSID
        password: Network password (optional for open networks)
    
    Returns:
        {
            "success": bool,
            "message": str,
            "error": str | None,
        }
    """
    try:
        if password:
            # WPA/WPA2 network
            result = subprocess.run(
                ["nmcli", "device", "wifi", "connect", ssid, "password", password],
                capture_output=True,
                text=True,
                timeout=30
            )
        else:
            # Open network
            result = subprocess.run(
                ["nmcli", "device", "wifi", "connect", ssid],
                capture_output=True,
                text=True,
                timeout=30
            )
        
        if result.returncode == 0:
            return {
                "success": True,
                "message": f"Successfully connected to {ssid}",
                "error": None,
            }
        else:
            return {
                "success": False,
                "message": f"Failed to connect to {ssid}",
                "error": result.stderr or result.stdout,
            }
    except subprocess.TimeoutExpired:
        return {
            "success": False,
            "message": f"Connection to {ssid} timed out",
            "error": "Timeout",
        }
    except Exception as e:
        return {
            "success": False,
            "message": f"Error connecting to {ssid}",
            "error": str(e),
        }


def forget_wifi(ssid: str) -> Dict[str, Any]:
    """
    Forget a Wi-Fi network.
    
    Args:
        ssid: Network SSID to forget
    
    Returns:
        {
            "success": bool,
            "message": str,
            "error": str | None,
        }
    """
    try:
        result = subprocess.run(
            ["nmcli", "connection", "delete", ssid],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode == 0:
            return {
                "success": True,
                "message": f"Forgot network {ssid}",
                "error": None,
            }
        else:
            return {
                "success": False,
                "message": f"Failed to forget {ssid}",
                "error": result.stderr or result.stdout,
            }
    except Exception as e:
        return {
            "success": False,
            "message": f"Error forgetting {ssid}",
            "error": str(e),
        }

