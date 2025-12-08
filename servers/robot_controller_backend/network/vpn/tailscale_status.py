"""
Tailscale VPN Status Module
===========================

Provides Tailscale VPN status checking.
"""

import subprocess
import json
from typing import Dict, Any


def get_tailscale_status() -> Dict[str, Any]:
    """
    Get Tailscale VPN status.
    
    Returns:
        {
            "enabled": bool,
            "ip": str | None,
            "status": "connected" | "disconnected",
            "hostname": str | None,
        }
    """
    status = {
        "enabled": False,
        "ip": None,
        "status": "disconnected",
        "hostname": None,
    }
    
    try:
        # Check if tailscale is installed
        result = subprocess.run(
            ["which", "tailscale"],
            capture_output=True,
            text=True,
            timeout=3
        )
        if result.returncode != 0:
            return status
        
        status["enabled"] = True
        
        # Get status
        result_status = subprocess.run(
            ["tailscale", "status", "--json"],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result_status.returncode == 0:
            try:
                ts_data = json.loads(result_status.stdout)
                if ts_data.get("Self", {}).get("Online"):
                    status["status"] = "connected"
                    # Get Tailscale IP
                    addrs = ts_data.get("Self", {}).get("TailscaleIPs", [])
                    if addrs:
                        # Filter for 100.x.x.x addresses
                        for addr in addrs:
                            if addr.startswith("100."):
                                status["ip"] = addr
                                break
                    # Get hostname
                    hostname = ts_data.get("Self", {}).get("DNSName")
                    if hostname:
                        status["hostname"] = hostname
            except (json.JSONDecodeError, KeyError):
                pass
    except Exception:
        pass
    
    return status

