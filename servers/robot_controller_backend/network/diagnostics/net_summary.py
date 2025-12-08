"""
Unified Network Summary
=======================

Single endpoint that returns comprehensive network status:
- Mode (ap/client)
- Interface details (wlan0/bnep0)
- SSID, IP, Gateway, RSSI
- VPN status (Tailscale)
- PAN status (Bluetooth)
- Service status
- Errors/warnings
"""

import os
import subprocess
import json
from typing import Dict, Any, Optional
from pathlib import Path

NETWORK_STATE_FILE = "/etc/omega-network/state.json"


def get_service_status(service: str) -> Dict[str, Any]:
    """Get systemd service status and enabled state"""
    try:
        # Check if active
        result = subprocess.run(
            ["systemctl", "is-active", service],
            capture_output=True,
            text=True,
            timeout=5
        )
        is_active = result.returncode == 0
        
        # Check if enabled
        result_enabled = subprocess.run(
            ["systemctl", "is-enabled", service],
            capture_output=True,
            text=True,
            timeout=5
        )
        is_enabled = result_enabled.returncode == 0
        
        return {
            "status": "active" if is_active else "inactive",
            "enabled": is_enabled
        }
    except Exception:
        return {"status": "unknown", "enabled": False}


def get_wlan0_info() -> Dict[str, Any]:
    """Get wlan0 interface information"""
    info = {
        "ip": None,
        "ssid": None,
        "status": "down",
        "rssi": None,
        "gateway": None,
    }
    
    try:
        # Get IP address
        result = subprocess.run(
            ["ip", "addr", "show", "wlan0"],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            for line in result.stdout.split("\n"):
                if "inet " in line and "127.0.0.1" not in line:
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        info["ip"] = parts[1].split("/")[0]
                        info["status"] = "up"
                        break
            
            # Get SSID
            result_ssid = subprocess.run(
                ["iwgetid", "-r", "wlan0"],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result_ssid.returncode == 0:
                info["ssid"] = result_ssid.stdout.strip()
            
            # Get RSSI (signal strength)
            result_rssi = subprocess.run(
                ["iwconfig", "wlan0"],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result_rssi.returncode == 0:
                for line in result_rssi.stdout.split("\n"):
                    if "Signal level" in line:
                        # Extract dBm value
                        parts = line.split("Signal level=")
                        if len(parts) > 1:
                            rssi_str = parts[1].split()[0]
                            try:
                                info["rssi"] = int(rssi_str)
                            except ValueError:
                                pass
            
            # Get gateway
            result_gw = subprocess.run(
                ["ip", "route", "show", "default"],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result_gw.returncode == 0:
                for line in result_gw.stdout.split("\n"):
                    if "wlan0" in line:
                        parts = line.split()
                        if "via" in parts:
                            idx = parts.index("via")
                            if idx + 1 < len(parts):
                                info["gateway"] = parts[idx + 1]
                                break
    except Exception:
        pass
    
    return info


def get_tailscale_status() -> Dict[str, Any]:
    """Get Tailscale VPN status"""
    status = {
        "enabled": False,
        "ip": None,
        "status": "disconnected",
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
        
        # Check status
        result_status = subprocess.run(
            ["tailscale", "status", "--json"],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result_status.returncode == 0:
            try:
                ts_data = json.loads(result_status.stdout)
                status["enabled"] = True
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
            except (json.JSONDecodeError, KeyError):
                pass
    except Exception:
        pass
    
    return status


def get_pan_status() -> Dict[str, Any]:
    """Get Bluetooth PAN status (optional)"""
    status = {
        "enabled": False,
        "interface": None,
        "ip": None,
        "status": "disconnected",
    }
    
    try:
        # Check if bnep0 interface exists
        result = subprocess.run(
            ["ip", "addr", "show", "bnep0"],
            capture_output=True,
            text=True,
            timeout=3
        )
        if result.returncode == 0:
            status["enabled"] = True
            status["interface"] = "bnep0"
            
            # Get IP
            for line in result.stdout.split("\n"):
                if "inet " in line and "127.0.0.1" not in line:
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        status["ip"] = parts[1].split("/")[0]
                        status["status"] = "connected"
                        break
    except Exception:
        pass
    
    return status


def load_network_state() -> Optional[Dict[str, Any]]:
    """Load network state from file"""
    if os.path.exists(NETWORK_STATE_FILE):
        try:
            with open(NETWORK_STATE_FILE, "r") as f:
                return json.load(f)
        except Exception:
            pass
    return None


def get_network_summary() -> Dict[str, Any]:
    """
    Get unified network summary.
    
    Returns comprehensive network status including:
    - mode: ap | client
    - interface: wlan0/bnep0
    - ssid, ip, gateway, rssi
    - vpn_status: Tailscale status
    - pan_status: Bluetooth PAN status
    - services_running: Service statuses
    - errors/warnings: Any issues
    """
    state = load_network_state()
    mode = state.get("mode", "unknown") if state else "unknown"
    
    # Get interface info
    wlan0_info = get_wlan0_info()
    
    # Determine primary interface
    interface = "wlan0"
    if wlan0_info["status"] == "down":
        pan_info = get_pan_status()
        if pan_info["status"] == "connected":
            interface = "bnep0"
    
    # Get service statuses
    services = {
        "hostapd": get_service_status("hostapd"),
        "dnsmasq": get_service_status("dnsmasq"),
        "dhcpcd": get_service_status("dhcpcd"),
        "wpa_supplicant": get_service_status("wpa_supplicant"),
    }
    
    # Get VPN status
    vpn_status = get_tailscale_status()
    
    # Get PAN status
    pan_status = get_pan_status()
    
    # Build summary
    summary = {
        "ok": True,
        "mode": mode,
        "interface": interface,
        "ssid": wlan0_info["ssid"],
        "ip": wlan0_info["ip"] or pan_status.get("ip"),
        "gateway": wlan0_info["gateway"],
        "rssi": wlan0_info["rssi"],
        "vpn_status": vpn_status,
        "pan_status": pan_status,
        "services_running": services,
    }
    
    # Add AP config if in AP mode
    if mode == "ap" and state:
        summary["ap_config"] = {
            "ssid": state.get("ap_ssid"),
            "ip": state.get("ap_ip"),
        }
    
    # Check for errors/warnings
    errors = []
    warnings = []
    
    if mode == "ap":
        if services["hostapd"]["status"] != "active":
            errors.append("hostapd is not running")
        if services["dnsmasq"]["status"] != "active":
            errors.append("dnsmasq is not running")
        if not wlan0_info["ip"]:
            errors.append("wlan0 has no IP address")
    elif mode == "client":
        if not wlan0_info["ssid"]:
            warnings.append("Not connected to Wi-Fi network")
        if not wlan0_info["ip"]:
            errors.append("wlan0 has no IP address")
    
    summary["errors"] = errors
    summary["warnings"] = warnings
    
    if state:
        summary["last_updated"] = state.get("last_updated")
    
    return summary

