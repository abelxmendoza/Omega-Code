"""
Network Management API Routes

Provides REST endpoints for Omega-1 Network Wizard functionality.
Allows frontend to control Wi-Fi mode switching (AP/Client).
"""

import logging
import subprocess
import json
import os
from typing import Dict, Any, Optional
from fastapi import APIRouter, HTTPException, Body
from pydantic import BaseModel, Field

log = logging.getLogger(__name__)

router = APIRouter(prefix="/api/network", tags=["Network"])

# Path to network wizard script
NETWORK_WIZARD_PATH = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "network",
    "network_wizard.py"
)
NETWORK_STATE_FILE = "/etc/omega-network/state.json"


class NetworkModeRequest(BaseModel):
    """Request model for setting network mode."""
    mode: str = Field(..., description="Network mode: 'ap' or 'client'")


class NetworkStatusResponse(BaseModel):
    """Response model for network status."""
    mode: str
    ap_ssid: Optional[str] = None
    ap_ip: Optional[str] = None
    services: Dict[str, str]
    wlan0_ip: Optional[str] = None
    last_updated: Optional[str] = None


def run_network_command(command: str) -> Dict[str, Any]:
    """
    Run network wizard command via sudo.
    Returns success status and output.
    """
    try:
        if not os.path.exists(NETWORK_WIZARD_PATH):
            return {
                "success": False,
                "error": f"Network wizard not found at {NETWORK_WIZARD_PATH}"
            }
        
        # Run with sudo
        result = subprocess.run(
            ["sudo", "python3", NETWORK_WIZARD_PATH, command],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        if result.returncode == 0:
            return {
                "success": True,
                "output": result.stdout,
                "message": f"Network mode switched to {command}"
            }
        else:
            return {
                "success": False,
                "error": result.stderr or result.stdout,
                "returncode": result.returncode
            }
    except subprocess.TimeoutExpired:
        return {
            "success": False,
            "error": "Network command timed out"
        }
    except Exception as e:
        log.error(f"Network command failed: {e}")
        return {
            "success": False,
            "error": str(e)
        }


def get_network_state() -> Optional[Dict[str, Any]]:
    """Load network state from file."""
    if os.path.exists(NETWORK_STATE_FILE):
        try:
            with open(NETWORK_STATE_FILE, "r") as f:
                return json.load(f)
        except Exception as e:
            log.warning(f"Failed to load network state: {e}")
    return None


def get_service_status(service: str) -> str:
    """Get systemd service status."""
    try:
        result = subprocess.run(
            ["systemctl", "is-active", service],
            capture_output=True,
            text=True,
            timeout=5
        )
        return "active" if result.returncode == 0 else "inactive"
    except Exception:
        return "unknown"


def get_wlan0_ip() -> Optional[str]:
    """Get wlan0 interface IP address."""
    try:
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
                        ip = parts[1].split("/")[0]
                        return ip
    except Exception:
        pass
    return None


@router.get("/status")
async def get_network_status() -> Dict[str, Any]:
    """
    Get current network status.
    
    Returns:
    - Current mode (ap/client)
    - Service statuses
    - IP address
    - AP configuration (if in AP mode)
    """
    try:
        state = get_network_state()
        mode = state.get("mode", "unknown") if state else "unknown"
        
        services = {
            "hostapd": get_service_status("hostapd"),
            "dnsmasq": get_service_status("dnsmasq"),
            "dhcpcd": get_service_status("dhcpcd"),
            "wpa_supplicant": get_service_status("wpa_supplicant"),
        }
        
        wlan0_ip = get_wlan0_ip()
        
        response = {
            "ok": True,
            "mode": mode,
            "services": services,
            "wlan0_ip": wlan0_ip,
        }
        
        if state:
            response["ap_ssid"] = state.get("ap_ssid")
            response["ap_ip"] = state.get("ap_ip")
            response["last_updated"] = state.get("last_updated")
        
        return response
    except Exception as e:
        log.error(f"Failed to get network status: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/mode")
async def set_network_mode(request: NetworkModeRequest) -> Dict[str, Any]:
    """
    Set network mode (AP or Client).
    
    Requires:
    - mode: "ap" or "client"
    
    Note: This requires sudo privileges. The backend must be run with
    appropriate permissions or have sudo access configured.
    """
    mode = request.mode.lower()
    
    if mode not in ["ap", "client"]:
        raise HTTPException(
            status_code=400,
            detail=f"Invalid mode: {mode}. Must be 'ap' or 'client'"
        )
    
    log.info(f"Switching network mode to: {mode}")
    
    result = run_network_command(mode)
    
    if result.get("success"):
        return {
            "ok": True,
            "mode": mode,
            "message": result.get("message", f"Network mode set to {mode}"),
            "output": result.get("output", "")
        }
    else:
        error_msg = result.get("error", "Unknown error")
        log.error(f"Network mode switch failed: {error_msg}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to switch network mode: {error_msg}"
        )


@router.post("/validate")
async def validate_network_config() -> Dict[str, Any]:
    """
    Run network configuration validation.
    
    Checks:
    - Configuration files exist
    - Services are running correctly
    - IP addresses are configured properly
    """
    try:
        result = run_network_command("validate")
        
        if result.get("success"):
            return {
                "ok": True,
                "valid": True,
                "message": "Network configuration is valid",
                "output": result.get("output", "")
            }
        else:
            return {
                "ok": True,
                "valid": False,
                "message": "Network configuration validation failed",
                "error": result.get("error", "Unknown error"),
                "output": result.get("output", "")
            }
    except Exception as e:
        log.error(f"Network validation failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/info")
async def get_network_info() -> Dict[str, Any]:
    """
    Get detailed network information.
    
    Returns comprehensive network information including:
    - Current mode and state
    - All service statuses
    - Interface details
    - Configuration summary
    """
    try:
        state = get_network_state()
        mode = state.get("mode", "unknown") if state else "unknown"
        
        # Get detailed service statuses
        services = {}
        for service in ["hostapd", "dnsmasq", "dhcpcd", "wpa_supplicant"]:
            status = get_service_status(service)
            services[service] = {
                "status": status,
                "enabled": False  # Could check with systemctl is-enabled
            }
        
        # Get interface details
        wlan0_ip = get_wlan0_ip()
        
        # Get SSID if connected to Wi-Fi
        ssid = None
        try:
            result = subprocess.run(
                ["iwgetid", "-r", "wlan0"],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result.returncode == 0:
                ssid = result.stdout.strip()
        except Exception:
            pass
        
        response = {
            "ok": True,
            "mode": mode,
            "services": services,
            "interfaces": {
                "wlan0": {
                    "ip": wlan0_ip,
                    "ssid": ssid,
                    "status": "up" if wlan0_ip else "down"
                }
            }
        }
        
        if state:
            response["ap_config"] = {
                "ssid": state.get("ap_ssid"),
                "ip": state.get("ap_ip"),
            }
            response["last_updated"] = state.get("last_updated")
        
        return response
    except Exception as e:
        log.error(f"Failed to get network info: {e}")
        raise HTTPException(status_code=500, detail=str(e))

