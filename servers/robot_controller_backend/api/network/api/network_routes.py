"""
Network API Routes

Provides endpoints for network management (AP mode, Client mode, Wi-Fi scanning).
Includes input validation for security.
"""

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import JSONResponse
from typing import Dict, Any, List, Optional
import logging
import sys
import os

# Add parent directories to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))
from api.input_validators import validate_ip_address, validate_network_ssid, validate_network_password

log = logging.getLogger(__name__)

router = APIRouter(prefix="/api/network", tags=["network"])

# Network wizard path
NETWORK_WIZARD_PATH = os.path.join(
    os.path.dirname(__file__),
    "../../../network/wizard/network_wizard.py"
)


@router.get("/")
async def get_network_status() -> Dict[str, Any]:
    """
    Get unified network status.
    
    Returns:
        Network status including mode, IP, SSID, services, VPN status
    """
    try:
        from network.diagnostics.net_summary import get_network_summary
        summary = get_network_summary()
        return {
            "ok": True,
            "network": summary
        }
    except Exception as e:
        log.error(f"Failed to get network status: {e}")
        raise HTTPException(status_code=500, detail="Failed to get network status")


@router.post("/mode")
async def set_network_mode(
    mode: str = Query(..., description="Network mode: 'ap' or 'client'")
) -> Dict[str, Any]:
    """
    Switch network mode (AP or Client).
    
    Args:
        mode: 'ap' for Access Point mode, 'client' for Wi-Fi Client mode
    
    Returns:
        Success status and new mode
    """
    # Validate mode parameter
    mode = mode.lower().strip()
    if mode not in ['ap', 'client']:
        raise HTTPException(
            status_code=400,
            detail="Mode must be 'ap' or 'client'"
        )
    
    try:
        import subprocess
        result = subprocess.run(
            ["sudo", "python3", NETWORK_WIZARD_PATH, mode],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        if result.returncode == 0:
            return {
                "ok": True,
                "mode": mode,
                "message": f"Network mode switched to {mode}"
            }
        else:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to switch network mode: {result.stderr}"
            )
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=504, detail="Network mode switch timed out")
    except Exception as e:
        log.error(f"Failed to set network mode: {e}")
        raise HTTPException(status_code=500, detail="Failed to set network mode")


@router.post("/validate")
async def validate_network_config() -> Dict[str, Any]:
    """
    Validate network configuration.
    
    Returns:
        Validation results
    """
    try:
        import subprocess
        result = subprocess.run(
            ["sudo", "python3", NETWORK_WIZARD_PATH, "validate"],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        return {
            "ok": result.returncode == 0,
            "valid": result.returncode == 0,
            "output": result.stdout,
            "errors": result.stderr if result.returncode != 0 else None
        }
    except Exception as e:
        log.error(f"Failed to validate network config: {e}")
        raise HTTPException(status_code=500, detail="Failed to validate network config")


@router.get("/wifi/scan")
async def scan_wifi() -> Dict[str, Any]:
    """
    Scan for available Wi-Fi networks.
    
    Returns:
        List of available Wi-Fi networks
    """
    try:
        from network.wifi.scan import scan_wifi_networks
        networks = scan_wifi_networks()
        return {
            "ok": True,
            "networks": networks
        }
    except Exception as e:
        log.error(f"Failed to scan Wi-Fi: {e}")
        raise HTTPException(status_code=500, detail="Failed to scan Wi-Fi networks")


@router.post("/wifi/connect")
async def connect_wifi(
    ssid: str = Query(..., description="Wi-Fi SSID"),
    password: Optional[str] = Query(None, description="Wi-Fi password (if required)")
) -> Dict[str, Any]:
    """
    Connect to a Wi-Fi network.
    
    Args:
        ssid: Wi-Fi network SSID
        password: Wi-Fi password (if required)
    
    Returns:
        Connection status
    """
    # Validate inputs
    try:
        ssid = validate_network_ssid(ssid)
        if password:
            password = validate_network_password(password)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    
    try:
        from network.wifi.connect import connect_to_wifi
        result = connect_to_wifi(ssid, password)
        return {
            "ok": result.get("success", False),
            "message": result.get("message", "Connection attempt completed"),
            "ssid": ssid
        }
    except Exception as e:
        log.error(f"Failed to connect to Wi-Fi: {e}")
        raise HTTPException(status_code=500, detail="Failed to connect to Wi-Fi network")

