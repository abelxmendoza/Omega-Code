"""
Unified Network Management API Routes
=====================================

Single unified endpoint: GET /api/network

Provides comprehensive network status and control:
- Mode switching (AP/Client)
- Network summary
- Wi-Fi management
- Validation
"""

import logging
import subprocess
import os
import sys
from typing import Dict, Any
from fastapi import APIRouter, HTTPException, Body, Query
from pydantic import BaseModel, Field

# Import network modules
# Calculate path to network module
NETWORK_MODULE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
    "network"
)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

try:
    from network.wizard.network_wizard import (
        enable_ap_mode,
        enable_client_mode,
        validate_network_config,
    )
    from network.diagnostics.net_summary import get_network_summary
    from network.wifi.scan import scan_wifi_networks
    from network.wifi.connect import connect_to_wifi, forget_wifi
except ImportError:
    # Fallback: direct import
    import importlib.util
    wizard_path = os.path.join(NETWORK_MODULE_PATH, "wizard", "network_wizard.py")
    diagnostics_path = os.path.join(NETWORK_MODULE_PATH, "diagnostics", "net_summary.py")
    wifi_scan_path = os.path.join(NETWORK_MODULE_PATH, "wifi", "scan.py")
    wifi_connect_path = os.path.join(NETWORK_MODULE_PATH, "wifi", "connect.py")
    
    spec = importlib.util.spec_from_file_location("network_wizard", wizard_path)
    network_wizard = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(network_wizard)
    enable_ap_mode = network_wizard.enable_ap_mode
    enable_client_mode = network_wizard.enable_client_mode
    validate_network_config = network_wizard.validate_network_config
    
    spec = importlib.util.spec_from_file_location("net_summary", diagnostics_path)
    net_summary = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(net_summary)
    get_network_summary = net_summary.get_network_summary
    
    spec = importlib.util.spec_from_file_location("wifi_scan", wifi_scan_path)
    wifi_scan = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(wifi_scan)
    scan_wifi_networks = wifi_scan.scan_wifi_networks
    
    spec = importlib.util.spec_from_file_location("wifi_connect", wifi_connect_path)
    wifi_connect = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(wifi_connect)
    connect_to_wifi = wifi_connect.connect_to_wifi
    forget_wifi = wifi_connect.forget_wifi

log = logging.getLogger(__name__)

router = APIRouter(prefix="/api/network", tags=["Network"])


class NetworkModeRequest(BaseModel):
    """Request model for setting network mode."""
    mode: str = Field(..., description="Network mode: 'ap' or 'client'")


class WiFiConnectRequest(BaseModel):
    """Request model for connecting to Wi-Fi."""
    ssid: str = Field(..., description="Wi-Fi network SSID")
    password: str = Field(None, description="Wi-Fi network password (optional for open networks)")


@router.get("")
async def get_network() -> Dict[str, Any]:
    """
    Unified network endpoint.
    
    Returns comprehensive network summary:
    - mode: ap | client
    - interface: wlan0/bnep0
    - ssid, ip, gateway, rssi
    - vpn_status: Tailscale status
    - pan_status: Bluetooth PAN status
    - services_running: Service statuses
    - errors/warnings: Any issues
    """
    try:
        summary = get_network_summary()
        return summary
    except Exception as e:
        log.error(f"Failed to get network summary: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/mode")
async def set_network_mode(request: NetworkModeRequest) -> Dict[str, Any]:
    """
    Set network mode (AP or Client).
    
    Uses omega-nettoggle.sh for clean mode switching.
    
    Requires:
    - mode: "ap" or "client" (client = restore)
    
    Note: This requires sudo privileges.
    """
    mode = request.mode.lower()
    
    if mode not in ["ap", "client"]:
        raise HTTPException(
            status_code=400,
            detail=f"Invalid mode: {mode}. Must be 'ap' or 'client'"
        )
    
    log.info(f"Switching network mode to: {mode}")
    
    try:
        # Use omega-nettoggle.sh for clean switching
        script_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
            "network",
            "omega-nettoggle.sh"
        )
        
        # Map "client" to "restore" for omega-nettoggle
        nettoggle_mode = "restore" if mode == "client" else "ap"
        
        result = subprocess.run(
            ["sudo", script_path, nettoggle_mode],
            capture_output=True,
            text=True,
            timeout=60
        )
        
        if result.returncode == 0:
            return {
                "ok": True,
                "mode": mode,
                "message": f"Network mode set to {mode}",
                "output": result.stdout
            }
        else:
            error_msg = result.stderr or result.stdout
            log.error(f"Network mode switch failed: {error_msg}")
            raise HTTPException(
                status_code=500,
                detail=f"Failed to switch network mode: {error_msg}"
            )
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=500, detail="Network mode switch timed out")
    except Exception as e:
        log.error(f"Network mode switch error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/validate")
async def validate_config() -> Dict[str, Any]:
    """
    Run network configuration validation checks.
    """
    try:
        results = validate_network_config()
        all_valid = all(results.values())
        
        return {
            "ok": True,
            "valid": all_valid,
            "results": results,
            "message": "Network configuration is valid" if all_valid else "Some validations failed",
        }
    except Exception as e:
        log.error(f"Network validation failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/toggle/status")
async def get_toggle_status() -> Dict[str, Any]:
    """
    Get network status using omega-nettoggle.sh diagnostics.
    
    Returns comprehensive network diagnostics including:
    - WiFi radio status
    - Network interfaces
    - NetworkManager status
    - Service status
    - Current mode detection
    """
    try:
        script_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
            "network",
            "omega-nettoggle.sh"
        )
        
        if not os.path.exists(script_path):
            raise HTTPException(
                status_code=404,
                detail="omega-nettoggle.sh not found. Run install.sh first."
            )
        
        result = subprocess.run(
            ["sudo", script_path, "status"],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        return {
            "ok": True,
            "status": result.stdout,
            "exit_code": result.returncode
        }
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=500, detail="Status check timed out")
    except Exception as e:
        log.error(f"Failed to get toggle status: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/wifi/scan")
async def scan_wifi() -> Dict[str, Any]:
    """
    Scan for available Wi-Fi networks.
    """
    try:
        networks = scan_wifi_networks()
        return {
            "ok": True,
            "networks": networks,
            "count": len(networks),
        }
    except Exception as e:
        log.error(f"Wi-Fi scan failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/wifi/connect")
async def connect_wifi(request: WiFiConnectRequest) -> Dict[str, Any]:
    """
    Connect to a Wi-Fi network.
    
    Security:
    - Validates SSID and password input
    - Masks password in logs
    """
    try:
        # Validate input
        from api.input_validators import validate_network_ssid, validate_network_password
        
        validate_network_ssid(request.ssid)
        if request.password:
            validate_network_password(request.password)
        
        # Log connection attempt (but mask password)
        log.info(f"Wi-Fi connection attempt to SSID: {request.ssid} (password masked)")
        
        result = connect_to_wifi(request.ssid, request.password)
        return {
            "ok": result["success"],
            "message": result["message"],
            "error": result.get("error"),
        }
    except ValueError as e:
        # Input validation error
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        # Don't log password in error messages
        log.error(f"Wi-Fi connect failed for SSID: {request.ssid}")
        raise HTTPException(status_code=500, detail="Failed to connect to Wi-Fi network")


@router.delete("/wifi/forget")
async def forget_wifi_network(ssid: str = Query(..., description="SSID to forget")) -> Dict[str, Any]:
    """
    Forget a Wi-Fi network.
    """
    try:
        result = forget_wifi(ssid)
        return {
            "ok": result["success"],
            "message": result["message"],
            "error": result.get("error"),
        }
    except Exception as e:
        log.error(f"Wi-Fi forget failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))

