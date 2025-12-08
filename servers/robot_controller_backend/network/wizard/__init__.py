"""
Omega-1 Network Wizard - Core Module

Manages Wi-Fi mode switching (AP/Client) for Raspberry Pi.
"""

from .network_wizard import (
    enable_ap_mode,
    enable_client_mode,
    get_network_status,
    validate_network_config,
    AP_SSID,
    AP_PASSWORD,
    AP_IP,
    AP_DHCP_START,
    AP_DHCP_END,
)

__all__ = [
    'enable_ap_mode',
    'enable_client_mode',
    'get_network_status',
    'validate_network_config',
    'AP_SSID',
    'AP_PASSWORD',
    'AP_IP',
    'AP_DHCP_START',
    'AP_DHCP_END',
]

