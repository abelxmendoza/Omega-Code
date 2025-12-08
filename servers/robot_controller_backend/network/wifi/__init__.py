"""
Wi-Fi Client Management Module

Provides Wi-Fi scanning and connection functionality.
"""

from .scan import scan_wifi_networks
from .connect import connect_to_wifi, forget_wifi

__all__ = ['scan_wifi_networks', 'connect_to_wifi', 'forget_wifi']

