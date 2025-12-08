"""
VPN Module

Provides VPN status checking (Tailscale).
"""

from .tailscale_status import get_tailscale_status

__all__ = ['get_tailscale_status']

