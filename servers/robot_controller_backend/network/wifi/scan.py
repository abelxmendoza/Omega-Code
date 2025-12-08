"""
Wi-Fi Network Scanning
=====================

Provides Wi-Fi network scanning functionality using nmcli.
"""

import subprocess
import json
from typing import List, Dict, Any


def scan_wifi_networks() -> List[Dict[str, Any]]:
    """
    Scan for available Wi-Fi networks.
    
    Returns list of networks with:
        - ssid: Network name
        - security: Security type (WPA2, WPA, Open, etc.)
        - signal: Signal strength (percentage)
        - frequency: Frequency in MHz
    """
    networks = []
    
    try:
        # Use nmcli to scan
        result = subprocess.run(
            ["nmcli", "-t", "-f", "SSID,SECURITY,SIGNAL,FREQ", "device", "wifi", "list"],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode == 0:
            for line in result.stdout.strip().split("\n"):
                if not line or line.startswith("--"):
                    continue
                
                parts = line.split(":")
                if len(parts) >= 4:
                    ssid = parts[0] if parts[0] else None
                    security = parts[1] if parts[1] else "Open"
                    signal = parts[2] if parts[2] else "0"
                    freq = parts[3] if parts[3] else None
                    
                    if ssid:  # Only add networks with SSID
                        networks.append({
                            "ssid": ssid,
                            "security": security,
                            "signal": int(signal) if signal.isdigit() else 0,
                            "frequency": freq,
                        })
    except Exception:
        pass
    
    return networks

