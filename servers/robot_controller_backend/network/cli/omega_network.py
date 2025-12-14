#!/usr/bin/env python3
"""
Omega Network CLI Tool
======================

Command-line interface for Omega-1 network management.

Usage:
    sudo omega-network ap          # Enable AP mode
    sudo omega-network client       # Enable client mode
    sudo omega-network status       # Show network status
    sudo omega-network validate     # Validate configuration
    sudo omega-network logs         # View logs
"""

import os
import sys
import subprocess
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from wizard.network_wizard import (
    enable_ap_mode,
    enable_client_mode,
    show_network_status,
    run_full_validation,
    check_root,
    log,
    LOG_FILE,
)


def show_logs(lines: int = 50):
    """Display recent log entries"""
    if not os.path.exists(LOG_FILE):
        print("No log file found.")
        return
    
    try:
        result = subprocess.run(
            ["tail", "-n", str(lines), LOG_FILE],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            print(result.stdout)
        else:
            # Fallback: read file directly
            with open(LOG_FILE, "r") as f:
                all_lines = f.readlines()
                for line in all_lines[-lines:]:
                    print(line, end="")
    except Exception as e:
        print(f"Error reading logs: {e}")


def main():
    """Main CLI entry point"""
    check_root()
    
    if len(sys.argv) < 2:
        print("Omega Network CLI Tool")
        print("=" * 60)
        print("Usage: sudo omega-network <command>")
        print("")
        print("Commands:")
        print("  ap          Enable AP mode (field mode)")
        print("  client      Enable client mode (home mode)")
        print("  status      Show network status")
        print("  validate    Validate network configuration")
        print("  logs        View recent log entries")
        print("  logs <n>    View last N log entries")
        sys.exit(1)
    
    command = sys.argv[1].lower()
    
    if command == "ap":
        enable_ap_mode()
    elif command == "client":
        enable_client_mode()
    elif command == "status":
        show_network_status()
    elif command == "validate":
        run_full_validation()
    elif command == "logs":
        lines = int(sys.argv[2]) if len(sys.argv) > 2 else 50
        show_logs(lines)
    else:
        log(f"Unknown command: {command}", "ERROR")
        print("Usage: sudo omega-network [ap|client|status|validate|logs]")
        sys.exit(1)


if __name__ == "__main__":
    main()

