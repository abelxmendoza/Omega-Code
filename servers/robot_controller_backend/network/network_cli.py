#!/usr/bin/env python3
"""
Omega Network CLI Wrapper
==========================

Simple command-line wrapper for network wizard.
Allows commands like:
    sudo omega-network ap
    sudo omega-network client
    sudo omega-network status
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from network_wizard import main

if __name__ == "__main__":
    main()

