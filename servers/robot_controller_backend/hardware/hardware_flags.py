"""
Hardware Flags Module

Centralized, cached hardware mode flags to avoid repeated os.getenv() calls.
This provides O(1) access after initial evaluation.
"""

import os

# Cached at module load time - evaluated once
SIM_MODE = os.getenv("SIM_MODE", "0") == "1" or os.getenv("ROBOT_SIM", "0") == "1"
TEST_MODE = os.getenv("TEST_MODE", "0") == "1"

def is_sim():
    """Check if simulation mode is enabled."""
    return SIM_MODE or TEST_MODE

