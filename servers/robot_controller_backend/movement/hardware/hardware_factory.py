"""
Hardware Factory - Auto-Detect Platform and Return Appropriate Driver

Automatically detects the platform and returns the correct motor driver.
"""

import platform
import os
import logging

from .motor_driver_pi import PiMotorDriver
from .motor_driver_orin import OrinMotorDriver
from .motor_driver_mac import MacMotorDriver
from .motor_driver_linux import LinuxMotorDriver
from .motor_driver_sim import SimMotorDriver
from .motor_driver_noop import NoopMotorDriver
from .base_motor_driver import BaseMotorDriver

logger = logging.getLogger(__name__)


def get_motor_driver(trim_left: int = 0, trim_right: int = 0) -> BaseMotorDriver:
    """
    Auto-detect platform and return appropriate motor driver.
    
    Detection order:
    1. ROBOT_SIM=1 → SimMotorDriver
    2. Raspberry Pi → PiMotorDriver
    3. Jetson Orin Nano → OrinMotorDriver
    4. macOS → MacMotorDriver
    5. Linux → LinuxMotorDriver
    6. Fallback → NoopMotorDriver
    
    Args:
        trim_left: Trim offset for left motors
        trim_right: Trim offset for right motors
    
    Returns:
        BaseMotorDriver instance appropriate for the platform
    """
    # Check for explicit simulation mode
    if os.getenv("ROBOT_SIM") == "1" or os.getenv("SIM_MODE") == "1":
        logger.info("[HW_FACTORY] ROBOT_SIM=1 → Using SimMotorDriver")
        return SimMotorDriver(trim_left, trim_right)
    
    system = platform.system()
    machine = platform.machine()
    
    # Raspberry Pi detection
    try:
        if os.path.exists("/sys/firmware/devicetree/base/model"):
            with open("/sys/firmware/devicetree/base/model", "r") as f:
                model = f.read().lower()
                if "raspberry pi" in model:
                    logger.info(f"[HW_FACTORY] Detected Raspberry Pi: {model.strip()}")
                    try:
                        return PiMotorDriver(trim_left, trim_right)
                    except Exception as e:
                        logger.warning(f"[HW_FACTORY] PiMotorDriver failed: {e}, falling back")
                        print(f"🔥 [HW_FACTORY][ERROR] Failed to initialize PiMotorDriver")
                        print(f"   Hardware type: Raspberry Pi")
                        print(f"   Exception: {repr(e)}")
                        print(f"   Type: {type(e).__name__}")
                        # Try to detect I2C devices
                        try:
                            result = subprocess.run(['i2cdetect', '-y', '1'], 
                                                   capture_output=True, text=True, timeout=2)
                            print(f"   I2C devices detected:\n{result.stdout}")
                        except Exception:
                            print("   Could not run i2cdetect (may need sudo or i2c-tools)")
                        raise
    except Exception as e:
        logger.debug(f"[HW_FACTORY] Pi detection failed: {e}")
    
    # Jetson Orin Nano detection
    try:
        if os.path.isfile("/etc/nv_tegra_release") or "tegra" in machine.lower():
            logger.info(f"[HW_FACTORY] Detected Jetson platform: {machine}")
            try:
                return OrinMotorDriver(trim_left, trim_right)
            except Exception as e:
                logger.warning(f"[HW_FACTORY] OrinMotorDriver failed: {e}, falling back")
    except Exception as e:
        logger.debug(f"[HW_FACTORY] Jetson detection failed: {e}")
    
    # macOS detection
    if system == "Darwin":
        logger.info("[HW_FACTORY] Detected macOS → Using MacMotorDriver")
        return MacMotorDriver(trim_left, trim_right)
    
    # Linux detection
    if system == "Linux":
        logger.info("[HW_FACTORY] Detected Linux → Using LinuxMotorDriver")
        return LinuxMotorDriver(trim_left, trim_right)
    
    # Fallback to NOOP
    logger.warning(f"[HW_FACTORY] Unknown platform ({system}/{machine}) → Using NoopMotorDriver")
    return NoopMotorDriver(trim_left, trim_right)

