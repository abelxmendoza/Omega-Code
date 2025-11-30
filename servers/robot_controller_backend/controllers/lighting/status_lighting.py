"""
Status-Aware Lighting for Omega1

Automatically adjusts lighting based on robot status:
- Battery level
- Movement state
- Error conditions
- Charging state
- Sensor status

This module provides intelligent lighting that responds to robot state.
"""

import sys
import os
from typing import Optional, Dict, Any

# Add parent directory to Python path if not already set
if 'PYTHONPATH' not in os.environ or 'robot_controller_backend' not in os.environ.get('PYTHONPATH', ''):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.abspath(os.path.join(script_dir, '../..'))
    if parent_dir not in sys.path:
        sys.path.insert(0, parent_dir)

from controllers.lighting.led_control import LedController
from controllers.lighting.patterns import status_indicator
from controllers.lighting.color_presets import get_theme, COLOR_PRESETS


class StatusLighting:
    """Manages status-aware lighting for Omega1 robot."""
    
    def __init__(self, led_controller: LedController):
        """
        Initialize status lighting manager.
        
        Args:
            led_controller: Instance of LedController.
        """
        self.led = led_controller
        self.current_status = "idle"
        self.last_battery_level = 100
        self.is_moving = False
        self.has_error = False
        self.is_charging = False
    
    def update_status(
        self,
        battery_level: Optional[int] = None,
        is_moving: Optional[bool] = None,
        has_error: Optional[bool] = None,
        is_charging: Optional[bool] = None,
        sensor_status: Optional[Dict[str, Any]] = None,
    ):
        """
        Update robot status and adjust lighting accordingly.
        
        Args:
            battery_level: Battery percentage (0-100).
            is_moving: Whether robot is currently moving.
            has_error: Whether robot has an error condition.
            is_charging: Whether robot is charging.
            sensor_status: Dictionary of sensor statuses.
        """
        # Update state
        if battery_level is not None:
            self.last_battery_level = battery_level
        if is_moving is not None:
            self.is_moving = is_moving
        if has_error is not None:
            self.has_error = has_error
        if is_charging is not None:
            self.is_charging = is_charging
        
        # Determine status priority (error > low battery > charging > moving > idle)
        if self.has_error:
            self.current_status = "error"
            self._apply_status_lighting("error")
        elif self.last_battery_level < 20:
            self.current_status = "low_battery"
            self._apply_status_lighting("low_battery")
        elif self.is_charging:
            self.current_status = "charging"
            self._apply_status_lighting("charging")
        elif self.is_moving:
            self.current_status = "moving"
            self._apply_status_lighting("moving")
        else:
            self.current_status = "idle"
            self._apply_status_lighting("idle")
    
    def _apply_status_lighting(self, status: str):
        """Apply lighting based on status."""
        try:
            theme = get_theme(f"{status}_standby" if status == "idle" else status)
            
            # Convert hex color to int
            color_hex = theme["color"].lstrip("#")
            color_int = int(color_hex, 16)
            
            # Apply theme
            self.led.set_led(
                color=color_int,
                mode="single",
                pattern=theme["pattern"],
                interval=theme["interval"],
                brightness=theme["brightness"],
            )
            
            print(f"✅ [STATUS] Applied {status} lighting: {theme['pattern']} pattern", file=sys.stderr)
        except Exception as e:
            print(f"❌ [ERROR] Failed to apply status lighting: {e}", file=sys.stderr)
    
    def get_status(self) -> str:
        """Get current status."""
        return self.current_status
    
    def manual_override(self, color: int, pattern: str, brightness: float = 1.0):
        """
        Manually override status lighting with custom settings.
        
        Args:
            color: 24-bit RGB color integer.
            pattern: Pattern name.
            brightness: Brightness (0.0-1.0).
        """
        try:
            self.led.set_led(
                color=color,
                mode="single",
                pattern=pattern,
                interval=500,
                brightness=brightness,
            )
            print(f"✅ [MANUAL] Override applied: {pattern}", file=sys.stderr)
        except Exception as e:
            print(f"❌ [ERROR] Manual override failed: {e}", file=sys.stderr)

