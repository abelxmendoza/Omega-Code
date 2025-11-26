"""
Lighting Control Routes (FastAPI)

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/lighting_routes.py

This module defines the FastAPI routes for controlling LED lighting on the robot.
It receives JSON payloads from the frontend (via WebSocket or REST), parses
the lighting configuration (color, mode, pattern, interval), and dispatches the command
to the appropriate LED pattern function using the rpi_ws281x driver.

Example Payload:
{
    "command": "SET_LED",
    "color": "#ff0000",
    "mode": "single",
    "pattern": "fade",
    "interval": 1000
}
"""

import sys
import traceback
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field, validator
from typing import Optional

from controllers.lighting.led_control import LedController
from controllers.lighting.dispatcher import apply_lighting_mode

# Initialize FastAPI router
router = APIRouter()

# Persistent LED controller instance (do NOT recreate for each request!)
try:
    led = LedController()
    print("✅ [INIT] LED controller initialized for FastAPI routes", file=sys.stderr)
except Exception as e:
    print(f"❌ [ERROR] Failed to initialize LED controller: {e}", file=sys.stderr)
    traceback.print_exc(file=sys.stderr)
    led = None  # Will fail gracefully on requests

class LightingRequest(BaseModel):
    command: str = Field(..., description="Command type (must be SET_LED)")
    color: str = Field(..., description="Hex color string, e.g. '#ff0000'")
    mode: str = Field(default="single", description="Lighting mode: single, dual, rainbow, etc.")
    pattern: str = Field(default="static", description="Pattern: static, fade, blink, chase, etc.")
    interval: Optional[int] = Field(default=1000, ge=0, le=60000, description="Interval in ms (0-60000)")
    brightness: Optional[float] = Field(default=1.0, ge=0.0, le=1.0, description="Brightness 0.0-1.0")
    
    @validator('color')
    def validate_color(cls, v):
        """Validate hex color format."""
        v = v.lstrip('#').upper()
        if len(v) != 6:
            raise ValueError(f"Color must be 6 hex digits, got: {len(v)}")
        try:
            int(v, 16)
        except ValueError:
            raise ValueError(f"Invalid hex color: {v}")
        return '#' + v
    
    @validator('command')
    def validate_command(cls, v):
        """Validate command."""
        if v.upper() != "SET_LED":
            raise ValueError(f"Command must be SET_LED, got: {v}")
        return v.upper()

@router.post("/lighting/control")
def set_lighting(request: LightingRequest):
    """
    Receives a lighting control payload and dispatches it to the LED controller.
    Comprehensive error handling with detailed messages.
    """
    if led is None:
        raise HTTPException(
            status_code=503,
            detail="LED controller not available. Check hardware initialization."
        )
    
    try:
        payload = request.dict()
        print(f"📨 [API] Received lighting command: {payload}", file=sys.stderr)
        
        # Dispatcher handles all parsing and validation
        apply_lighting_mode(payload, led)
        
        return {
            "status": "success",
            "message": "Lighting command executed successfully",
            "received": payload
        }
    except ValueError as e:
        print(f"❌ [ERROR] Invalid request: {e}", file=sys.stderr)
        raise HTTPException(status_code=400, detail=f"Invalid request: {str(e)}")
    except Exception as e:
        print(f"❌ [ERROR] Lighting command failed: {e}", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        raise HTTPException(status_code=500, detail=f"LED operation failed: {str(e)}")

# Optional: Explicit power control (On/Off)
class PowerRequest(BaseModel):
    power: bool  # True = ON, False = OFF

@router.post("/lighting/power")
def set_power(request: PowerRequest):
    """
    Turns LED strip power on or off with comprehensive error handling.
    """
    if led is None:
        raise HTTPException(
            status_code=503,
            detail="LED controller not available. Check hardware initialization."
        )
    
    try:
        if request.power:
            print(f"💡 [API] Turning LEDs ON", file=sys.stderr)
            from rpi_ws281x import Color
            led.color_wipe(Color(255, 255, 255))  # ON: set to white
        else:
            print(f"💡 [API] Turning LEDs OFF", file=sys.stderr)
            led.clear_strip()  # OFF: turn off all LEDs
        
        return {
            "status": "success",
            "message": f"LEDs turned {'ON' if request.power else 'OFF'}",
            "power": request.power
        }
    except Exception as e:
        print(f"❌ [ERROR] Power control failed: {e}", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        raise HTTPException(status_code=500, detail=f"Power control failed: {str(e)}")
