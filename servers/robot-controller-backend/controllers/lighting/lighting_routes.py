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

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional

from controllers.lighting.led_control import LedController
from controllers.lighting.dispatcher import apply_lighting_mode

# Initialize FastAPI router
router = APIRouter()

# Persistent LED controller instance (do NOT recreate for each request!)
led = LedController()

class LightingRequest(BaseModel):
    command: str
    color: str                  # Always a hex string, e.g. "#ff0000"
    mode: str                   # "single", "rainbow", etc.
    pattern: str                # "static", "pulse", etc.
    interval: Optional[int] = None  # Optional (default in dispatcher)

@router.post("/lighting/control")
def set_lighting(request: LightingRequest):
    """
    Receives a lighting control payload and dispatches it to the LED controller.
    """
    try:
        payload = request.dict()
        # Only allow the supported command
        if payload.get("command", "").upper() != "SET_LED":
            raise HTTPException(status_code=400, detail="Unknown command")
        # Dispatcher handles all parsing, including color hex
        apply_lighting_mode(payload, led)
        return {"status": "success", "received": payload}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# Optional: Explicit power control (On/Off)
class PowerRequest(BaseModel):
    power: bool  # True = ON, False = OFF

@router.post("/lighting/power")
def set_power(request: PowerRequest):
    """
    Turns LED strip power on or off.
    """
    try:
        if request.power:
            # You can change this to any default color or animation you want
            led.color_wipe((255, 255, 255))  # ON: set to white
        else:
            led.clear_strip()  # OFF: turn off all LEDs
        return {"status": "success", "power": request.power}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
