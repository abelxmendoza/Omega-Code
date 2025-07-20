"""
Lighting Control Routes (FastAPI)

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/lighting_routes.py

This module defines the FastAPI route for controlling LED lighting on the robot.
It receives JSON payloads from the frontend (via WebSocket or REST bridge), parses
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
from controllers.lighting.led_control import LedController
from controllers.lighting.dispatcher import apply_lighting_mode

# Initialize FastAPI router
router = APIRouter()

# Create one persistent LED controller instance
led = LedController()

# Define expected payload from UI
class LightingRequest(BaseModel):
    command: str
    color: str
    mode: str
    pattern: str
    interval: int | None = None  # Optional for static patterns

# Route to handle lighting control
@router.post("/lighting/control")
def set_lighting(request: LightingRequest):
    try:
        payload = request.dict()
        apply_lighting_mode(payload, led)
        return {"status": "success", "received": payload}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
class PowerRequest(BaseModel):
    power: bool  # True = ON, False = OFF

@router.post("/lighting/power")
def set_power(request: PowerRequest):
    try:
        led_control.toggle_led_power(request.power)
        return {"status": "success", "power": request.power}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
