# api/lighting_routes.py
"""
Lighting API Routes - FastAPI endpoints for LED control

This module provides REST API endpoints that delegate to the proper lighting controller.
For full lighting control, use controllers/lighting/lighting_routes.py which provides
more comprehensive endpoints with pattern/mode/brightness support.
"""

from fastapi import APIRouter, HTTPException
from controllers.lighting.led_control import LedController
from rpi_ws281x import Color

router = APIRouter()

# Persistent LED controller instance
try:
    led_controller = LedController()
except Exception as e:
    led_controller = None
    print(f"⚠️ [WARN] LED controller not available: {e}")

@router.get("/light/on")
def turn_light_on():
    """Turn LEDs on with default white color"""
    if led_controller is None:
        raise HTTPException(status_code=503, detail="LED controller not available")
    try:
        led_controller.color_wipe(Color(255, 255, 255), wait_ms=10)
        return {"status": "success", "message": "Light turned on"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to turn on light: {str(e)}")

@router.get("/light/off")
def turn_light_off():
    """Turn LEDs off"""
    if led_controller is None:
        raise HTTPException(status_code=503, detail="LED controller not available")
    try:
        led_controller.clear_strip()
        return {"status": "success", "message": "Light turned off"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to turn off light: {str(e)}")

