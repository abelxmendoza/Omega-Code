# File: /Omega-Code/servers/robot-controller-backend/controllers/lighting/lighting_routes.py

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import controllers.lighting.led_control as led_control  # import your LED logic

router = APIRouter()

class ColorRequest(BaseModel):
    color: str  # Hex string, like "#ff0000"

@router.post("/lighting/color")
def set_color(request: ColorRequest):
    try:
        # This is where you call your actual LED control function
        led_control.set_led_color(request.color)
        return {"status": "success", "color": request.color}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
