# api/lighting_routes.py

from fastapi import APIRouter
import subprocess

router = APIRouter()

@router.get("/light/on")
def turn_light_on():
    # Replace this with the actual command or function
    try:
        subprocess.run(["python3", "controllers/lighting/led_control.py", "--on"], check=True)
        return {"status": "success", "message": "Light turned on"}
    except subprocess.CalledProcessError:
        return {"status": "error", "message": "Failed to turn on light"}

@router.get("/light/off")
def turn_light_off():
    # Replace this with the actual command or function
    try:
        subprocess.run(["python3", "controllers/lighting/led_control.py", "--off"], check=True)
        return {"status": "success", "message": "Light turned off"}
    except subprocess.CalledProcessError:
        return {"status": "error", "message": "Failed to turn off light"}

