# main_api.py

from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from api import router as api_router
from api.security_middleware import create_security_middleware_stack
from api.error_handlers import global_exception_handler, http_exception_handler
from api.ros_bridge import OmegaRosBridge
from api.sensor_bridge import OmegaSensorBridge
from api.control_path import registry as _ctrl_registry
import asyncio
import uvicorn
import logging
import os

logger = logging.getLogger(__name__)


_VIDEO_SERVER_URL = os.getenv("VIDEO_SERVER_URL", "http://127.0.0.1:5000")


def _push_mode_to_video_server() -> None:
    """Push current mode from system_state to video_server on FastAPI startup."""
    import json as _json
    import urllib.request as _urllib_req
    import time as _time
    try:
        from video.system_state import get_system_state
        mode = get_system_state().get_current_mode()
    except Exception as exc:
        logger.warning("[STARTUP_SYNC] Could not read system_state mode: %s", exc)
        return
    body = _json.dumps({"mode": mode}).encode()
    for attempt in range(1, 4):
        try:
            req = _urllib_req.Request(
                f"{_VIDEO_SERVER_URL}/mode/set",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            _urllib_req.urlopen(req, timeout=2)
            logger.info("[STARTUP_SYNC] Pushed mode=%d to video_server", mode)
            return
        except Exception as exc:
            logger.warning("[STARTUP_SYNC] Attempt %d failed: %s", attempt, exc)
            if attempt < 3:
                _time.sleep(1)
    logger.warning("[STARTUP_SYNC] Could not reach video_server after 3 attempts — will sync on first mode-set")


@asynccontextmanager
async def lifespan(app: FastAPI):
    # ROS publish bridge (movement, lighting, system commands)
    bridge = OmegaRosBridge.create()
    app.state.ros_bridge = bridge
    logger.info('ROS bridge active=%s', bridge.is_active)

    # [CTRL_PATH] motor path registration (Step 2 — Phase 1)
    _motor_path = (
        'both'   if bridge.is_active and bridge.motor_driver_ok else
        'ros2'   if bridge.is_active else
        'direct' if bridge.motor_driver_ok else
        'none'
    )
    _ctrl_registry.register('motor', _motor_path)
    logger.info('[CTRL_PATH] motor → %s', _motor_path)

    # ROS subscribe bridge (ultrasonic, line tracking → WebSocket fan-out)
    loop = asyncio.get_event_loop()
    sensor_bridge = OmegaSensorBridge.create(loop)
    app.state.sensor_bridge = sensor_bridge
    logger.info('Sensor bridge active=%s', sensor_bridge.is_active)

    # [CTRL_PATH] sensor + autonomy path registration
    _sensor_path = 'ros2' if sensor_bridge.is_active else 'direct'
    _ctrl_registry.register('sensor', _sensor_path)
    logger.info('[CTRL_PATH] sensor → %s', _sensor_path)

    _ctrl_registry.register('autonomy', 'fastapi')
    logger.info('[CTRL_PATH] autonomy → fastapi')

    # Push current mode to video_server so both processes stay in sync after restarts
    loop.run_in_executor(None, _push_mode_to_video_server)

    # Start localization prediction loop (dead reckoning + ArUco EKF)
    from api.localization_routes import start_prediction_loop, stop_prediction_loop
    start_prediction_loop()
    logger.info('[LOCALIZATION] SE(2) EKF prediction loop started')

    # Simulation engine (only when SIM_MODE=1)
    _sim_enabled = os.getenv("SIM_MODE", "0").strip() == "1"
    if _sim_enabled:
        logger.info('[SIM] SIM_MODE=1 — simulation engine active')

    yield

    stop_prediction_loop()
    bridge.shutdown()
    sensor_bridge.shutdown()

    if _sim_enabled:
        from api.sim_routes import shutdown_sim
        shutdown_sim()
        logger.info('[SIM] Simulation engine shut down')


app = FastAPI(title="Omega Robot Controller API", lifespan=lifespan)

# Load security configuration from config manager (if available)
try:
    from omega_config.config_manager import get_config_manager
    config_manager = get_config_manager()
    config = config_manager.get_config()
    
    security_config = config.get("security", {})
    allowed_origins = security_config.get("allowed_origins", [])
    api_auth_enabled = security_config.get("api_auth_enabled", False)
    api_key = security_config.get("api_key", "") or os.getenv("API_KEY", "")
    rate_limit_enabled = security_config.get("rate_limit_enabled", True)
    requests_per_minute = security_config.get("requests_per_minute", 60)
    csrf_enabled = security_config.get("csrf_enabled", False)
    request_size_limit = security_config.get("request_size_limit_mb", 10) * 1024 * 1024
    audit_logging = security_config.get("audit_logging", True)
    
    # Never log API keys or secrets
    logger.info(f"🔐 Security config loaded: auth={'enabled' if api_auth_enabled else 'disabled'}, "
                f"rate_limit={'enabled' if rate_limit_enabled else 'disabled'}, "
                f"csrf={'enabled' if csrf_enabled else 'disabled'}")
except Exception as e:
    logger.warning(f"Failed to load security config: {e}, using defaults")
    # Fallback to environment variables or defaults
    allowed_origins = os.getenv("ALLOWED_ORIGINS", "").split(",") if os.getenv("ALLOWED_ORIGINS") else []
    api_auth_enabled = os.getenv("API_AUTH_ENABLED", "false").lower() == "true"
    api_key = os.getenv("API_KEY", "")
    rate_limit_enabled = True
    requests_per_minute = 60
    csrf_enabled = os.getenv("CSRF_ENABLED", "false").lower() == "true"
    request_size_limit = int(os.getenv("REQUEST_SIZE_LIMIT_MB", "10")) * 1024 * 1024
    audit_logging = os.getenv("AUDIT_LOGGING", "true").lower() == "true"

# Create security middleware stack
create_security_middleware_stack(
    app,
    allowed_origins=allowed_origins,
    api_key=api_key,
    api_auth_enabled=api_auth_enabled,
    rate_limit_enabled=rate_limit_enabled,
    requests_per_minute=requests_per_minute,
    csrf_enabled=csrf_enabled,
    request_size_limit=request_size_limit,
    audit_logging=audit_logging
)

# Include your modular routes
app.include_router(api_router)

@app.get("/health")
async def health():
    return "ok"

# Add global exception handlers for secure error handling
from api.error_handlers import global_exception_handler, http_exception_handler
app.add_exception_handler(Exception, global_exception_handler)
app.add_exception_handler(HTTPException, http_exception_handler)

if __name__ == "__main__":
    uvicorn.run("main_api:app", host="0.0.0.0", port=8000, reload=True)

