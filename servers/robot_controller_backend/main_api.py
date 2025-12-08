# main_api.py

from fastapi import FastAPI, HTTPException
from api import router as api_router
from api.security_middleware import create_security_middleware_stack
from api.error_handlers import global_exception_handler, http_exception_handler
import uvicorn
import logging
import os

logger = logging.getLogger(__name__)

app = FastAPI(title="Omega Robot Controller API")

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

# Add global exception handlers for secure error handling
from api.error_handlers import global_exception_handler, http_exception_handler
app.add_exception_handler(Exception, global_exception_handler)
app.add_exception_handler(HTTPException, http_exception_handler)

if __name__ == "__main__":
    uvicorn.run("main_api:app", host="0.0.0.0", port=8000, reload=True)

