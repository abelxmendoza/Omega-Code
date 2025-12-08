"""
Security Middleware for OmegaOS API

Provides:
- Configurable CORS with origin allowlist
- Optional API key authentication
- Rate limiting
- Security headers
- Input sanitization helpers
"""

from fastapi import Request, HTTPException, status
from fastapi.responses import Response
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.base import BaseHTTPMiddleware
from typing import List, Optional, Set
import time
import logging
from collections import defaultdict
from functools import lru_cache

logger = logging.getLogger(__name__)

# Rate limiting storage (in-memory, use Redis in production)
_rate_limit_store: defaultdict = defaultdict(list)
_rate_limit_cleanup_interval = 300  # Clean up old entries every 5 minutes
_last_cleanup = time.time()


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """Add security headers to all responses"""
    
    async def dispatch(self, request: Request, call_next):
        response = await call_next(request)
        
        # Security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
        
        # Content Security Policy (CSP) - allow inline styles/scripts for accessibility
        # but prevent XSS attacks
        csp = (
            "default-src 'self'; "
            "script-src 'self' 'unsafe-inline' 'unsafe-eval'; "  # unsafe-eval needed for some libs
            "style-src 'self' 'unsafe-inline'; "  # unsafe-inline needed for Tailwind
            "img-src 'self' data: blob:; "
            "font-src 'self' data:; "
            "connect-src 'self' ws: wss: http: https:; "
            "frame-ancestors 'none'; "
            "base-uri 'self';"
        )
        response.headers["Content-Security-Policy"] = csp
        
        # Remove server header (don't leak server info)
        response.headers.pop("server", None)
        
        return response


class RateLimitMiddleware(BaseHTTPMiddleware):
    """Rate limiting middleware to prevent abuse"""
    
    def __init__(self, app, requests_per_minute: int = 60, burst: int = 10):
        super().__init__(app)
        self.requests_per_minute = requests_per_minute
        self.burst = burst
    
    async def dispatch(self, request: Request, call_next):
        # Skip rate limiting for health checks
        if request.url.path in ["/health", "/api/health"]:
            return await call_next(request)
        
        # Get client identifier (IP address)
        client_ip = request.client.host if request.client else "unknown"
        
        # Clean up old entries periodically
        global _last_cleanup
        if time.time() - _last_cleanup > _rate_limit_cleanup_interval:
            self._cleanup_old_entries()
            _last_cleanup = time.time()
        
        # Check rate limit
        now = time.time()
        window_start = now - 60  # 1 minute window
        
        # Get requests in current window
        requests = _rate_limit_store[client_ip]
        requests[:] = [req_time for req_time in requests if req_time > window_start]
        
        # Check if limit exceeded
        if len(requests) >= self.requests_per_minute:
            logger.warning(f"Rate limit exceeded for {client_ip}")
            return Response(
                content='{"error": "Rate limit exceeded. Please try again later."}',
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                media_type="application/json",
                headers={"Retry-After": "60"}
            )
        
        # Add current request
        requests.append(now)
        
        return await call_next(request)
    
    def _cleanup_old_entries(self):
        """Remove old rate limit entries"""
        now = time.time()
        window_start = now - 60
        for client_ip in list(_rate_limit_store.keys()):
            _rate_limit_store[client_ip] = [
                req_time for req_time in _rate_limit_store[client_ip]
                if req_time > window_start
            ]
            if not _rate_limit_store[client_ip]:
                del _rate_limit_store[client_ip]


class APIKeyAuthMiddleware(BaseHTTPMiddleware):
    """Optional API key authentication middleware"""
    
    def __init__(self, app, api_key: Optional[str] = None, enabled: bool = False):
        super().__init__(app)
        self.api_key = api_key
        self.enabled = enabled
        # Public endpoints that don't require auth (for accessibility)
        self.public_endpoints = {
            "/health",
            "/api/health",
            "/api/capabilities",
            "/api/system/mode/list",
            "/api/system/mode/status",
        }
    
    async def dispatch(self, request: Request, call_next):
        # Skip if auth not enabled
        if not self.enabled:
            return await call_next(request)
        
        # Skip public endpoints
        if request.url.path in self.public_endpoints:
            return await call_next(request)
        
        # Check for API key in header
        api_key_header = request.headers.get("X-API-Key") or request.headers.get("Authorization")
        
        if api_key_header:
            # Remove "Bearer " prefix if present
            if api_key_header.startswith("Bearer "):
                api_key_header = api_key_header[7:]
            
            if api_key_header == self.api_key:
                return await call_next(request)
        
        # No valid API key
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="API key required. Include X-API-Key header or Authorization: Bearer <key>"
        )


def create_cors_middleware(app, allowed_origins: List[str], allow_all: bool = False):
    """
    Create CORS middleware with configurable origins.
    
    Args:
        app: FastAPI application
        allowed_origins: List of allowed origin URLs
        allow_all: If True, allow all origins (dev mode only)
    """
    if allow_all:
        logger.warning("⚠️ CORS: Allowing all origins (dev mode)")
        origins = ["*"]
    else:
        origins = allowed_origins if allowed_origins else []
        if not origins:
            logger.warning("⚠️ CORS: No allowed origins configured, defaulting to localhost")
            origins = ["http://localhost:3000", "http://localhost:8080"]
    
    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "PATCH"],
        allow_headers=["*"],
        expose_headers=["X-Request-ID"],
        max_age=3600,
    )


def sanitize_input(value: str, max_length: int = 1000) -> str:
    """
    Sanitize user input to prevent injection attacks.
    
    Args:
        value: Input string to sanitize
        max_length: Maximum allowed length
    
    Returns:
        Sanitized string
    """
    if not isinstance(value, str):
        return str(value)[:max_length]
    
    # Remove null bytes
    value = value.replace("\x00", "")
    
    # Remove control characters (except newline, tab, carriage return)
    value = "".join(
        char for char in value
        if ord(char) >= 32 or char in "\n\r\t"
    )
    
    # Truncate to max length
    if len(value) > max_length:
        value = value[:max_length]
        logger.warning(f"Input truncated to {max_length} characters")
    
    return value


def sanitize_path(path: str) -> str:
    """
    Sanitize file paths to prevent path traversal attacks.
    
    Args:
        path: Path to sanitize
    
    Returns:
        Sanitized path
    
    Raises:
        ValueError: If path contains traversal attempts
    """
    # Normalize path
    normalized = path.replace("\\", "/")
    
    # Check for path traversal attempts
    if ".." in normalized or normalized.startswith("/"):
        raise ValueError("Invalid path: path traversal detected")
    
    # Remove any remaining dangerous characters
    dangerous_chars = ["<", ">", "|", "&", ";", "`", "$", "(", ")", "{", "}"]
    for char in dangerous_chars:
        if char in normalized:
            raise ValueError(f"Invalid path: dangerous character '{char}' detected")
    
    return normalized


def validate_json_depth(obj: dict, max_depth: int = 10, current_depth: int = 0) -> bool:
    """
    Validate JSON object depth to prevent JSON bomb attacks.
    
    Args:
        obj: JSON object to validate
        max_depth: Maximum allowed nesting depth
        current_depth: Current nesting depth
    
    Returns:
        True if valid, False otherwise
    """
    if current_depth > max_depth:
        return False
    
    if isinstance(obj, dict):
        return all(
            validate_json_depth(value, max_depth, current_depth + 1)
            for value in obj.values()
        )
    elif isinstance(obj, list):
        return all(
            validate_json_depth(item, max_depth, current_depth + 1)
            for item in obj
        )
    
    return True


def get_client_ip(request: Request) -> str:
    """Get client IP address from request, handling proxies"""
    # Check X-Forwarded-For header (from proxy)
    forwarded_for = request.headers.get("X-Forwarded-For")
    if forwarded_for:
        # Take first IP in chain
        return forwarded_for.split(",")[0].strip()
    
    # Check X-Real-IP header
    real_ip = request.headers.get("X-Real-IP")
    if real_ip:
        return real_ip
    
    # Fall back to direct client
    return request.client.host if request.client else "unknown"


def create_security_middleware_stack(
    app,
    allowed_origins: List[str] = None,
    api_key: Optional[str] = None,
    api_auth_enabled: bool = False,
    rate_limit_enabled: bool = True,
    requests_per_minute: int = 60,
    csrf_enabled: bool = False,
    request_size_limit: int = 10 * 1024 * 1024,  # 10MB
    audit_logging: bool = True
):
    """
    Create complete security middleware stack.
    
    Args:
        app: FastAPI application
        allowed_origins: List of allowed CORS origins
        api_key: API key for authentication (if enabled)
        api_auth_enabled: Whether to enable API key auth
        rate_limit_enabled: Whether to enable rate limiting
        requests_per_minute: Rate limit threshold
        csrf_enabled: Whether to enable CSRF protection
        request_size_limit: Maximum request body size in bytes
        audit_logging: Whether to enable security audit logging
    """
    # Import enhanced security modules
    try:
        from .security_enhancements import (
            EnhancedSecurityHeadersMiddleware,
            CSRFProtectionMiddleware,
            RequestSizeLimitMiddleware,
            SecurityAuditMiddleware
        )
        use_enhanced = True
    except ImportError:
        use_enhanced = False
        logger.warning("Enhanced security modules not available, using basic security")
    
    # 1. Enhanced security headers (first, so they're always applied)
    if use_enhanced:
        app.add_middleware(EnhancedSecurityHeadersMiddleware)
    else:
        app.add_middleware(SecurityHeadersMiddleware)
    
    # 2. Request size limits (prevent DoS)
    if use_enhanced:
        app.add_middleware(RequestSizeLimitMiddleware, max_size=request_size_limit)
    
    # 3. Security audit logging
    if use_enhanced and audit_logging:
        app.add_middleware(SecurityAuditMiddleware, enabled=True)
    
    # 4. Rate limiting (before auth, to prevent brute force)
    if rate_limit_enabled:
        app.add_middleware(RateLimitMiddleware, requests_per_minute=requests_per_minute)
    
    # 5. CSRF protection (before CORS for preflight)
    if use_enhanced and csrf_enabled:
        app.add_middleware(CSRFProtectionMiddleware, enabled=True)
        logger.info("🔐 CSRF protection enabled")
    
    # 6. CORS (before auth, for preflight requests)
    create_cors_middleware(
        app,
        allowed_origins=allowed_origins or [],
        allow_all=(not allowed_origins or len(allowed_origins) == 0)
    )
    
    # 7. API key authentication (last, so other middleware can handle public endpoints)
    if api_auth_enabled and api_key:
        app.add_middleware(APIKeyAuthMiddleware, api_key=api_key, enabled=True)
        logger.info("🔐 API key authentication enabled")
    else:
        logger.info("🔓 API key authentication disabled (public access)")

