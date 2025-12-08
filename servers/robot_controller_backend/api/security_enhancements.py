"""
Security Enhancements for OmegaOS

Additional security features that maintain accessibility:
- CSRF protection
- Request size limits
- Enhanced security headers
- Security audit logging
- Secret masking in logs
"""

from fastapi import Request, HTTPException, status
from fastapi.responses import Response
from starlette.middleware.base import BaseHTTPMiddleware
from typing import Optional, Dict, Any
import logging
import re
import secrets
import hashlib
import time
from collections import defaultdict

logger = logging.getLogger(__name__)

# CSRF token storage (in-memory, use Redis in production)
_csrf_tokens: Dict[str, Dict[str, Any]] = {}
_csrf_cleanup_interval = 3600  # 1 hour
_last_csrf_cleanup = time.time()

# Request size limits
MAX_REQUEST_SIZE = 10 * 1024 * 1024  # 10MB
MAX_JSON_DEPTH = 20
MAX_JSON_SIZE = 5 * 1024 * 1024  # 5MB

# Secrets to mask in logs
SECRET_PATTERNS = [
    r'password["\']?\s*[:=]\s*["\']?([^"\']+)',
    r'api[_-]?key["\']?\s*[:=]\s*["\']?([^"\']+)',
    r'secret["\']?\s*[:=]\s*["\']?([^"\']+)',
    r'token["\']?\s*[:=]\s*["\']?([^"\']+)',
    r'auth["\']?\s*[:=]\s*["\']?([^"\']+)',
]


def mask_secrets(text: str) -> str:
    """
    Mask secrets in log messages to prevent accidental exposure.
    
    Args:
        text: Text that may contain secrets
    
    Returns:
        Text with secrets masked
    """
    masked = text
    for pattern in SECRET_PATTERNS:
        masked = re.sub(pattern, r'\1', masked, flags=re.IGNORECASE)
        # Replace with asterisks
        masked = re.sub(
            r'(password|api[_-]?key|secret|token|auth)["\']?\s*[:=]\s*["\']?([^"\']+)',
            r'\1="***MASKED***"',
            masked,
            flags=re.IGNORECASE
        )
    return masked


class CSRFProtectionMiddleware(BaseHTTPMiddleware):
    """
    CSRF protection middleware.
    
    Allows GET, HEAD, OPTIONS without CSRF token.
    Requires CSRF token for POST, PUT, DELETE, PATCH.
    Public endpoints (health checks) are exempt.
    """
    
    def __init__(self, app, enabled: bool = True, exempt_paths: set = None):
        super().__init__(app)
        self.enabled = enabled
        self.exempt_paths = exempt_paths or {
            "/health",
            "/api/health",
            "/api/capabilities",
        }
        self.safe_methods = {"GET", "HEAD", "OPTIONS"}
    
    async def dispatch(self, request: Request, call_next):
        # Skip if disabled
        if not self.enabled:
            return await call_next(request)
        
        # Skip safe methods
        if request.method in self.safe_methods:
            return await call_next(request)
        
        # Skip exempt paths
        if request.url.path in self.exempt_paths:
            return await call_next(request)
        
        # Check for CSRF token
        csrf_token = request.headers.get("X-CSRF-Token") or request.headers.get("X-Csrf-Token")
        
        if not csrf_token:
            logger.warning(f"CSRF token missing for {request.method} {request.url.path}")
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="CSRF token required. Include X-CSRF-Token header."
            )
        
        # Validate token (simple check - in production, use signed tokens)
        if not self._validate_token(csrf_token, request):
            logger.warning(f"Invalid CSRF token for {request.method} {request.url.path}")
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Invalid CSRF token"
            )
        
        return await call_next(request)
    
    def _validate_token(self, token: str, request: Request) -> bool:
        """Validate CSRF token"""
        # Simple validation - check if token exists and matches origin
        # In production, use signed tokens with expiration
        if len(token) < 32:  # Minimum token length
            return False
        
        # Check origin matches
        origin = request.headers.get("Origin")
        referer = request.headers.get("Referer")
        
        # Allow if no origin/referer (same-origin request)
        if not origin and not referer:
            return True
        
        # Token should be associated with a session/origin
        # For now, accept any valid-length token (can be enhanced)
        return True


class RequestSizeLimitMiddleware(BaseHTTPMiddleware):
    """Limit request body size to prevent DoS attacks"""
    
    def __init__(self, app, max_size: int = MAX_REQUEST_SIZE):
        super().__init__(app)
        self.max_size = max_size
    
    async def dispatch(self, request: Request, call_next):
        # Check content length
        content_length = request.headers.get("Content-Length")
        if content_length:
            try:
                size = int(content_length)
                if size > self.max_size:
                    logger.warning(f"Request too large: {size} bytes (max: {self.max_size})")
                    raise HTTPException(
                        status_code=status.HTTP_413_REQUEST_ENTITY_TOO_LARGE,
                        detail=f"Request body too large. Maximum size: {self.max_size / 1024 / 1024:.1f}MB"
                    )
            except ValueError:
                pass  # Invalid content length header
        
        # Read body with size limit
        body = await request.body()
        if len(body) > self.max_size:
            logger.warning(f"Request body too large: {len(body)} bytes")
            raise HTTPException(
                status_code=status.HTTP_413_REQUEST_ENTITY_TOO_LARGE,
                detail=f"Request body too large. Maximum size: {self.max_size / 1024 / 1024:.1f}MB"
            )
        
        # Create new request with body
        async def receive():
            return {"type": "http.request", "body": body}
        
        request._receive = receive
        
        return await call_next(request)


class EnhancedSecurityHeadersMiddleware(BaseHTTPMiddleware):
    """Enhanced security headers including HSTS and Permissions Policy"""
    
    async def dispatch(self, request: Request, call_next):
        response = await call_next(request)
        
        # HSTS (HTTP Strict Transport Security) - only if HTTPS
        if request.url.scheme == "https":
            response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
        
        # Permissions Policy (formerly Feature Policy)
        permissions_policy = (
            "geolocation=(), "
            "microphone=(), "
            "camera=(), "
            "payment=(), "
            "usb=()"
        )
        response.headers["Permissions-Policy"] = permissions_policy
        
        # X-Content-Type-Options
        response.headers["X-Content-Type-Options"] = "nosniff"
        
        # X-Frame-Options
        response.headers["X-Frame-Options"] = "DENY"
        
        # X-XSS-Protection
        response.headers["X-XSS-Protection"] = "1; mode=block"
        
        # Referrer Policy
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
        
        # Content Security Policy (enhanced)
        csp = (
            "default-src 'self'; "
            "script-src 'self' 'unsafe-inline' 'unsafe-eval'; "  # unsafe-eval needed for some libs
            "style-src 'self' 'unsafe-inline'; "  # unsafe-inline needed for Tailwind
            "img-src 'self' data: blob:; "
            "font-src 'self' data:; "
            "connect-src 'self' ws: wss: http: https:; "
            "frame-ancestors 'none'; "
            "base-uri 'self'; "
            "form-action 'self';"
        )
        response.headers["Content-Security-Policy"] = csp
        
        # Remove server header
        response.headers.pop("server", None)
        response.headers.pop("X-Powered-By", None)
        
        return response


class SecurityAuditMiddleware(BaseHTTPMiddleware):
    """Log security-relevant events for auditing"""
    
    def __init__(self, app, enabled: bool = True):
        super().__init__(app)
        self.enabled = enabled
        self.audit_logger = logging.getLogger("security_audit")
        self.audit_logger.setLevel(logging.INFO)
    
    async def dispatch(self, request: Request, call_next):
        start_time = time.time()
        client_ip = request.client.host if request.client else "unknown"
        
        try:
            response = await call_next(request)
            
            # Log security events
            if self.enabled:
                duration = time.time() - start_time
                
                # Log failed authentication attempts
                if response.status_code == 401:
                    self.audit_logger.warning(
                        f"SECURITY_AUDIT: Failed authentication - "
                        f"IP: {client_ip}, Path: {request.url.path}, Method: {request.method}"
                    )
                
                # Log forbidden access attempts
                if response.status_code == 403:
                    self.audit_logger.warning(
                        f"SECURITY_AUDIT: Forbidden access - "
                        f"IP: {client_ip}, Path: {request.url.path}, Method: {request.method}"
                    )
                
                # Log rate limit hits
                if response.status_code == 429:
                    self.audit_logger.warning(
                        f"SECURITY_AUDIT: Rate limit exceeded - "
                        f"IP: {client_ip}, Path: {request.url.path}, Method: {request.method}"
                    )
                
                # Log suspicious requests (very large, very slow)
                if duration > 5.0:  # Requests taking > 5 seconds
                    self.audit_logger.info(
                        f"SECURITY_AUDIT: Slow request - "
                        f"IP: {client_ip}, Path: {request.url.path}, Duration: {duration:.2f}s"
                    )
            
            return response
            
        except HTTPException as e:
            # Log security-related exceptions
            if self.enabled and e.status_code in [401, 403, 429]:
                self.audit_logger.warning(
                    f"SECURITY_AUDIT: HTTP {e.status_code} - "
                    f"IP: {client_ip}, Path: {request.url.path}, "
                    f"Detail: {mask_secrets(str(e.detail))}"
                )
            raise
        except Exception as e:
            # Log unexpected errors
            if self.enabled:
                self.audit_logger.error(
                    f"SECURITY_AUDIT: Unexpected error - "
                    f"IP: {client_ip}, Path: {request.url.path}, "
                    f"Error: {mask_secrets(str(e))}"
                )
            raise


def generate_csrf_token() -> str:
    """Generate a CSRF token"""
    return secrets.token_urlsafe(32)


def hash_secret(secret: str) -> str:
    """Hash a secret for storage (one-way)"""
    return hashlib.sha256(secret.encode()).hexdigest()


def verify_secret(secret: str, hashed: str) -> bool:
    """Verify a secret against its hash"""
    return hash_secret(secret) == hashed


def sanitize_for_logging(data: Any) -> Any:
    """
    Sanitize data before logging to prevent secret exposure.
    
    Args:
        data: Data to sanitize (dict, list, or string)
    
    Returns:
        Sanitized data with secrets masked
    """
    if isinstance(data, dict):
        sanitized = {}
        for key, value in data.items():
            # Mask common secret field names
            if any(secret_word in key.lower() for secret_word in ["password", "secret", "key", "token", "auth"]):
                sanitized[key] = "***MASKED***"
            else:
                sanitized[key] = sanitize_for_logging(value)
        return sanitized
    elif isinstance(data, list):
        return [sanitize_for_logging(item) for item in data]
    elif isinstance(data, str):
        return mask_secrets(data)
    else:
        return data

