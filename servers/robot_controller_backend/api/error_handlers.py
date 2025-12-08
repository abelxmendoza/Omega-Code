"""
Error Handling Utilities

Provides secure error handling that doesn't leak sensitive information
while maintaining useful error messages for accessibility.
"""

from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from typing import Any, Dict
import logging
import traceback

logger = logging.getLogger(__name__)


class SecureHTTPException(HTTPException):
    """HTTP Exception that doesn't leak sensitive information"""
    
    def __init__(
        self,
        status_code: int,
        detail: str,
        internal_error: str = None,
        log_error: bool = True
    ):
        super().__init__(status_code=status_code, detail=detail)
        self.internal_error = internal_error
        self.log_error = log_error
        
        if log_error and internal_error:
            logger.error(f"Internal error: {internal_error}")


def create_error_response(
    status_code: int,
    message: str,
    error_code: str = None,
    details: Dict[str, Any] = None
) -> JSONResponse:
    """
    Create a secure error response.
    
    Args:
        status_code: HTTP status code
        message: User-friendly error message (accessible)
        error_code: Machine-readable error code
        details: Additional details (sanitized)
    
    Returns:
        JSONResponse with error information
    """
    response_data = {
        "ok": False,
        "error": message,
    }
    
    if error_code:
        response_data["error_code"] = error_code
    
    if details:
        # Sanitize details to prevent information leakage
        sanitized_details = {}
        for key, value in details.items():
            # Only include safe, non-sensitive information
            if isinstance(value, (str, int, float, bool)):
                sanitized_details[key] = value
        response_data["details"] = sanitized_details
    
    return JSONResponse(
        status_code=status_code,
        content=response_data
    )


async def global_exception_handler(request: Request, exc: Exception) -> JSONResponse:
    """
    Global exception handler that prevents information disclosure.
    
    Args:
        request: FastAPI request
        exc: Exception that was raised
    
    Returns:
        Secure error response
    """
    # Log full error details server-side
    logger.error(
        f"Unhandled exception: {exc.__class__.__name__}",
        exc_info=exc,
        extra={"path": request.url.path, "method": request.method}
    )
    
    # Return generic error to client (don't leak stack traces)
    return create_error_response(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        message="An internal error occurred. Please try again later.",
        error_code="INTERNAL_ERROR"
    )


async def http_exception_handler(request: Request, exc: HTTPException) -> JSONResponse:
    """
    HTTP exception handler with secure error messages.
    
    Args:
        request: FastAPI request
        exc: HTTPException that was raised
    
    Returns:
        Secure error response
    """
    # Use the detail from HTTPException (already sanitized)
    return create_error_response(
        status_code=exc.status_code,
        message=exc.detail,
        error_code=f"HTTP_{exc.status_code}"
    )


def sanitize_error_message(error: Exception) -> str:
    """
    Sanitize error messages to prevent information disclosure.
    
    Args:
        error: Exception object
    
    Returns:
        Safe error message for client
    """
    error_type = error.__class__.__name__
    
    # Map common errors to user-friendly messages
    error_messages = {
        "ValueError": "Invalid input provided",
        "TypeError": "Invalid data type",
        "KeyError": "Required field missing",
        "AttributeError": "Invalid operation",
        "PermissionError": "Access denied",
        "FileNotFoundError": "Resource not found",
        "ConnectionError": "Connection failed",
        "TimeoutError": "Request timed out",
    }
    
    # Return user-friendly message or generic error
    return error_messages.get(error_type, "An error occurred")


def log_error_safely(error: Exception, context: str = None):
    """
    Log error with full details server-side, but don't expose to client.
    
    Args:
        error: Exception to log
        context: Additional context
    """
    logger.error(
        f"Error in {context or 'unknown context'}: {error.__class__.__name__}",
        exc_info=error,
        extra={"error_type": error.__class__.__name__}
    )

