"""
Global exception handlers for FastAPI
"""

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from pydantic import ValidationError
from typing import Union, Dict, Any
import logging
from datetime import datetime

logger = logging.getLogger(__name__)

class ModelNotFoundError(Exception):
    """Model not found error"""
    pass

class InstanceNotFoundError(Exception):
    """Instance not found error"""
    pass

class DuplicateModelError(Exception):
    """Duplicate model error"""
    pass

class DuplicateInstanceError(Exception):
    """Duplicate instance error"""
    pass

class InvalidParameterError(Exception):
    """Invalid parameter error"""
    pass

class InstanceInMotionError(Exception):
    """Instance in motion error"""
    pass

class CppException(Exception):
    """C++ core exception wrapper"""
    def __init__(self, message: str, code: str):
        self.message = message
        self.code = code
        super().__init__(message)

def create_error_response(
    status_code: int,
    message: str,
    error_code: str = "UNKNOWN_ERROR",
    details: Dict[str, Any] = None
) -> JSONResponse:
    """Create standardized error response"""
    error_data = {
        "error": {
            "code": error_code,
            "message": message
        },
        "timestamp": datetime.utcnow().isoformat()
    }
    
    if details:
        error_data["error"]["details"] = details
    
    return JSONResponse(
        status_code=status_code,
        content=error_data
    )

def setup_exception_handlers(app: FastAPI):
    """Setup global exception handlers"""
    
    @app.exception_handler(ModelNotFoundError)
    async def model_not_found_handler(request: Request, exc: ModelNotFoundError):
        return create_error_response(
            status_code=404,
            message=str(exc),
            error_code="MODEL_NOT_FOUND"
        )
    
    @app.exception_handler(InstanceNotFoundError)
    async def instance_not_found_handler(request: Request, exc: InstanceNotFoundError):
        return create_error_response(
            status_code=404,
            message=str(exc),
            error_code="INSTANCE_NOT_FOUND"
        )
    
    @app.exception_handler(DuplicateModelError)
    async def duplicate_model_handler(request: Request, exc: DuplicateModelError):
        return create_error_response(
            status_code=409,
            message=str(exc),
            error_code="DUPLICATE_MODEL"
        )
    
    @app.exception_handler(DuplicateInstanceError)
    async def duplicate_instance_handler(request: Request, exc: DuplicateInstanceError):
        return create_error_response(
            status_code=409,
            message=str(exc),
            error_code="DUPLICATE_INSTANCE"
        )
    
    @app.exception_handler(InvalidParameterError)
    async def invalid_parameter_handler(request: Request, exc: InvalidParameterError):
        return create_error_response(
            status_code=400,
            message=str(exc),
            error_code="INVALID_PARAMETER"
        )
    
    @app.exception_handler(InstanceInMotionError)
    async def instance_in_motion_handler(request: Request, exc: InstanceInMotionError):
        return create_error_response(
            status_code=409,
            message=str(exc),
            error_code="INSTANCE_IN_MOTION"
        )
    
    @app.exception_handler(CppException)
    async def cpp_exception_handler(request: Request, exc: CppException):
        return create_error_response(
            status_code=500,
            message=exc.message,
            error_code=exc.code
        )
    
    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        return create_error_response(
            status_code=422,
            message="Validation failed",
            error_code="VALIDATION_ERROR",
            details=exc.errors()
        )
    
    @app.exception_handler(ValidationError)
    async def pydantic_validation_handler(request: Request, exc: ValidationError):
        return create_error_response(
            status_code=422,
            message="Pydantic validation failed",
            error_code="VALIDATION_ERROR",
            details=exc.errors()
        )
    
    @app.exception_handler(StarletteHTTPException)
    async def http_exception_handler(request: Request, exc: StarletteHTTPException):
        return create_error_response(
            status_code=exc.status_code,
            message=exc.detail,
            error_code="HTTP_ERROR"
        )
    
    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception):
        logger.error(f"Unhandled exception: {type(exc).__name__}: {exc}")
        return create_error_response(
            status_code=500,
            message="Internal server error",
            error_code="INTERNAL_ERROR"
        )
