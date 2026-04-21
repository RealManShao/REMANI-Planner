"""
Configuration settings for REMANI API
"""

from pydantic import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    """Application settings"""
    
    # API settings
    api_title: str = "REMANI Kinematic API"
    api_description: str = "Industrial Robot Kinematic Model Management API"
    api_version: str = "1.0.0"
    api_prefix: str = "/api/v1"
    
    # Server settings
    host: str = "0.0.0.0"
    port: int = 8000
    debug: bool = False
    
    # JWT settings
    jwt_secret: str = "your-secret-key-change-in-production"
    jwt_algorithm: str = "HS256"
    jwt_expiration_hours: int = 24
    
    # Rate limiting
    max_requests_per_minute: int = 100
    
    # Logging
    log_level: str = "INFO"
    log_file: Optional[str] = None
    
    # Model storage (future)
    models_storage_path: str = "./models"
    instances_storage_path: str = "./instances"
    
    class Config:
        env_file = ".env"
        case_sensitive = False

# Global settings instance
settings = Settings()
