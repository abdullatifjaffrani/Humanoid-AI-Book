from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    app_name: str = "RAG Chatbot for Physical AI & Humanoid Robotics Textbook"
    app_version: str = "1.0.0"
    DEBUG: bool = True
    API_V1_STR: str = "/api/v1"
    
    # Qdrant settings
    QDRANT_URL: Optional[str] = None
    QDRANT_API_KEY: Optional[str] = None
    QDRANT_COLLECTION_NAME: str = "textbook_content"
    
    # Google Gemini settings
    GEMINI_API_KEY: Optional[str] = None
    GEMINI_MODEL: str = "gemini-2.5-flash-lite"  # Default model
    
    # Database settings
    DATABASE_URL: Optional[str] = None
    
    # Similarity threshold for retrieval
    SIMILARITY_THRESHOLD: float = 0.5
    
    class Config:
        env_file = ".env"


settings = Settings()