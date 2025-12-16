from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# Import using absolute paths from the backend directory
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))  # Add backend to path

from src.api import api_router, include_routers
from src.config.settings import settings


def create_app():
    """
    Create and configure the FastAPI application
    """
    app = FastAPI(
        title=settings.app_name,
        version=settings.app_version,
        debug=settings.DEBUG,
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, replace with specific origins
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include API routes
    include_routers()

    # Mount the main API router
    app.include_router(api_router, prefix=settings.API_V1_STR)

    @app.get("/")
    def read_root():
        return {"message": "RAG Chatbot for Physical AI & Humanoid Robotics Textbook API"}

    @app.get("/health")
    def health_check():
        return {"status": "healthy", "version": settings.app_version}

    return app


# Create the main FastAPI application instance
app = create_app()

# If running this file directly, start the server
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        reload=True if settings.debug else False
    )