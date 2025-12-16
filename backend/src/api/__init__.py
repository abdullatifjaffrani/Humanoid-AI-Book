from fastapi import APIRouter
from . import chat

# Create the main API router
api_router = APIRouter()

# Include all API routes
api_router.include_router(chat.router, tags=["chat"])


def include_routers():
    """
    Function to include all API routes
    """
    # This function can be used to register all routes in the main app
    pass