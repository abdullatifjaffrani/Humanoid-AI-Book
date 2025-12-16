from fastapi import APIRouter, HTTPException, Depends
from uuid import UUID, uuid4
from typing import Optional
import datetime
from ..models.query import QueryRequest, QueryModel
from ..database.engine import get_db, SessionLocal
from ..services.rag_service import rag_service
from ..database.session import session_db_service
from ..config.constants import HTTP_422_UNPROCESSABLE_ENTITY, ERROR_QUERY_OUTSIDE_SCOPE
from ..config.settings import settings


router = APIRouter()


@router.post("")
async def chat_endpoint(query_request: QueryRequest):
    """
    Chat endpoint that processes user queries and returns grounded responses with citations
    """
    try:
        db = SessionLocal()

        # Validate the query length
        if len(query_request.query) < 1 or len(query_request.query) > 1000:
            raise HTTPException(
                status_code=422,
                detail="Query must be between 1 and 1000 characters"
            )

        # Get or create session
        if query_request.session_id:
            try:
                session_uuid = UUID(query_request.session_id)
            except ValueError:
                # If session_id is not a valid UUID, generate a new one
                session_uuid = uuid4()
        else:
            session_uuid = uuid4()

        # Create or retrieve session from database
        db_session = session_db_service.get_session(db, session_uuid)
        if not db_session:
            db_session = session_db_service.create_session(db, None)  # No user_id for anonymous sessions
            session_uuid = db_session.session_id

        # Add query to session
        db_query = session_db_service.add_query_to_session(
            db,
            session_uuid,
            query_request.query,
            query_request.metadata
        )

        # Process the query using RAG service
        response_model = rag_service.process_query(query_request)

        # If response is None, it means the query cannot be answered from textbook content
        if response_model is None:
            session_db_service.update_session_last_accessed(db, session_uuid)
            db.close()
            raise HTTPException(
                status_code=HTTP_422_UNPROCESSABLE_ENTITY,
                detail=ERROR_QUERY_OUTSIDE_SCOPE
            )

        # Add response to database
        db_response = session_db_service.add_response_to_query(
            db,
            db_query.query_id,
            response_model.content,
            response_model.citations,
            response_model.confidence_score
        )

        # Update session last accessed time
        session_db_service.update_session_last_accessed(db, session_uuid)

        # Prepare the response
        result = {
            "response_id": str(db_response.response_id),
            "query": query_request.query,
            "answer": response_model.content,
            "citations": response_model.citations,
            "confidence_score": response_model.confidence_score,
            "session_id": str(session_uuid),
            "timestamp": datetime.datetime.now().isoformat()
        }

        db.close()
        return result

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Handle any other exceptions
        try:
            db.close()
        except:
            pass  # Ignore errors when closing db in error handling
        raise HTTPException(
            status_code=500,
            detail=f"Internal server error: {str(e)}"
        )