from pydantic import BaseModel
from typing import Dict, Any, Optional


class ChunkModel(BaseModel):
    chunk_id: Optional[str] = None
    document_id: Optional[str] = None
    content: str
    metadata: Optional[Dict[str, Any]] = None