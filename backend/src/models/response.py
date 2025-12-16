from pydantic import BaseModel
from typing import List, Dict, Any, Optional


class ResponseModel(BaseModel):
    content: str
    citations: List[Dict[str, Any]]
    confidence_score: float