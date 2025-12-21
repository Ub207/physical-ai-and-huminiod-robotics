from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

class Book(BaseModel):
    id: str
    title: str
    author: str
    file_path: str
    file_type: str
    total_pages: int
    total_chunks: int
    created_at: datetime
    updated_at: datetime

class Chunk(BaseModel):
    id: str
    book_id: str
    page_number: int
    content: str
    embedding: Optional[List[float]] = None
    metadata: dict = {}

class QueryRequest(BaseModel):
    book_id: str
    query: str
    user_selected_text: Optional[str] = None
    context_window: Optional[int] = 3  # Number of surrounding chunks to include

class QueryResponse(BaseModel):
    answer: str
    sources: List[dict]
    confidence: float
    tokens_used: int

class IngestionRequest(BaseModel):
    book_id: str
    title: str
    author: str
    file_path: str
    chunk_size: int = 512
    chunk_overlap: int = 50