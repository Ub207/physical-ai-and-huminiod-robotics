import os
import uuid
from typing import List, Dict, Any
from pathlib import Path
import re

def generate_id() -> str:
    """Generate a unique ID for books and chunks."""
    return str(uuid.uuid4())

def validate_file_type(file_path: str) -> bool:
    """Validate if the file type is supported."""
    supported_types = {'.pdf', '.epub', '.txt'}
    file_extension = Path(file_path).suffix.lower()
    return file_extension in supported_types

def ensure_upload_dir(upload_path: str) -> None:
    """Ensure upload directory exists."""
    os.makedirs(upload_path, exist_ok=True)

def format_sources(chunks: List[Dict[str, Any]]) -> List[dict]:
    """Format source chunks for response."""
    sources = []
    for chunk in chunks:
        sources.append({
            'page': chunk.get('page_number', 1),
            'content_snippet': chunk.get('content', '')[:200] + '...' if len(chunk.get('content', '')) > 200 else chunk.get('content', ''),
            'relevance_score': chunk.get('score', 0.0)
        })
    return sources

def sanitize_filename(filename: str) -> str:
    """Sanitize filename to prevent security issues."""
    # Remove path traversal characters
    filename = filename.replace('../', '').replace('..\\', '')
    # Only allow alphanumeric characters, dots, hyphens, and underscores
    sanitized = re.sub(r'[^a-zA-Z0-9._-]', '_', filename)
    return sanitized