import os
from dotenv import load_dotenv
from pydantic_settings import BaseSettings

# Load environment variables
load_dotenv()

class Settings(BaseSettings):
    # AI Provider Configuration
    ai_provider: str = os.getenv("AI_PROVIDER", "cohere")  # Options: 'cohere' or 'gemini'

    # Cohere Configuration
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    cohere_embed_model: str = os.getenv("COHERE_EMBED_MODEL", "embed-english-v3.0")
    cohere_generation_model: str = os.getenv("COHERE_GENERATION_MODEL", "command-r-plus")

    # Gemini Configuration
    gemini_api_key: str = os.getenv("GEMINI_API_KEY", "")
    gemini_embed_model: str = os.getenv("GEMINI_EMBED_MODEL", "embedding-001")
    gemini_generation_model: str = os.getenv("GEMINI_GENERATION_MODEL", "gemini-1.5-pro-latest")

    # Qdrant Configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")

    # Neon Postgres Configuration
    neon_database_url: str = os.getenv("NEON_DATABASE_URL", "")

    # Application Configuration
    upload_folder: str = os.getenv("UPLOAD_FOLDER", "uploads")
    max_content_length: int = int(os.getenv("MAX_CONTENT_LENGTH", "16777216"))
    allowed_extensions: set = set(os.getenv("ALLOWED_EXTENSIONS", "pdf,epub,txt").split(','))

    # Model Configuration
    embedding_size: int = 1024  # Cohere embed-english-v3.0 returns 1024-dim vectors
    chunk_size: int = 512
    chunk_overlap: int = 50
    max_context_length: int = 4000  # Max tokens for context

    class Config:
        case_sensitive = True

settings = Settings()