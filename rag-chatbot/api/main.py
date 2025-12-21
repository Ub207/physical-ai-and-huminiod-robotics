from fastapi import FastAPI, HTTPException, File, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional
import os
import logging

from config.settings import settings
from models.schemas import QueryRequest, QueryResponse, IngestionRequest
from api.rag_service import RAGService
from utils.helpers import sanitize_filename

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="RAG Chatbot for Published Books",
    description="A RAG system that answers questions based on book content and user-selected text",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize RAG service
rag_service = RAGService()

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API for Published Books"}

@app.post("/ingest", summary="Ingest a book into the RAG system")
async def ingest_book(request: IngestionRequest):
    """
    Process and ingest a book file into the vector database for RAG.
    Supports PDF, EPUB, and TXT formats.
    """
    try:
        logger.info(f"Starting ingestion for book: {request.book_id}")
        result = await rag_service.ingest_book(request)
        logger.info(f"Successfully ingested book: {request.book_id}")
        return result
    except Exception as e:
        logger.error(f"Error ingesting book {request.book_id}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error ingesting book: {str(e)}")

@app.post("/query", response_model=QueryResponse, summary="Query the RAG system")
async def query_book(request: QueryRequest):
    """
    Query the RAG system with a question about the book.
    Can optionally include user-selected text for ad-hoc queries.
    """
    try:
        logger.info(f"Processing query for book: {request.book_id}")
        result = await rag_service.query(request)
        logger.info(f"Successfully processed query for book: {request.book_id}")
        return result
    except Exception as e:
        logger.error(f"Error processing query for book {request.book_id}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/upload", summary="Upload a book file")
async def upload_file(file: UploadFile = File(...)):
    """
    Upload a book file (PDF, EPUB, TXT) to the server.
    """
    try:
        # Validate file type
        if not file.filename:
            raise HTTPException(status_code=400, detail="No file provided")

        # Check file extension
        file_ext = os.path.splitext(file.filename)[1].lower()
        if file_ext not in ['.pdf', '.epub', '.txt']:
            raise HTTPException(status_code=400, detail="File type not supported. Only PDF, EPUB, and TXT are allowed.")

        # Create upload directory if it doesn't exist
        os.makedirs(settings.upload_folder, exist_ok=True)

        # Sanitize filename to prevent security issues
        sanitized_filename = sanitize_filename(file.filename)
        file_path = os.path.join(settings.upload_folder, sanitized_filename)

        # Save file
        with open(file_path, "wb") as buffer:
            content = await file.read()
            buffer.write(content)

        return {
            "filename": sanitized_filename,
            "file_path": file_path,
            "size": len(content),
            "message": "File uploaded successfully"
        }
    except Exception as e:
        logger.error(f"Error uploading file: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error uploading file: {str(e)}")

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy", "message": "RAG Chatbot API is running"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)