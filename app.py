import gradio as gr
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import os
import logging

# Import your existing modules
from rag_chatbot.api.main import app as api_app
from rag_chatbot.api.rag_service import RAGService

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create the main app
main_app = FastAPI(title="RAG Chatbot for Published Books - Hugging Face Deployment")

# Add CORS middleware for Hugging Face Spaces
main_app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # For Hugging Face Spaces, we may need to be more permissive
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount the API routes
main_app.mount("/api", api_app)

# Create a simple Gradio interface for testing
def test_query(book_id: str, query: str):
    # This is a simplified version for demonstration
    # In practice, you'd call your RAG service here
    rag_service = RAGService()
    # You would need to implement the actual query logic here
    return f"Query: {query} for book: {book_id}"

with main_app:
    gr.mount_gradio_app(main_app,
                       gr.Interface(fn=test_query,
                                   inputs=["text", "text"],
                                   outputs="text",
                                   title="RAG Chatbot Test Interface",
                                   description="Test the RAG chatbot backend"),
                       path="/gradio")

if __name__ == "__main__":
    uvicorn.run(main_app, host="0.0.0.0", port=int(os.environ.get("PORT", 7860)))