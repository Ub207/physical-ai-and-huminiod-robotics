---
title: Physical AI & Humanoid Robotics RAG Chatbot
emoji: 🤖
colorFrom: purple
colorTo: yellow
sdk: docker
dockerFile: Dockerfile
pinned: false
license: mit
---

# Physical AI & Humanoid Robotics RAG Chatbot

This is a Retrieval-Augmented Generation (RAG) chatbot that answers questions based on Physical AI and Humanoid Robotics textbook content. The system processes PDF, EPUB, and TXT files and uses AI models to answer questions based on the content.

## Environment Variables Required

You'll need to set the following secrets in the Space settings:

### AI Provider Configuration
- `AI_PROVIDER`: Either "cohere" or "gemini" (default: "cohere")
- `COHERE_API_KEY`: Your Cohere API key (required if using Cohere)
- `COHERE_EMBED_MODEL`: Cohere embedding model (default: "embed-english-v3.0")
- `COHERE_GENERATION_MODEL`: Cohere generation model (default: "command-r-plus")
- `GEMINI_API_KEY`: Your Google Gemini API key (required if using Gemini)
- `GEMINI_EMBED_MODEL`: Gemini embedding model (default: "embedding-001")
- `GEMINI_GENERATION_MODEL`: Gemini generation model (default: "gemini-1.5-pro-latest")

### Vector Database Configuration
- `QDRANT_URL`: Your Qdrant cloud instance URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `QDRANT_COLLECTION_NAME`: Collection name for storing vectors (default: "book_chunks")

### Database Configuration
- `NEON_DATABASE_URL`: Your Neon Postgres connection string

### Application Configuration
- `UPLOAD_FOLDER`: Directory for file uploads (default: "uploads")
- `MAX_CONTENT_LENGTH`: Maximum upload size in bytes (default: 16777216 = 16MB)
- `ALLOWED_EXTENSIONS`: Comma-separated list of allowed file extensions (default: "pdf,epub,txt")

## How to Deploy on Hugging Face

### Option 1: Hugging Face Spaces (UI-based deployment)
1. Fork this repository to your Hugging Face account
2. Go to your Hugging Face profile → Spaces → Create new Space
3. Select "Docker" SDK and "CPU" hardware (or GPU if needed)
4. Enter the repository URL of your forked repo
5. Set up the required environment variables in Space secrets
6. Wait for the Space to build and start
7. The API will be available at the Space URL

### Option 2: Hugging Face Inference API (API-based deployment)
1. Push this repository to a Hugging Face Model Hub repository
2. Go to the repository page and click on "Deploy" → "Inference API"
3. Configure the hardware and environment variables
4. The API endpoint will be available at `https://api-inference.huggingface.co/models/YOUR_USERNAME/YOUR_REPO_NAME`

## API Endpoints

- `GET /` - Root endpoint
- `GET /health` - Health check
- `POST /query` - Query the RAG system with a question about a book
- `POST /ingest` - Ingest a book file into the vector database
- `POST /upload` - Upload a book file (PDF, EPUB, TXT)

## Usage Examples

### Query a book:
```json
{
  "book_id": "my-book-id",
  "question": "What is the main theme of this book?"
}
```

### Ingest a book:
```json
{
  "book_id": "my-book-id",
  "file_path": "/path/to/book.pdf",
  "title": "My Book Title"
}
```

## Architecture

The application uses:
- FastAPI for the web framework
- Cohere or Google Gemini for AI processing
- Qdrant for vector storage and similarity search
- PostgreSQL (Neon) for metadata storage
- PDFPlumber and EbookLib for document processing