# Integrated RAG Chatbot for Published Book

A Retrieval-Augmented Generation (RAG) chatbot that can be embedded into digital books (PDF viewers or web pages) to answer questions based on book content and user-selected text.

## Features

- **Book Question Answering**: Answers questions based on the entire book content using RAG
- **Ad-hoc Text Queries**: Answers questions based on user-selected/highlighted text
- **Embeddable Widget**: Can be integrated into PDF viewers or web pages as a chat widget
- **Multi-format Support**: Supports PDF, EPUB, and plain text book formats
- **Cohere-powered**: Uses Cohere's free trial API for embeddings and generation (embed-english-v3.0 and command-r-plus)
- **Cloud-native**: Uses Qdrant Cloud for vector storage and Neon Postgres for session management

## Tech Stack

- **Backend**: FastAPI
- **Embeddings & Generation**: Cohere API (embed-english-v3.0 and command-r-plus)
- **Vector Database**: Qdrant Cloud (free tier)
- **Relational Database**: Neon Serverless Postgres
- **Frontend**: Embeddable JavaScript widget
- **Ingestion CLI**: Command-line tool for book processing

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your API keys and connection strings
```

3. Run the backend:
```bash
uvicorn api.main:app --reload
```

4. Process a book:
```bash
python -m cli.ingest --file path/to/book.pdf --book-id my-book
```

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Book Viewer   │    │    Frontend      │    │    FastAPI      │
│   (PDF/HTML)    │◄──►│   Chat Widget    │◄──►│    Backend      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                         │
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Qdrant Cloud  │    │  Neon Postgres   │    │   Cohere API    │
│   (Vectors)     │    │   (Sessions)     │    │ (Embeddings/LLM)│
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## API Endpoints

- `POST /ingest` - Ingest a book into the RAG system
- `POST /query` - Query the RAG system with a question
- `POST /upload` - Upload a book file
- `GET /health` - Health check

## Configuration

All configuration is managed through environment variables in `.env`:

- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `NEON_DATABASE_URL`: Your Neon Postgres connection string