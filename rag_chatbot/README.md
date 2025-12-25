---
title: RAG Chatbot Backend
emoji: 🤖
colorFrom: purple
colorTo: blue
sdk: docker
pinned: false
---

# RAG Chatbot Backend

This is the backend API for the RAG (Retrieval-Augmented Generation) Chatbot system that answers questions based on book content.

## API Endpoints

- `GET /health` - Health check
- `POST /query` - Query the RAG system
- `POST /ingest` - Ingest a book into the system
- `POST /upload` - Upload a book file

## Environment Variables

You need to set the following environment variables:

```
COHERE_API_KEY=your_cohere_api_key
COHERE_EMBED_MODEL=embed-english-v3.0
COHERE_GENERATION_MODEL=command-r-plus
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_chunks
NEON_DATABASE_URL=your_neon_db_url
```

## Usage

This backend is designed to work with the frontend component of the RAG chatbot system. The frontend will make requests to the API endpoints to query the knowledge base.