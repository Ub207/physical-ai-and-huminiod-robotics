# RAG Chatbot Claude Code Commands

This document describes the available Claude Code slash commands for managing the RAG chatbot system.

## Available Commands

### 1. `/rag.setup` - Setup Environment
Setup and configure the RAG chatbot environment from scratch.

**What it does:**
- Creates Python virtual environment
- Installs dependencies from requirements.txt
- Creates .env file from template
- Validates API keys and connections
- Sets up Qdrant and Neon database

**Usage:**
```bash
/rag.setup
```

**When to use:**
- First time setup
- After cloning the repository
- When reinstalling dependencies

---

### 2. `/rag.index` - Index Book Content
Index book content into the Qdrant vector database.

**What it does:**
- Reads book content (TXT, PDF, or EPUB)
- Splits text into chunks with overlap
- Generates embeddings using Cohere/Gemini
- Stores vectors in Qdrant collection
- Creates payload index for book_id

**Usage:**
```bash
# Index default book
/rag.index

# Index specific file
/rag.index path/to/book.txt
```

**When to use:**
- After setup, before first query
- When book content is updated
- To re-index with different settings

---

### 3. `/rag.start` - Start Backend Server
Start the FastAPI backend server for the RAG chatbot.

**What it does:**
- Checks port availability (default: 8001)
- Starts uvicorn server with hot reload
- Enables CORS for frontend
- Exposes API endpoints and documentation

**Usage:**
```bash
# Start on default port 8001
/rag.start

# Start on custom port
/rag.start 8080
```

**When to use:**
- Before testing queries
- Before starting the Docusaurus frontend
- After making code changes (auto-reloads)

**Endpoints:**
- API Docs: http://localhost:8001/docs
- Health Check: http://localhost:8001/health
- Query: POST http://localhost:8001/query

---

### 4. `/rag.query` - Query the Chatbot
Send a query to the RAG chatbot and get an answer.

**What it does:**
- Sends question to backend API
- Performs vector similarity search
- Generates answer using AI model
- Returns answer with confidence and sources

**Usage:**
```bash
/rag.query What is Physical AI?

/rag.query Explain ROS 2 architecture
```

**Response includes:**
- Generated answer
- Confidence score (0-100%)
- Source chunks used
- Query performance metrics

**When to use:**
- Testing chatbot responses
- Verifying indexed content quality
- Debugging query accuracy

---

### 5. `/rag.test` - Test the System
Run comprehensive tests on the RAG chatbot system.

**What it does:**
- Tests environment configuration
- Verifies Qdrant connection
- Tests embedding generation
- Checks API endpoints
- Runs sample queries
- Reports performance metrics

**Usage:**
```bash
# Full test suite
/rag.test

# Test specific component
/rag.test query
```

**Tests performed:**
- Environment variables
- Database connections
- Vector search accuracy
- API response times
- Confidence scores

**When to use:**
- After setup to verify installation
- Before deployment
- When debugging issues
- After making changes

---

### 6. `/rag.deploy` - Deploy to Cloud
Deploy the RAG chatbot backend to cloud platforms.

**What it does:**
- Guides deployment to Render, Hugging Face, Railway
- Creates platform-specific configuration files
- Sets up environment variables
- Tests deployed endpoints
- Updates frontend with production URL

**Usage:**
```bash
# Deploy to Render
/rag.deploy render

# Deploy to Hugging Face
/rag.deploy huggingface
```

**Supported platforms:**
- Render (recommended for FastAPI)
- Hugging Face Spaces (good for demos)
- Railway (quick prototypes)
- Fly.io (edge computing)

**When to use:**
- When ready for production
- To share with others
- For public demos

---

## Typical Workflow

### First Time Setup
```bash
1. /rag.setup          # Setup environment
2. /rag.index          # Index book content
3. /rag.start          # Start backend
4. /rag.test           # Verify everything works
5. /rag.query          # Test with questions
```

### Daily Development
```bash
1. /rag.start          # Start backend (auto-reloads on changes)
2. /rag.query          # Test queries as needed
```

### Before Deployment
```bash
1. /rag.test           # Run full test suite
2. /rag.deploy render  # Deploy to production
```

### After Content Updates
```bash
1. /rag.index          # Re-index new content
2. /rag.test           # Verify indexing
3. /rag.query          # Test with relevant queries
```

---

## Configuration Files

### Environment Variables (.env)
```env
# AI Provider (choose one)
AI_PROVIDER=cohere
COHERE_API_KEY=your_cohere_key
COHERE_EMBED_MODEL=embed-english-v3.0
COHERE_GENERATION_MODEL=command-r-plus

# Vector Database
QDRANT_URL=https://xxx.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_key
QDRANT_COLLECTION_NAME=book_chunks

# Relational Database
NEON_DATABASE_URL=postgresql://user:pass@host/db
```

### Book Content Location
- Default: `rag-chatbot/physical_ai_textbook.txt`
- Supported formats: TXT, PDF, EPUB
- Custom path: Use with `/rag.index path/to/file`

---

## Troubleshooting

### Command Not Found
Make sure you're in the project root directory:
```bash
cd D:/physical-ai and huminiod-robotics
```

### Backend Won't Start
Check if port is in use:
```bash
netstat -ano | findstr :8001
```

### Low Confidence Scores
- Re-index content: `/rag.index`
- Try more specific queries
- Check if topic is in book content

### Deployment Issues
- Verify all environment variables
- Check platform-specific logs
- Test endpoints after deployment

---

## API Endpoints Reference

### POST /query
Query the RAG system with a question.

**Request:**
```json
{
  "book_id": "physical_ai_textbook",
  "query": "What is Physical AI?",
  "user_selected_text": null
}
```

**Response:**
```json
{
  "answer": "Physical AI refers to...",
  "sources": ["chunk1", "chunk2"],
  "confidence": 0.85,
  "metadata": {
    "query_time_ms": 432,
    "chunks_searched": 661
  }
}
```

### GET /health
Check server health and configuration.

**Response:**
```json
{
  "status": "healthy",
  "ai_provider": "cohere",
  "qdrant_connected": true,
  "database_connected": true
}
```

---

## Performance Benchmarks

Typical performance metrics:
- **Indexing**: 2-5 minutes for 661 chunks
- **Query embedding**: 50-200ms
- **Vector search**: 50-150ms
- **Answer generation**: 200-500ms
- **Total query time**: 300-800ms

---

## Notes

- Keep `.env` file secure (never commit to git)
- Backend must run on port 8001 for frontend integration
- First query may be slower due to cold start
- Free tier limits:
  - Cohere: 100 requests/min
  - Qdrant: 1GB storage, 1M vectors
  - Neon: 512MB database

---

## Support

For issues or questions:
1. Check command help: `/rag.command --help`
2. Review logs in console
3. Test with `/rag.test`
4. Check README.md for detailed documentation

---

**Last Updated:** December 25, 2024
**Version:** 1.0.0
