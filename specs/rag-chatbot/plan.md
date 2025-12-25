# RAG Chatbot System - Implementation Plan

## Executive Summary

This plan outlines the technical implementation of a Retrieval-Augmented Generation (RAG) chatbot system for the Physical AI & Humanoid Robotics textbook. The system combines vector similarity search with AI-powered generation to provide accurate, source-attributed answers.

**Timeline**: Already implemented and deployed
**Status**: Production-ready with ongoing enhancements
**Tech Stack**: Python/FastAPI, Qdrant, Cohere/Gemini, React/Docusaurus

## Technical Context

### Current Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Docusaurus Frontend                       │
│  ┌────────────┐     ┌──────────────────┐                   │
│  │   Pages    │────▶│  RagChatWidget   │                   │
│  └────────────┘     └────────┬─────────┘                   │
└─────────────────────────────┼─────────────────────────────┘
                               │ HTTP/JSON
                               │
┌──────────────────────────────▼─────────────────────────────┐
│                    FastAPI Backend                          │
│  ┌────────────┐     ┌──────────────────┐                   │
│  │  API Main  │────▶│   RAG Service    │                   │
│  └────────────┘     └────────┬─────────┘                   │
└─────────────────────────────┼─────────────────────────────┘
                               │
        ┌──────────────────────┼──────────────────────┐
        │                      │                      │
        ▼                      ▼                      ▼
┌───────────────┐    ┌─────────────────┐    ┌──────────────┐
│    Qdrant     │    │  Cohere/Gemini  │    │   Neon PG    │
│ Vector Store  │    │   AI Provider   │    │   Database   │
└───────────────┘    └─────────────────┘    └──────────────┘
```

### Technology Stack

#### Backend
- **Framework**: FastAPI 0.104+ (async Python web framework)
- **Language**: Python 3.11+
- **Server**: Uvicorn (ASGI server)
- **Dependencies**:
  - `cohere` - Cohere AI SDK
  - `google-generativeai` - Google Gemini SDK
  - `qdrant-client` - Vector database client
  - `psycopg2-binary` - PostgreSQL adapter
  - `pdfplumber` - PDF text extraction
  - `ebooklib` - EPUB processing

#### Frontend
- **Framework**: React 18+ (via Docusaurus)
- **Build Tool**: Docusaurus 3.1+
- **Widget**: Custom React component with vanilla JS injection
- **Styling**: Inline CSS for portability

#### Infrastructure
- **Vector DB**: Qdrant Cloud (1M vectors, 1GB storage)
- **Relational DB**: Neon PostgreSQL (512MB, serverless)
- **AI Provider**: Cohere (primary), Google Gemini (fallback)
- **Frontend Host**: Vercel (static site)
- **Backend Host**: Render (or Railway, Hugging Face)

### Design Decisions

#### Decision 1: Vector Database Selection
**Options Considered**:
1. Qdrant Cloud
2. Pinecone
3. Weaviate
4. Chroma (local)

**Chosen**: Qdrant Cloud

**Rationale**:
- Generous free tier (1M vectors)
- Cloud-hosted (no self-hosting overhead)
- Excellent Python client
- Cosine similarity support
- Payload filtering for multi-book support
- Fast query performance (<100ms)

**Trade-offs**:
- ✅ Easy setup and maintenance
- ✅ Scalable cloud infrastructure
- ❌ Vendor lock-in (mitigation: export vectors regularly)
- ❌ Network latency vs local (acceptable: <100ms)

#### Decision 2: AI Provider Strategy
**Options Considered**:
1. OpenAI GPT-4
2. Cohere (primary)
3. Google Gemini (fallback)
4. Anthropic Claude

**Chosen**: Cohere + Gemini multi-provider

**Rationale**:
- Cohere specialized in embeddings + generation
- Competitive pricing vs OpenAI
- Gemini provides redundancy
- Both have generous free tiers
- Consistent API interfaces

**Trade-offs**:
- ✅ Cost-effective for prototype/demo
- ✅ Provider redundancy reduces risk
- ✅ Good documentation and SDKs
- ❌ Two providers to maintain
- ❌ Slightly different response formats

#### Decision 3: Embedding Model
**Options Considered**:
1. Cohere embed-english-v3.0 (1024-dim)
2. OpenAI text-embedding-3-small (1536-dim)
3. Sentence-transformers (384-dim, local)

**Chosen**: Cohere embed-english-v3.0

**Rationale**:
- 1024 dimensions: sweet spot for accuracy vs storage
- English-optimized for textbook content
- Fast generation (<200ms)
- Free tier: 100 requests/min
- Consistent with generation model

**Trade-offs**:
- ✅ High quality embeddings
- ✅ Fast and cost-effective
- ❌ Requires API key (not local)
- ❌ Network dependency

#### Decision 4: Chunking Strategy
**Options Considered**:
1. Fixed-size (500 char, 50 overlap)
2. Sentence-based (semantic boundaries)
3. Paragraph-based (natural breaks)

**Chosen**: Fixed-size with overlap

**Rationale**:
- Predictable chunk sizes
- Overlap preserves context across boundaries
- Simple implementation
- Works well with technical content
- Tested: 661 chunks from textbook

**Parameters**:
- Chunk size: 500 characters
- Overlap: 50 characters (10%)
- Separator: Whitespace-aware

**Trade-offs**:
- ✅ Simple and predictable
- ✅ Good retrieval performance
- ❌ May split sentences awkwardly
- ❌ Not semantically aware (acceptable for technical content)

## Architecture Deep Dive

### Component Breakdown

#### 1. Document Processor (`utils/text_processor.py`)
**Responsibilities**:
- Read TXT, PDF, EPUB files
- Clean and normalize text
- Split into chunks with overlap
- Maintain chunk metadata (position, source)

**Key Functions**:
```python
def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]
def process_pdf(file_path: str) -> str
def process_epub(file_path: str) -> str
```

#### 2. Vector Store Manager (`utils/vector_store.py`)
**Responsibilities**:
- Initialize Qdrant connection
- Create/manage collections
- Store embeddings with metadata
- Search for similar chunks

**Key Functions**:
```python
def create_collection(name: str, vector_size: int = 1024)
def index_documents(book_id: str, chunks: List[str])
def search(query_vector: List[float], limit: int = 5) -> List[SearchResult]
```

#### 3. AI Client Factory (`utils/client_factory.py`)
**Responsibilities**:
- Select provider based on config
- Instantiate appropriate client
- Provide consistent interface

**Key Functions**:
```python
def get_ai_client(provider: str = "cohere") -> BaseAIClient
class BaseAIClient:
    def embed(text: str) -> List[float]
    def generate(prompt: str, context: List[str]) -> str
```

#### 4. Cohere Client (`utils/cohere_client.py`)
**Responsibilities**:
- Generate embeddings via Cohere API
- Generate answers with command-r-plus
- Handle API errors and retries

**Key Functions**:
```python
def embed_text(text: str) -> List[float]
def generate_answer(query: str, chunks: List[str]) -> Dict
```

#### 5. RAG Service (`api/rag_service.py`)
**Responsibilities**:
- Orchestrate RAG pipeline
- Combine query + context + chunks
- Calculate confidence scores
- Format responses

**Key Functions**:
```python
def query(book_id: str, question: str, context: Optional[str]) -> QueryResponse
def calculate_confidence(similarities: List[float]) -> float
```

#### 6. FastAPI Main (`api/main.py`)
**Responsibilities**:
- Define API endpoints
- Handle CORS
- Request validation
- Error handling

**Endpoints**:
```python
POST /query - Query the RAG system
POST /ingest - Ingest new document
POST /upload - Upload document file
GET /health - System health check
```

#### 7. Chat Widget (`src/components/chat/RagChatWidget.js`)
**Responsibilities**:
- Render chat UI
- Capture user input
- Handle text selection
- Display responses
- Manage state

**Key Features**:
- Floating button
- Expandable chat window
- Message history
- Loading indicators
- Error states

### Data Flow

#### Query Flow
```
1. User types question in widget
   ↓
2. Widget sends POST /query to backend
   {
     "book_id": "physical_ai_textbook",
     "query": "What is Physical AI?",
     "user_selected_text": null
   }
   ↓
3. RAG Service receives request
   ↓
4. Generate query embedding (Cohere)
   [0.123, 0.456, ..., 0.789] (1024 dims)
   ↓
5. Search Qdrant for similar chunks
   Returns top 5 chunks with scores
   ↓
6. Build prompt:
   "Context: [chunk1] [chunk2] [chunk3]
    Question: What is Physical AI?
    Answer based only on context:"
   ↓
7. Generate answer (Cohere command-r-plus)
   ↓
8. Calculate confidence from similarity scores
   ↓
9. Format response:
   {
     "answer": "Physical AI refers to...",
     "sources": ["chunk1", "chunk2"],
     "confidence": 0.85
   }
   ↓
10. Widget displays answer with sources
```

#### Indexing Flow
```
1. Run: python convert_docs.py
   ↓
2. Read physical_ai_textbook.txt
   ↓
3. Clean and normalize text
   ↓
4. Split into 500-char chunks (50 overlap)
   ↓
5. For each chunk:
   - Generate embedding (Cohere)
   - Store in Qdrant with metadata:
     {
       "book_id": "physical_ai_textbook",
       "chunk_id": 42,
       "text": "Physical AI refers to...",
       "vector": [0.123, ...]
     }
   ↓
6. Create payload index on book_id
   ↓
7. Validate: Query test question
   ↓
8. Report: 661 chunks indexed ✓
```

### Database Schema

#### Qdrant Collection: `book_chunks`
```javascript
{
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  },
  "payload": {
    "book_id": "string",      // Filter by book
    "chunk_id": "integer",    // Chunk number
    "text": "string",         // Original text
    "position": "integer"     // Position in document
  }
}
```

#### Neon PostgreSQL: `conversations`
```sql
CREATE TABLE conversations (
  id SERIAL PRIMARY KEY,
  session_id VARCHAR(255) NOT NULL,
  book_id VARCHAR(255) NOT NULL,
  query TEXT NOT NULL,
  answer TEXT NOT NULL,
  confidence FLOAT,
  sources JSONB,
  created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_session_id ON conversations(session_id);
CREATE INDEX idx_created_at ON conversations(created_at);
```

### API Contracts

See `specs/rag-chatbot/contracts/api.yaml` for full OpenAPI specification.

#### POST /query
**Request**:
```json
{
  "book_id": "physical_ai_textbook",
  "query": "What is Physical AI?",
  "user_selected_text": "optional context"
}
```

**Response** (200 OK):
```json
{
  "answer": "Physical AI refers to artificial intelligence systems...",
  "sources": [
    "Chunk 42: Physical AI combines computer vision...",
    "Chunk 156: These systems interact with..."
  ],
  "confidence": 0.852,
  "metadata": {
    "query_time_ms": 432,
    "chunks_searched": 661,
    "embedding_time_ms": 120,
    "search_time_ms": 85,
    "generation_time_ms": 227
  }
}
```

**Response** (400 Bad Request):
```json
{
  "error": "Invalid request",
  "detail": "Query text is required"
}
```

**Response** (500 Internal Server Error):
```json
{
  "error": "Internal server error",
  "detail": "Failed to connect to vector database"
}
```

## Configuration Management

### Environment Variables

#### Required (No defaults)
```bash
# AI Provider Selection
AI_PROVIDER=cohere  # or "gemini"

# Cohere Configuration (if AI_PROVIDER=cohere)
COHERE_API_KEY=your_cohere_api_key_here

# Gemini Configuration (if AI_PROVIDER=gemini)
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Configuration
QDRANT_URL=https://xxx.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Database Configuration
NEON_DATABASE_URL=postgresql://user:pass@host/db
```

#### Optional (With defaults)
```bash
# AI Model Configuration
COHERE_EMBED_MODEL=embed-english-v3.0
COHERE_GENERATION_MODEL=command-r-plus
GEMINI_EMBED_MODEL=embedding-001
GEMINI_GENERATION_MODEL=gemini-1.5-pro-latest

# Qdrant Configuration
QDRANT_COLLECTION_NAME=book_chunks
QDRANT_PREFER_GRPC=false

# Application Configuration
UPLOAD_FOLDER=uploads
MAX_CONTENT_LENGTH=16777216  # 16MB
ALLOWED_EXTENSIONS=pdf,epub,txt

# Chunking Configuration
CHUNK_SIZE=500
CHUNK_OVERLAP=50
```

### Configuration Files

#### `.env` (Local Development)
```bash
AI_PROVIDER=cohere
COHERE_API_KEY=***
QDRANT_URL=https://xxx.cloud.qdrant.io
QDRANT_API_KEY=***
NEON_DATABASE_URL=postgresql://***
```

#### `.env.example` (Template)
```bash
AI_PROVIDER=cohere
COHERE_API_KEY=your_cohere_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_key_here
NEON_DATABASE_URL=your_neon_connection_string_here
```

## Deployment Architecture

### Development Environment
```
Local Machine
├── Backend: http://localhost:8001 (uvicorn --reload)
├── Frontend: http://localhost:3000 (npm start)
├── Qdrant: Cloud instance
├── Neon: Cloud database
└── API Keys: From .env file
```

### Production Environment
```
Frontend: Vercel (static site)
  └─▶ https://physical-robotics-pi.vercel.app

Backend: Render (web service)
  └─▶ https://rag-chatbot-backend.onrender.com

Qdrant: Cloud (managed)
  └─▶ https://xxx.cloud.qdrant.io

Neon: Cloud (managed)
  └─▶ postgresql://xxx@xxx.neon.tech/xxx
```

### Scaling Considerations

#### Current Limits (Free Tier)
- **Cohere**: 100 requests/minute
- **Qdrant**: 1M vectors, 1GB storage
- **Neon**: 512MB database, 1 connection
- **Render**: 750 hours/month, cold start delays

#### Scaling Path
1. **Level 1** (Current): Free tiers, <100 users
2. **Level 2** (Paid): Render Starter ($7/mo), 500 users
3. **Level 3** (Growth): Dedicated hosting, 5K users
4. **Level 4** (Scale): Multi-region, load balancing

## Performance Optimization

### Current Performance
- **Embedding**: ~120ms per query
- **Vector Search**: ~85ms (661 chunks)
- **Generation**: ~227ms (command-r-plus)
- **Total**: ~432ms (p50), ~650ms (p95)

### Optimization Strategies

#### 1. Caching
```python
# Cache common queries for 1 hour
from functools import lru_cache

@lru_cache(maxsize=100)
def cached_query(query: str, book_id: str) -> str:
    return rag_service.query(book_id, query)
```

#### 2. Connection Pooling
```python
# Reuse connections
from qdrant_client import QdrantClient

client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    prefer_grpc=False,
    timeout=30
)
```

#### 3. Batch Processing
```python
# Batch embeddings during indexing
chunks = [...]  # 100 chunks
embeddings = cohere.embed(chunks, input_type="search_document")
# Single API call vs 100 calls
```

#### 4. Async Operations
```python
# Non-blocking I/O
@app.post("/query")
async def query_endpoint(request: QueryRequest):
    embedding = await get_embedding(request.query)
    results = await search_vectors(embedding)
    answer = await generate_answer(results)
    return answer
```

## Security Considerations

### API Key Management
- ✅ All keys in environment variables
- ✅ `.env` in `.gitignore`
- ✅ Separate keys for dev/prod
- ✅ Key rotation every 90 days
- ✅ No keys in logs/errors

### CORS Configuration
```python
# api/main.py
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://physical-robotics-pi.vercel.app"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Rate Limiting
```python
# Future: Add rate limiting middleware
from slowapi import Limiter

limiter = Limiter(key_func=get_remote_address)

@app.post("/query")
@limiter.limit("10/minute")
async def query_endpoint(...):
    ...
```

## Testing Strategy

### Test Levels

#### 1. Unit Tests
```python
# tests/test_text_processor.py
def test_chunk_text():
    text = "A" * 1000
    chunks = chunk_text(text, chunk_size=500, overlap=50)
    assert len(chunks) == 2
    assert len(chunks[0]) == 500
    assert chunks[0][-50:] == chunks[1][:50]  # Overlap
```

#### 2. Integration Tests
```python
# tests/test_rag_service.py
def test_query_flow():
    # Index test document
    index_documents("test_book", ["chunk1", "chunk2"])

    # Query
    result = rag_service.query("test_book", "test query")

    # Validate
    assert result.answer is not None
    assert len(result.sources) > 0
    assert 0 <= result.confidence <= 1
```

#### 3. End-to-End Tests
```python
# tests/test_api.py
from fastapi.testclient import TestClient

def test_query_endpoint():
    response = client.post("/query", json={
        "book_id": "physical_ai_textbook",
        "query": "What is Physical AI?"
    })
    assert response.status_code == 200
    assert "answer" in response.json()
```

### Test Data
- **test_book.txt**: Small sample (10 chunks)
- **Known queries**: Questions with expected answers
- **Edge cases**: Empty query, very long query, special characters

## Monitoring & Observability

### Metrics to Track
```python
# Log every query
logger.info(
    "Query processed",
    extra={
        "query": query,
        "book_id": book_id,
        "confidence": confidence,
        "query_time_ms": elapsed,
        "chunks_searched": num_chunks
    }
)
```

### Health Checks
```python
@app.get("/health")
async def health_check():
    checks = {
        "qdrant": await test_qdrant_connection(),
        "database": await test_database_connection(),
        "ai_provider": await test_ai_provider()
    }

    healthy = all(checks.values())
    status = "healthy" if healthy else "degraded"

    return {
        "status": status,
        "checks": checks,
        "timestamp": datetime.utcnow()
    }
```

### Error Tracking
- **Development**: Console logs + stack traces
- **Production**: Structured JSON logs to stdout
- **Future**: Sentry or similar error tracking

## Rollout Plan

### Phase 1: Current State ✅
- [x] Core RAG pipeline implemented
- [x] Qdrant integration complete
- [x] Cohere + Gemini providers working
- [x] FastAPI backend deployed
- [x] Widget integrated in Docusaurus
- [x] 661 chunks indexed
- [x] Testing with sample queries

### Phase 2: Enhancements (Future)
- [ ] Response caching for common queries
- [ ] Conversation history in PostgreSQL
- [ ] User feedback collection
- [ ] Admin dashboard for analytics
- [ ] Multi-book support

### Phase 3: Scale (Future)
- [ ] Rate limiting and quotas
- [ ] CDN for static assets
- [ ] Multi-region deployment
- [ ] Advanced monitoring
- [ ] A/B testing framework

## Risk Mitigation

### Risk 1: API Rate Limits
**Impact**: Service degradation during high traffic
**Mitigation**:
- Monitor usage closely
- Implement client-side rate limiting
- Cache responses for 1 hour
- Upgrade API plan when needed

### Risk 2: Vector DB Downtime
**Impact**: Queries fail, user frustration
**Mitigation**:
- Health check before each query
- Graceful error messages
- Retry with exponential backoff
- SLA monitoring and alerts

### Risk 3: Answer Quality
**Impact**: Users distrust system
**Mitigation**:
- Confidence threshold (60%)
- Source attribution always visible
- Feedback mechanism to report issues
- Regular content audits

---

**Document Version**: 1.0.0
**Created**: 2025-12-25
**Last Updated**: 2025-12-25
**Status**: Approved
