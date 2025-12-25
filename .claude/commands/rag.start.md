---
description: Start the RAG chatbot backend server
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

This command starts the FastAPI backend server for the RAG chatbot system.

1. **Pre-flight Checks**:
   - Verify `.env` file exists with required variables
   - Check that Qdrant collection has indexed content
   - Confirm database connections are available
   - Check if port 8001 is available (or user-specified port)

2. **Port Management**:
   - Default port: 8001
   - If user provided port in arguments: `$ARGUMENTS`
   - Check if port is already in use:
   ```bash
   netstat -ano | findstr :[PORT]
   ```
   - If occupied, either kill the process or use different port

3. **Start Backend Server**:
   ```bash
   cd rag-chatbot && python -m uvicorn api.main:app --host 0.0.0.0 --port 8001 --reload
   ```

   Server features:
   - FastAPI with automatic API docs
   - CORS enabled for frontend integration
   - Hot reload for development
   - Health check endpoint
   - Query endpoint with RAG pipeline
   - Ingest endpoint for new content

4. **Verify Server Status**:
   - Wait 3-5 seconds for startup
   - Check health endpoint:
   ```bash
   curl http://localhost:8001/health
   ```
   - Verify API docs accessible: `http://localhost:8001/docs`

5. **Test Query Endpoint**:
   ```bash
   curl -X POST http://localhost:8001/query \
     -H "Content-Type: application/json" \
     -d '{"book_id": "physical_ai_textbook", "query": "What is Physical AI?"}'
   ```

6. **Report Server Info**:
   - Server URL: `http://localhost:[PORT]`
   - API Documentation: `http://localhost:[PORT]/docs`
   - Health Check: `http://localhost:[PORT]/health`
   - Available endpoints:
     - POST `/query` - Query the RAG system
     - POST `/ingest` - Ingest new book content
     - POST `/upload` - Upload book files
     - GET `/health` - Server health status

## Usage Examples

```bash
# Start on default port 8001
/rag.start

# Start on custom port
/rag.start 8080

# Start with specific host
/rag.start --host 0.0.0.0 --port 8001
```

## API Endpoints

### POST /query
```json
{
  "book_id": "physical_ai_textbook",
  "query": "What is ROS 2?",
  "user_selected_text": "optional context from selected text"
}
```

Response:
```json
{
  "answer": "ROS 2 is...",
  "sources": ["source1", "source2"],
  "confidence": 0.85
}
```

### GET /health
Returns server health status and configuration

## Error Handling

- **Port in use**: Kill existing process or choose different port
- **Missing dependencies**: Run `pip install -r requirements.txt`
- **Database connection error**: Check Qdrant and Neon URLs
- **API key errors**: Verify environment variables

## Notes

- Server runs in reload mode for development
- For production, use gunicorn or similar WSGI server
- Backend must be running for frontend chatbot widget to work
- Server logs will show each query and response
