---
description: Test the RAG chatbot system with sample queries
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

This command runs comprehensive tests on the RAG chatbot system to verify it's working correctly.

1. **Test Configuration**:
   - Verify backend server is running (port 8001 or custom)
   - Check Qdrant vector database connection
   - Verify book content is indexed
   - Confirm AI provider (Cohere/Gemini) is accessible

2. **Run Test Suite**:

   a. **Environment Test**:
   ```bash
   cd rag-chatbot && python -c "
   from config.settings import settings
   print(f'AI Provider: {settings.AI_PROVIDER}')
   print(f'Qdrant URL: {settings.QDRANT_URL}')
   print(f'Collection: {settings.QDRANT_COLLECTION_NAME}')
   "
   ```

   b. **Vector Store Test**:
   ```bash
   python test_query.py
   ```
   - Verifies Qdrant connection
   - Tests embedding generation
   - Checks vector similarity search
   - Reports number of chunks in collection

   c. **Backend API Test**:
   ```bash
   python test_backend.py
   ```
   - Tests health endpoint
   - Sends sample queries
   - Validates response format
   - Checks confidence scores

3. **Sample Query Tests**:
   Run predefined queries to test different aspects:

   ```python
   test_queries = [
     {
       "query": "What is Physical AI?",
       "expected_topics": ["robotics", "embodied", "agents"]
     },
     {
       "query": "Explain ROS 2 architecture",
       "expected_topics": ["nodes", "topics", "services"]
     },
     {
       "query": "What are Vision-Language-Action models?",
       "expected_topics": ["VLA", "multimodal", "robotics"]
     }
   ]
   ```

4. **Performance Metrics**:
   - Average query response time
   - Embedding generation time
   - Vector search latency
   - Confidence score distribution
   - Number of sources returned

5. **Report Results**:
   ```
   ✓ Environment configuration: PASS
   ✓ Qdrant connection: PASS (661 chunks indexed)
   ✓ Embedding generation: PASS (avg 120ms)
   ✓ Vector search: PASS (avg 85ms)
   ✓ Backend API: PASS (avg response 450ms)
   ✓ Query accuracy: PASS (avg confidence 75.3%)

   Sample Query Results:
   - "What is Physical AI?" → 82.5% confidence, 5 sources
   - "Explain ROS 2" → 78.1% confidence, 5 sources
   - "What are VLA models?" → 71.2% confidence, 4 sources
   ```

## Usage Examples

```bash
# Run full test suite
/rag.test

# Run specific test
/rag.test query

# Test with custom query
/rag.test "What is humanoid robotics?"

# Run performance benchmark
/rag.test --benchmark
```

## Test Files

The command uses these test scripts:
- `test_query.py` - Tests vector search and embeddings
- `test_backend.py` - Tests API endpoints
- `test_system.py` - Full system integration test
- `comprehensive_test.py` - Detailed test with multiple queries

## Expected Output

### Successful Test:
```
Testing RAG Chatbot System
====================================

1. Environment Check: ✓
   - AI Provider: cohere
   - Qdrant URL: https://xxx.cloud.qdrant.io
   - Collection: book_chunks

2. Vector Store: ✓
   - Chunks indexed: 661
   - Embedding dimension: 1024
   - Distance metric: cosine

3. Sample Query: "What is Physical AI?"
   - Response time: 412ms
   - Confidence: 82.5%
   - Sources: 5
   - Answer: Physical AI refers to...

4. API Health: ✓
   - Endpoint: http://localhost:8001/health
   - Status: healthy

All tests passed! ✓
```

### Failed Test:
```
Testing RAG Chatbot System
====================================

1. Environment Check: ✓
2. Vector Store: ✗
   - Error: Connection to Qdrant failed
   - Reason: Invalid API key

Recommendation: Check QDRANT_API_KEY in .env file
```

## Error Handling

- **Server not running**: Start server with `/rag.start`
- **No indexed content**: Run `/rag.index` first
- **API key errors**: Verify environment variables
- **Low confidence scores**: May need to re-index or improve content
- **Timeout errors**: Check network connectivity to Qdrant

## Performance Benchmarks

Typical performance metrics:
- Embedding generation: 50-200ms
- Vector search: 50-150ms
- Total query time: 300-600ms
- Confidence threshold: 60%+ (reliable answers)

## Notes

- Tests should be run after indexing content
- Backend server must be running for API tests
- Low confidence (<60%) may indicate poor content match
- First query may be slower due to cold start
