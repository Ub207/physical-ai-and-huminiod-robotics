---
description: Query the RAG chatbot with a question about the book content
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

This command sends a query to the RAG chatbot and displays the response with sources.

1. **Validate Prerequisites**:
   - Check if backend server is running
   - Verify Qdrant has indexed content
   - Confirm book_id exists in collection

2. **Parse Query Input**:
   - If `$ARGUMENTS` provided, use as query
   - If empty, prompt user for query
   - Support multi-line queries
   - Allow optional context/selected text

3. **Send Query to Backend**:

   ```bash
   curl -X POST http://localhost:8001/query \
     -H "Content-Type: application/json" \
     -d '{
       "book_id": "physical_ai_textbook",
       "query": "$ARGUMENTS",
       "user_selected_text": null
     }'
   ```

   Or using Python:
   ```python
   import requests

   response = requests.post(
       "http://localhost:8001/query",
       json={
           "book_id": "physical_ai_textbook",
           "query": "$ARGUMENTS",
           "user_selected_text": None
       }
   )

   result = response.json()
   ```

4. **Display Response**:

   Format the output for readability:
   ```
   Query: What is Physical AI?
   ====================================

   Answer:
   Physical AI refers to artificial intelligence systems
   that interact with the physical world through embodied
   agents like robots. It combines computer vision, natural
   language processing, and robotic control to enable
   autonomous decision-making in real-world environments.

   Confidence: 85.2%

   Sources (5):
   1. [Chunk 42] Introduction to Physical AI - Page 1
   2. [Chunk 156] Embodied AI Systems - Page 8
   3. [Chunk 203] Robot Intelligence - Page 12
   4. [Chunk 387] Vision-Language Models - Page 24
   5. [Chunk 521] Autonomous Agents - Page 35

   Query Time: 432ms
   ```

5. **Handle Response Quality**:

   Based on confidence score:
   - **High (>75%)**: Display answer as-is
   - **Medium (50-75%)**: Add warning "Moderate confidence"
   - **Low (<50%)**: Suggest query refinement

   Example low confidence message:
   ```
   ⚠️ Low Confidence (43.2%)

   The system found limited relevant information for your
   query. Try:
   - Being more specific
   - Using different terminology
   - Breaking complex questions into simpler parts
   - Checking if topic is covered in the book
   ```

6. **Error Handling**:

   - **Server not running**:
     ```
     ❌ Cannot connect to backend server
     → Run '/rag.start' to start the server
     ```

   - **Book not indexed**:
     ```
     ❌ Book content not found in database
     → Run '/rag.index' to index the book content
     ```

   - **API error**:
     ```
     ❌ API Error: {error_message}
     → Check server logs for details
     ```

## Usage Examples

```bash
# Simple query
/rag.query What is Physical AI?

# Multi-word query
/rag.query Explain the ROS 2 architecture and its components

# Query with context
/rag.query --context "sensor integration" How do robots process sensor data?

# Query specific book
/rag.query --book custom_book_id What is covered in chapter 3?

# Detailed output
/rag.query --verbose What are VLA models?
```

## Query Tips

**Good Queries**:
- ✓ "What is Physical AI?"
- ✓ "Explain ROS 2 nodes and topics"
- ✓ "How do VLA models work in robotics?"
- ✓ "What are the key components of a digital twin?"

**Poor Queries**:
- ✗ "Tell me everything" (too broad)
- ✗ "Chapter 1" (not a question)
- ✗ "Is this good?" (subjective, no context)
- ✗ Single words like "robotics" (too vague)

## Query Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| query | The question text | Required |
| book_id | Book identifier | physical_ai_textbook |
| user_selected_text | Context from selection | null |
| max_sources | Number of sources | 5 |
| confidence_threshold | Minimum confidence | 0.4 |

## Response Format

```json
{
  "answer": "Detailed answer text...",
  "sources": [
    "Source chunk 1",
    "Source chunk 2",
    "Source chunk 3"
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

## Advanced Usage

### Batch Queries
```bash
# Query multiple questions from file
cat questions.txt | while read query; do
  /rag.query "$query"
  echo "---"
done
```

### Query with Context
```bash
# Use previous answer as context for follow-up
/rag.query What is ROS 2?
# Then:
/rag.query --follow-up How does it differ from ROS 1?
```

### Export Results
```bash
# Save query results to file
/rag.query "What is Physical AI?" > query_results.txt
```

## Performance Metrics

Typical query breakdown:
- **Embedding**: 50-200ms (convert query to vector)
- **Search**: 50-150ms (find similar chunks in Qdrant)
- **Generation**: 200-500ms (generate answer with Cohere)
- **Total**: 300-800ms

Factors affecting speed:
- Query complexity
- Number of sources
- AI provider response time
- Network latency to Qdrant/Cohere

## Debugging Queries

If getting unexpected results:

1. **Check indexed content**:
   ```bash
   /rag.test --check-index
   ```

2. **View similar chunks**:
   ```bash
   /rag.query --show-chunks "your query"
   ```

3. **Test with simple query**:
   ```bash
   /rag.query What is this book about?
   ```

4. **Check confidence scores**:
   - High confidence but wrong answer → May need better content
   - Low confidence → Query may be too specific or off-topic

## Notes

- First query may be slower (cold start)
- Confidence scores are estimates, not guarantees
- Sources help verify answer accuracy
- Complex queries may need to be broken down
- Context from selected text can improve answers
- Query history is stored in Neon database
