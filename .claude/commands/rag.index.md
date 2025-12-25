---
description: Index book content into the RAG chatbot vector database (Qdrant)
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

This command indexes book content into the Qdrant vector database for the RAG chatbot system.

1. **Check Prerequisites**:
   - Verify `.env` file exists in `rag-chatbot/` directory
   - Confirm required environment variables are set:
     - `QDRANT_URL`
     - `QDRANT_API_KEY`
     - `COHERE_API_KEY` or `GEMINI_API_KEY` (depending on AI_PROVIDER)
   - If any are missing, prompt user to add them

2. **Locate Book Content**:
   - Default location: `rag-chatbot/physical_ai_textbook.txt`
   - If user provided a file path in arguments, use that instead
   - Verify the file exists and is readable
   - Supported formats: TXT, PDF, EPUB

3. **Run Indexing Process**:
   ```bash
   cd rag-chatbot && python convert_docs.py
   ```

   This script will:
   - Connect to Qdrant cloud instance
   - Read the book content
   - Split text into chunks (overlap for context)
   - Generate embeddings using Cohere or Gemini
   - Store vectors in Qdrant collection
   - Create payload index for book_id field

4. **Verify Indexing**:
   - Check the output for number of chunks indexed
   - Verify no errors occurred
   - Test with a sample query:
   ```bash
   python test_query.py
   ```

5. **Report Results**:
   - Number of chunks indexed
   - Collection name used
   - Book ID assigned
   - Any warnings or errors encountered

## Usage Examples

```bash
# Index default book content
/rag.index

# Index specific file
/rag.index path/to/custom_book.txt

# Force re-index (recreate collection)
/rag.index --force
```

## Error Handling

- **Missing API keys**: Guide user to add keys in `.env` file
- **File not found**: List available files in directory
- **Qdrant connection error**: Check URL and API key validity
- **Embedding API error**: Verify AI provider API key and quota
- **Duplicate collection**: Offer to delete and recreate or append

## Notes

- Indexing may take several minutes for large books
- Embeddings are 1024-dimensional vectors
- Default chunk size: 500 characters with 50 character overlap
- Collection name: `book_chunks` (configurable in settings)
