---
description: Setup and configure the RAG chatbot environment
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

This command sets up the complete RAG chatbot environment from scratch.

1. **Check Current Setup**:
   - Verify Python version (3.11+ required)
   - Check if virtual environment exists
   - Verify `rag-chatbot/` directory structure
   - List existing configuration files

2. **Virtual Environment Setup**:
   ```bash
   cd rag-chatbot

   # Check if venv exists
   if [ ! -d "venv" ]; then
     python -m venv venv
   fi

   # Activate virtual environment
   # Windows:
   venv\Scripts\activate
   # Linux/Mac:
   source venv/bin/activate
   ```

3. **Install Dependencies**:
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

   Key dependencies:
   - FastAPI & Uvicorn (web framework)
   - Cohere SDK (embeddings & generation)
   - Qdrant Client (vector database)
   - Psycopg2 (PostgreSQL client)
   - PDFPlumber, EbookLib (document processing)

4. **Environment Configuration**:

   a. Check if `.env` file exists:
   ```bash
   if [ ! -f ".env" ]; then
     cp .env.example .env
   fi
   ```

   b. Guide user to fill required variables:
   ```
   Required Configuration:
   ========================

   AI Provider (choose one):
   - COHERE_API_KEY=your_cohere_key_here
   - GEMINI_API_KEY=your_gemini_key_here
   - AI_PROVIDER=cohere or gemini

   Vector Database:
   - QDRANT_URL=https://xxx.cloud.qdrant.io
   - QDRANT_API_KEY=your_qdrant_key
   - QDRANT_COLLECTION_NAME=book_chunks

   Relational Database:
   - NEON_DATABASE_URL=postgresql://user:pass@host/db

   Optional:
   - UPLOAD_FOLDER=uploads
   - MAX_CONTENT_LENGTH=16777216
   ```

   c. Validate each required variable:
   - Check if variable is set and non-empty
   - Test API connections if possible
   - Mark ✓ for configured, ✗ for missing

5. **Database Setup**:

   a. **Qdrant Setup**:
   - Test connection to Qdrant cloud
   - List existing collections
   - Create `book_chunks` collection if needed
   - Set up payload index for `book_id`

   b. **Neon PostgreSQL Setup**:
   - Test database connection
   - Create tables if needed (conversations, messages)
   - Verify schema

6. **Verify Installation**:
   ```bash
   # Test imports
   python -c "
   from api.main import app
   from utils.cohere_client import CohereClient
   from qdrant_client import QdrantClient
   print('All imports successful!')
   "
   ```

7. **Create Configuration Summary**:
   ```
   RAG Chatbot Setup Complete!
   ====================================

   ✓ Python: 3.11.5
   ✓ Virtual Environment: venv/
   ✓ Dependencies: 24 packages installed
   ✓ AI Provider: Cohere (command-r-plus)
   ✓ Vector DB: Qdrant (https://xxx.cloud.qdrant.io)
   ✓ SQL DB: Neon PostgreSQL
   ✓ Configuration: .env file ready

   Next Steps:
   -----------
   1. Index book content: /rag.index
   2. Start backend server: /rag.start
   3. Test the system: /rag.test

   Documentation:
   - API Docs: http://localhost:8001/docs (after /rag.start)
   - README: rag-chatbot/README.md
   ```

## Usage Examples

```bash
# Full setup from scratch
/rag.setup

# Setup with specific AI provider
/rag.setup --provider cohere

# Skip dependency installation
/rag.setup --skip-deps

# Verify existing setup
/rag.setup --verify
```

## Verification Checklist

After setup, verify:
- [ ] Virtual environment created and activated
- [ ] All dependencies installed (no errors)
- [ ] `.env` file exists with all required variables
- [ ] Qdrant connection successful
- [ ] Neon database connection successful
- [ ] Test imports work without errors
- [ ] Collection exists in Qdrant (or ready to create)

## Troubleshooting

### Python Version
```bash
python --version
# Should be 3.11 or higher
```

### Virtual Environment Issues
```bash
# Windows: If activation fails
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser

# Recreate venv
rm -rf venv
python -m venv venv
```

### Dependency Installation Errors
```bash
# Upgrade pip first
pip install --upgrade pip setuptools wheel

# Install with verbose output
pip install -r requirements.txt -v
```

### API Key Validation
```bash
# Test Cohere
python -c "import cohere; co = cohere.Client('your-key'); print(co.models.list())"

# Test Qdrant
python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='your-url', api_key='your-key'); print(client.get_collections())"
```

## Directory Structure

After setup, you should have:
```
rag-chatbot/
├── api/
│   ├── main.py           # FastAPI app
│   └── rag_service.py    # RAG logic
├── config/
│   └── settings.py       # Configuration
├── utils/
│   ├── cohere_client.py  # Cohere integration
│   ├── gemini_client.py  # Gemini integration
│   ├── vector_store.py   # Qdrant operations
│   └── text_processor.py # Text chunking
├── venv/                 # Virtual environment
├── .env                  # Environment variables
├── .env.example          # Template
├── requirements.txt      # Dependencies
└── README.md            # Documentation
```

## Notes

- Setup should be run once per environment
- Keep `.env` file secure (never commit to git)
- Virtual environment isolates dependencies
- Test configuration before indexing content
- API keys should have appropriate permissions:
  - Cohere: API access enabled
  - Qdrant: Read + Write access
  - Neon: Connection and query permissions
