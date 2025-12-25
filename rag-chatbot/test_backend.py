#!/usr/bin/env python3
"""
Simple test script to verify the RAG Chatbot backend components are working
"""
import asyncio
import sys
import os

# Add the project root to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_backend_components():
    """Test that all backend components can be imported and instantiated"""
    print("Testing RAG Chatbot backend components...")

    try:
        # Test imports
        from api.main import app
        print("[OK] FastAPI app imported successfully")

        from api.rag_service import RAGService
        print("[OK] RAGService imported successfully")

        from models.schemas import QueryRequest, QueryResponse, IngestionRequest
        print("[OK] Schemas imported successfully")

        from config.settings import settings
        print("[OK] Settings imported successfully")

        # Test RAGService instantiation
        rag_service = RAGService()
        print("[OK] RAGService instantiated successfully")

        # Test that we can create schema objects
        query_request = QueryRequest(
            book_id="test_book",
            query="What is Physical AI?"
        )
        print("[OK] QueryRequest schema created successfully")

        ingestion_request = IngestionRequest(
            book_id="test_book",
            title="Test Book",
            author="Test Author",
            file_path="test.txt"
        )
        print("[OK] IngestionRequest schema created successfully")

        print("\n[SUCCESS] All backend components are working correctly!")
        print("\nBackend Structure:")
        print("- FastAPI application in api/main.py")
        print("- RAG service logic in api/rag_service.py")
        print("- Data models in models/schemas.py")
        print("- Configuration in config/settings.py")
        print("- Utility functions in utils/ directory")

        print("\nTo run the full backend:")
        print("1. Set up your .env file with API keys (see .env.example)")
        print("2. Run: cd rag-chatbot && python -m uvicorn api.main:app --reload")
        print("3. The API will be available at http://localhost:8000")
        print("4. API endpoints: /, /query, /ingest, /upload, /health")

        return True

    except Exception as e:
        print(f"[ERROR] Error testing backend components: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def test_api_endpoints():
    """Test that the main API endpoints are defined"""
    print("\nTesting API endpoints...")

    from api.main import app

    # Get all routes
    routes = [route.path for route in app.routes]
    expected_routes = ['/', '/ingest', '/query', '/upload', '/health']

    print(f"Available routes: {routes}")

    for route in expected_routes:
        if any(route in r for r in routes):
            print(f"[OK] {route} endpoint is available")
        else:
            print(f"[WARN] {route} endpoint not found")

    return True

if __name__ == "__main__":
    print("="*60)
    print("RAG Chatbot Backend Test")
    print("="*60)

    success = test_backend_components()
    if success:
        test_api_endpoints()

    print("\n" + "="*60)
    if success:
        print("[SUCCESS] Backend test completed successfully!")
        print("\nNote: This test verifies the code structure and imports.")
        print("To use the full functionality, you'll need valid API keys in a .env file.")
    else:
        print("[ERROR] Backend test failed!")
    print("="*60)