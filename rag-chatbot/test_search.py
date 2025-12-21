import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from config.settings import settings

# Load environment variables
load_dotenv()

# Use the same settings as the RAG service
print(f"Collection Name: {settings.qdrant_collection_name}")

# Initialize Qdrant client the same way as the RAG service
if not settings.qdrant_url or not settings.qdrant_api_key:
    print("Error: Qdrant configuration not provided properly.")
else:
    try:
        qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )

        # Test search with the same parameters as the RAG service
        # Create a test embedding (using a simple array for testing)
        test_embedding = [0.1] * 1024  # Same dimension as Cohere embeddings

        # Search with book_id filter
        results = qdrant_client.search(
            collection_name=settings.qdrant_collection_name,
            query_vector=test_embedding,
            query_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="book_id",
                        match=models.MatchValue(value="physical_ai_textbook")
                    )
                ]
            ),
            limit=5
        )

        print(f"Search results count: {len(results)}")
        if results:
            print("Sample result:")
            for i, result in enumerate(results[:2]):
                print(f"Result {i+1}:")
                print(f"  ID: {result.id}")
                print(f"  Score: {result.score}")
                print(f"  Payload keys: {list(result.payload.keys())}")
                print(f"  Content snippet: {result.payload['content'][:100]}...")
                print()
        else:
            print("No results found with the search filter. Let's try without the filter:")

            # Search without filter to see if the collection is accessible
            results_all = qdrant_client.search(
                collection_name=settings.qdrant_collection_name,
                query_vector=test_embedding,
                limit=5
            )

            print(f"Search results without filter: {len(results_all)}")
            if results_all:
                print("Sample result without filter:")
                for i, result in enumerate(results_all[:2]):
                    print(f"Result {i+1}:")
                    print(f"  ID: {result.id}")
                    print(f"  Score: {result.score}")
                    print(f"  Payload keys: {list(result.payload.keys())}")
                    if "book_id" in result.payload:
                        print(f"  Book ID: {result.payload['book_id']}")
                    print(f"  Content snippet: {result.payload['content'][:100]}...")
                    print()

    except Exception as e:
        print(f"Error during search test: {e}")
        import traceback
        traceback.print_exc()