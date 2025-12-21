from utils.vector_store import VectorStore
from utils.cohere_client import CohereClient
from config.settings import settings

print("Debugging VectorStore functionality...")

# Initialize the same way as RAG service
vector_store = VectorStore()
cohere_client = CohereClient()

if vector_store.client is None:
    print("ERROR: VectorStore client is None - check Qdrant configuration")
    print(f"Qdrant URL: {settings.qdrant_url}")
    print(f"Qdrant API Key set: {bool(settings.qdrant_api_key)}")
    print(f"Collection name: {settings.qdrant_collection_name}")
else:
    print("VectorStore client initialized successfully")
    print(f"Collection name: {vector_store.collection_name}")

    # Test if collection exists
    try:
        collection_info = vector_store.client.get_collection(vector_store.collection_name)
        print(f"Collection exists with {collection_info.points_count} points")
    except Exception as e:
        print(f"Error accessing collection: {e}")

    # Generate a test query embedding
    query_text = "What is this book about?"
    query_embeddings = cohere_client.embed_texts([query_text], input_type="search_query")
    query_embedding = query_embeddings[0] if query_embeddings else []

    print(f"Generated query embedding of length: {len(query_embedding)}")

    # Test search function directly
    print("Testing search function...")
    try:
        results = vector_store.search(query_embedding, "physical_ai_textbook", limit=5)
        print(f"Search returned {len(results)} results")

        if results:
            print("SUCCESS: Found results!")
            for i, result in enumerate(results):
                print(f"Result {i+1}: Score={result['score']}, Page={result.get('page_number', 'N/A')}")
                print(f"  Content: {result['content'][:100]}...")
        else:
            print("No results found - checking if the issue is with book_id or search filter...")

            # Let's try to get a sample of points to see the exact structure
            try:
                sample_points = vector_store.client.scroll(
                    collection_name=vector_store.collection_name,
                    limit=2,
                    with_payload=True,
                    with_vectors=False
                )

                print(f"\nSample points from collection:")
                for i, (point, _) in enumerate(sample_points):
                    print(f"Point {i+1}: ID={point.id}")
                    print(f"  Payload keys: {list(point.payload.keys())}")
                    print(f"  Book ID in payload: {point.payload.get('book_id', 'NOT FOUND')}")
                    print(f"  Content snippet: {point.payload.get('content', '')[:100]}...")
                    print()

            except Exception as e:
                print(f"Error getting sample points: {e}")

    except Exception as e:
        print(f"Error during search: {e}")
        import traceback
        traceback.print_exc()