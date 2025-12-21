import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from config.settings import settings
import cohere

# Load environment variables
load_dotenv()

# Initialize Cohere client to generate a real embedding
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
cohere_client = cohere.Client(COHERE_API_KEY)

# Initialize Qdrant client the same way as the RAG service
if not settings.qdrant_url or not settings.qdrant_api_key:
    print("Error: Qdrant configuration not provided properly.")
else:
    try:
        qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )

        # Generate a test query embedding (like the RAG service would)
        query_text = "What is this book about?"
        response = cohere_client.embed(
            texts=[query_text],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        # The response.embeddings is already a list of lists
        query_embedding = response.embeddings[0]  # Use the main query for search

        print(f"Query: {query_text}")
        print(f"Query embedding length: {len(query_embedding)}")

        # Perform the exact same search as in vector_store.py
        results = qdrant_client.search(
            collection_name=settings.qdrant_collection_name,
            query_vector=query_embedding,
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
            print("SUCCESS! Found relevant chunks:")
            for i, result in enumerate(results):
                print(f"Result {i+1}:")
                print(f"  ID: {result.id}")
                print(f"  Score: {result.score}")
                print(f"  Content snippet: {result.payload['content'][:150]}...")
                print()
        else:
            print("No results found. Let's try a broader search without book_id filter:")

            # Search without filter to see if the collection is accessible at all
            results_all = qdrant_client.search(
                collection_name=settings.qdrant_collection_name,
                query_vector=query_embedding,
                limit=5
            )

            print(f"Results without filter: {len(results_all)}")
            if results_all:
                print("Results without book_id filter:")
                for i, result in enumerate(results_all[:2]):
                    print(f"Result {i+1}:")
                    print(f"  ID: {result.id}")
                    print(f"  Score: {result.score}")
                    print(f"  Book ID in payload: {result.payload.get('book_id', 'NOT FOUND')}")
                    print(f"  Content snippet: {result.payload['content'][:150]}...")
                    print()
            else:
                print("Even without filter, no results found - there may be an issue with the collection or search setup.")

    except Exception as e:
        print(f"Error during search test: {e}")
        import traceback
        traceback.print_exc()