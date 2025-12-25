import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from config.settings import settings

# Load environment variables
load_dotenv()

# Use the same settings as the RAG service
print(f"Qdrant URL: {settings.qdrant_url}")
print(f"Qdrant API Key available: {bool(settings.qdrant_api_key)}")
print(f"Collection Name: {settings.qdrant_collection_name}")

# Initialize Qdrant client the same way as the RAG service
if not settings.qdrant_url or not settings.qdrant_api_key:
    print("Error: Qdrant configuration not provided properly.")
else:
    try:
        qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False,
        )

        # Get collection info
        collection_info = qdrant_client.get_collection(settings.qdrant_collection_name)
        print(f"Collection '{settings.qdrant_collection_name}' exists.")
        print(f"Points count: {collection_info.points_count}")

        # Sample a few points to verify the structure
        points = qdrant_client.scroll(
            collection_name=settings.qdrant_collection_name,
            limit=3,
            with_payload=True,
            with_vectors=False
        )

        print("\nSample points:")
        for i, point in enumerate(points[0]):
            print(f"Point {i+1}:")
            print(f"  ID: {point.id}")
            print(f"  Payload keys: {list(point.payload.keys())}")
            if "book_id" in point.payload:
                print(f"  Book ID: {point.payload['book_id']}")
            print(f"  Content snippet: {point.payload['content'][:100]}...")
            print()

    except Exception as e:
        print(f"Error accessing collection: {e}")