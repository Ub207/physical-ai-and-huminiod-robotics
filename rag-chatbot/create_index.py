import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from config.settings import settings

# Load environment variables
load_dotenv()

# Initialize Qdrant client the same way as the RAG service
if not settings.qdrant_url or not settings.qdrant_api_key:
    print("Error: Qdrant configuration not provided properly.")
else:
    try:
        qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )

        print(f"Creating payload index for book_id in collection: {settings.qdrant_collection_name}")

        # Create payload index for book_id to enable filtering
        qdrant_client.create_payload_index(
            collection_name=settings.qdrant_collection_name,
            field_name="book_id",
            field_schema=models.PayloadSchemaType.KEYWORD
        )

        print("Payload index created successfully!")

        # Verify the index was created by checking collection info
        collection_info = qdrant_client.get_collection(settings.qdrant_collection_name)
        print(f"Collection '{settings.qdrant_collection_name}' has {collection_info.points_count} points")

    except Exception as e:
        print(f"Error creating payload index: {e}")
        import traceback
        traceback.print_exc()