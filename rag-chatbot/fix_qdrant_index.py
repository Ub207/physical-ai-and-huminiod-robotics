#!/usr/bin/env python3
"""
Fix Qdrant index for book_id field
"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables
load_dotenv()

# Initialize Qdrant client
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")

print(f"Connecting to Qdrant Cloud at {QDRANT_URL}...")
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY, prefer_grpc=False)

try:
    # Create payload index for book_id to enable filtering
    print(f"Creating payload index for 'book_id' field in collection '{COLLECTION_NAME}'...")
    qdrant_client.create_payload_index(
        collection_name=COLLECTION_NAME,
        field_name="book_id",
        field_schema=models.PayloadSchemaType.KEYWORD
    )
    print("[OK] Index created successfully!")

    # Verify the collection info
    collection_info = qdrant_client.get_collection(collection_name=COLLECTION_NAME)
    print(f"\nCollection info:")
    print(f"  - Name: {COLLECTION_NAME}")
    print(f"  - Points count: {collection_info.points_count}")
    print(f"  - Index created for 'book_id' field")

except Exception as e:
    print(f"Error: {e}")
