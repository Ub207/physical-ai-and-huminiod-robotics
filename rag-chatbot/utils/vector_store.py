import qdrant_client
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from config.settings import settings
from models.schemas import Chunk
import logging

logger = logging.getLogger(__name__)

class VectorStore:
    def __init__(self):
        # Check if Qdrant settings are provided
        if not settings.qdrant_url or not settings.qdrant_api_key:
            logger.warning("Qdrant configuration not provided. Vector store will not function.")
            self.client = None
            self.collection_name = settings.qdrant_collection_name
            return

        try:
            self.client = qdrant_client.QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
            )
            self.collection_name = settings.qdrant_collection_name
            self._ensure_collection_exists()
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            self.client = None

    def _ensure_collection_exists(self):
        """
        Ensure the collection exists in Qdrant.
        """
        try:
            if self.client:
                self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            if self.client:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=settings.embedding_size,  # Cohere embed-english-v3.0 returns 1024-dim vectors
                        distance=models.Distance.COSINE
                    )
                )
                # Create payload index for book_id to enable filtering
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="book_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

    def add_chunks(self, chunks: List[Chunk]):
        """
        Add chunks to the vector store.
        """
        if not self.client:
            logger.warning("Qdrant client not available. Skipping chunk addition.")
            return

        try:
            points = []
            for chunk in chunks:
                points.append(models.PointStruct(
                    id=chunk.id,
                    vector=chunk.embedding,
                    payload={
                        "book_id": chunk.book_id,
                        "page_number": chunk.page_number,
                        "content": chunk.content,
                        "metadata": chunk.metadata
                    }
                ))

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
        except Exception as e:
            logger.error(f"Failed to add chunks to vector store: {e}")

    def search(self, query_embedding: List[float], book_id: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar chunks in the vector store for a specific book.
        """
        if not self.client:
            logger.warning("Qdrant client not available. Returning empty results.")
            return []

        try:
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                query_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="book_id",
                            match=models.MatchValue(value=book_id)
                        )
                    ]
                ),
                limit=limit
            )

            return [
                {
                    "id": result.id,
                    "content": result.payload["content"],
                    "page_number": result.payload.get("page_number", 1),  # Default to 1 if not present
                    "score": result.score,
                    "metadata": result.payload.get("metadata", {})
                }
                for result in results.points  # Note: results.points in the new API
            ]
        except Exception as e:
            logger.error(f"Failed to search vector store: {e}")
            return []

    def delete_book(self, book_id: str):
        """
        Delete all chunks for a specific book.
        """
        if not self.client:
            logger.warning("Qdrant client not available. Skipping book deletion.")
            return

        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.FilterSelector(
                    filter=models.Filter(
                        must=[
                            models.FieldCondition(
                                key="book_id",
                                match=models.MatchValue(value=book_id)
                            )
                        ]
                    )
                )
            )
        except Exception as e:
            logger.error(f"Failed to delete book from vector store: {e}")