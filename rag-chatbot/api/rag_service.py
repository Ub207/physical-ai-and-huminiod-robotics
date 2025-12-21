import asyncio
import uuid
from typing import List, Dict, Any
from utils.cohere_client import CohereClient
from utils.vector_store import VectorStore
from utils.text_processor import TextProcessor
from utils.database import DatabaseManager
from models.schemas import QueryRequest, QueryResponse, IngestionRequest, Chunk
from config.settings import settings
import logging

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        self.cohere_client = CohereClient()
        self.vector_store = VectorStore()
        self.db_manager = DatabaseManager()

    async def ingest_book(self, request: IngestionRequest) -> Dict[str, Any]:
        """
        Process and ingest a book into the vector database.
        """
        logger.info(f"Starting ingestion for book: {request.book_id}")

        try:
            # Determine file type and process accordingly
            import os
            file_ext = os.path.splitext(request.file_path)[1].lower()

            if file_ext == '.pdf':
                pages = TextProcessor.extract_pages_from_pdf(request.file_path)
            elif file_ext == '.epub':
                pages = TextProcessor.extract_chapters_from_epub(request.file_path)
            elif file_ext == '.txt':
                pages = TextProcessor.extract_from_txt(request.file_path)
            else:
                raise ValueError(f"Unsupported file type: {file_ext}")

            # Process each page/chapter into chunks
            all_chunks = []
            total_chunks = 0

            for page_num, page_text in pages:
                # Split page text into chunks
                text_chunks = TextProcessor.chunk_text(
                    page_text,
                    chunk_size=request.chunk_size,
                    chunk_overlap=request.chunk_overlap
                )

                for chunk_text in text_chunks:
                    # Create embedding for the chunk
                    embeddings = self.cohere_client.embed_texts([chunk_text], input_type="search_document")
                    embedding = embeddings[0] if embeddings else []

                    # Create chunk object
                    chunk = Chunk(
                        id=str(uuid.uuid4()),
                        book_id=request.book_id,
                        page_number=page_num,
                        content=chunk_text,
                        embedding=embedding,
                        metadata={
                            "title": request.title,
                            "author": request.author
                        }
                    )
                    all_chunks.append(chunk)
                    total_chunks += 1

            # Add all chunks to vector store
            self.vector_store.add_chunks(all_chunks)

            logger.info(f"Successfully ingested book: {request.book_id} with {total_chunks} chunks")
            return {
                "book_id": request.book_id,
                "title": request.title,
                "status": "completed",
                "total_chunks": total_chunks,
                "message": f"Successfully ingested book with {total_chunks} chunks"
            }

        except Exception as e:
            logger.error(f"Error ingesting book {request.book_id}: {str(e)}")
            raise e

    async def query(self, request: QueryRequest) -> QueryResponse:
        """
        Query the RAG system with a question about the book.
        Can optionally include user-selected text for ad-hoc queries.
        """
        logger.info(f"Processing query for book: {request.book_id}")

        try:
            # Generate embedding for the query
            query_texts = [request.query]
            if request.user_selected_text:
                query_texts.append(request.user_selected_text)

            query_embeddings = self.cohere_client.embed_texts(query_texts, input_type="search_query")
            query_embedding = query_embeddings[0]  # Use the main query for search

            # Search for relevant chunks in the vector store
            relevant_chunks = self.vector_store.search(
                query_embedding=query_embedding,
                book_id=request.book_id,
                limit=5  # Retrieve top 5 most relevant chunks
            )

            # Build context from retrieved chunks
            context_parts = []
            for chunk in relevant_chunks:
                context_parts.append(f"Page {chunk['page_number']}: {chunk['content']}")

            context = "\n\n".join(context_parts)

            # Generate response using Cohere
            if request.user_selected_text:
                answer = self.cohere_client.chat(
                    message=request.query,
                    context=context,
                    selected_text=request.user_selected_text
                )
            else:
                answer = self.cohere_client.chat(
                    message=request.query,
                    context=context
                )

            # Format sources
            sources = []
            for chunk in relevant_chunks:
                sources.append({
                    "page": chunk["page_number"],
                    "content_snippet": chunk["content"][:200] + "..." if len(chunk["content"]) > 200 else chunk["content"],
                    "relevance_score": chunk["score"]
                })

            # Calculate tokens used (approximate)
            tokens_used = len(answer.split())

            # Store the conversation in the database (async operation)
            # For now, we'll use a simple session ID based on book_id
            session_id = f"{request.book_id}_default_session"
            self.db_manager.add_message(session_id, "user", request.query)
            self.db_manager.add_message(session_id, "assistant", answer)

            return QueryResponse(
                answer=answer,
                sources=sources,
                confidence=min(1.0, max(0.0, relevant_chunks[0]["score"] if relevant_chunks else 0.5)),
                tokens_used=tokens_used
            )

        except Exception as e:
            logger.error(f"Error processing query for book {request.book_id}: {str(e)}")
            raise e