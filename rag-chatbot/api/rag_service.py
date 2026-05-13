import asyncio
import uuid
from typing import List, Dict, Any, Optional
from utils.client_factory import AIClientFactory
from utils.vector_store import VectorStore
from utils.text_processor import TextProcessor
from utils.database import DatabaseManager
from models.schemas import QueryRequest, QueryResponse, IngestionRequest, Chunk
from config.settings import settings
import logging

logger = logging.getLogger(__name__)

_LANG_CODES = {
    "ur": "ur",
    "urdu": "ur",
    "es": "es",
    "spanish": "es",
    "fr": "fr",
    "french": "fr",
    "de": "de",
    "german": "de",
    "ar": "ar",
    "arabic": "ar",
    "zh": "zh-CN",
    "chinese": "zh-CN",
    "hi": "hi",
    "hindi": "hi",
}


def _google_translate(text: str, target_lang: str) -> str:
    from deep_translator import GoogleTranslator
    lang_code = _LANG_CODES.get(target_lang.lower(), target_lang)
    translated = GoogleTranslator(source="auto", target=lang_code).translate(text)
    return translated or text


class RAGService:
    def __init__(self):
        self.ai_client = AIClientFactory.create_client()
        self.vector_store = VectorStore()
        self.db_manager = DatabaseManager()

    async def translate_text(self, text: str, target_language: str) -> str:
        try:
            # Try Cohere first
            prompt = (
                f"Translate the following text to {target_language}. "
                f"Return ONLY the translated text, no explanations or notes.\n\n"
                f"Text to translate:\n{text}"
            )
            result = self.ai_client.generate_response(prompt, max_tokens=2000)
            if result and result.strip():
                return result.strip()
        except Exception as e:
            logger.warning(f"Cohere translation failed ({str(e)[:80]}), falling back to Google Translate")

        # Silent fallback to Google Translate (deep-translator)
        try:
            return _google_translate(text, target_language)
        except Exception as e:
            logger.error(f"Google Translate fallback also failed: {str(e)}")
            return text  # Return original text if all translation fails

    async def ingest_book(self, request: IngestionRequest) -> Dict[str, Any]:
        logger.info(f"Starting ingestion for book: {request.book_id}")

        try:
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

            all_chunks = []
            total_chunks = 0

            for page_num, page_text in pages:
                text_chunks = TextProcessor.chunk_text(
                    page_text,
                    chunk_size=request.chunk_size,
                    chunk_overlap=request.chunk_overlap
                )

                for chunk_text in text_chunks:
                    embeddings = self.ai_client.embed_texts([chunk_text], input_type="search_document")
                    embedding = embeddings[0] if embeddings else []

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
        logger.info(f"Processing query for book: {request.book_id}")

        try:
            query_texts = [request.query]
            if request.user_selected_text:
                query_texts.append(request.user_selected_text)

            query_embeddings = self.ai_client.embed_texts(query_texts, input_type="search_query")
            query_embedding = query_embeddings[0]

            relevant_chunks = self.vector_store.search(
                query_embedding=query_embedding,
                book_id=request.book_id,
                limit=5
            )

            context_parts = []
            for chunk in relevant_chunks:
                context_parts.append(f"Page {chunk['page_number']}: {chunk['content']}")

            context = "\n\n".join(context_parts)

            if not context.strip():
                context = ""

            answer = ""

            # Tier 1: Primary AI client (Cohere)
            try:
                if request.user_selected_text:
                    answer = self.ai_client.chat(
                        message=request.query,
                        context=context,
                        selected_text=request.user_selected_text
                    )
                else:
                    answer = self.ai_client.chat(
                        message=request.query,
                        context=context
                    )
                if answer:
                    logger.info("Answer generated via primary AI client")
            except Exception as e:
                logger.warning(f"Primary AI client failed ({str(e)[:80]})")

            # Tier 2: Groq fallback (free — 500 req/day, very fast, llama-3.3-70b)
            if not answer:
                try:
                    from utils.groq_client import GroqClient
                    if settings.groq_api_key:
                        groq_client = GroqClient()
                        answer = groq_client.chat(
                            message=request.query,
                            context=context,
                            selected_text=request.user_selected_text or ""
                        )
                        if answer:
                            logger.info("Answer generated via Groq fallback")
                except Exception as ge:
                    logger.warning(f"Groq fallback failed ({str(ge)[:80]})")

            # Tier 3: Gemini fallback
            if not answer:
                try:
                    from utils.gemini_client import GeminiClient
                    if settings.gemini_api_key:
                        gemini_client = GeminiClient()
                        answer = gemini_client.chat(
                            message=request.query,
                            context=context,
                            selected_text=request.user_selected_text or ""
                        )
                        if answer:
                            logger.info("Answer generated via Gemini fallback")
                except Exception as gme:
                    logger.warning(f"Gemini fallback failed ({str(gme)[:80]})")

            # Tier 4: Raw chunk text as last resort
            if not answer:
                if request.query.lower() in ["hi", "hello", "hey"]:
                    answer = "Hello! I'm your AI assistant for the Physical AI and Humanoid Robotics textbook. How can I help you?"
                elif relevant_chunks:
                    answer = (
                        f"Here is relevant information from the textbook about your question:\n\n"
                        + "\n\n".join(context_parts[:2])
                    )
                else:
                    answer = (
                        "I'm having trouble generating a response right now. "
                        "Please try rephrasing your question about the textbook content."
                    )

            # Format sources
            sources = []
            for chunk in relevant_chunks:
                sources.append({
                    "page": chunk["page_number"],
                    "content_snippet": chunk["content"][:200] + "..." if len(chunk["content"]) > 200 else chunk["content"],
                    "relevance_score": chunk["score"]
                })

            tokens_used = len(answer.split()) if answer else 0

            session_id = f"{request.book_id}_default_session"
            self.db_manager.add_message(session_id, "user", request.query)
            self.db_manager.add_message(session_id, "assistant", answer)

            confidence = 0.5
            if relevant_chunks:
                confidence = min(1.0, max(0.0, relevant_chunks[0]["score"]))

            return QueryResponse(
                answer=answer,
                sources=sources,
                confidence=confidence,
                tokens_used=tokens_used
            )

        except Exception as e:
            logger.error(f"Error processing query for book {request.book_id}: {str(e)}")
            raise e
