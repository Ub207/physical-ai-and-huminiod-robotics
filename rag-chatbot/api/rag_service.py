import asyncio
import uuid
from typing import List, Dict, Any
from utils.client_factory import AIClientFactory
from utils.vector_store import VectorStore
from utils.text_processor import TextProcessor
from utils.database import DatabaseManager
from models.schemas import QueryRequest, QueryResponse, IngestionRequest, Chunk, TranslationRequest, TranslationResponse
from config.settings import settings
import logging

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        self.ai_client = AIClientFactory.create_client()
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
                    embeddings = self.ai_client.embed_texts([chunk_text], input_type="search_document")
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

            query_embeddings = self.ai_client.embed_texts(query_texts, input_type="search_query")
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

            # If no relevant context is found, set a default context
            if not context.strip():
                context = "You are an AI assistant for a Physical AI and Humanoid Robotics textbook. The user has asked a general question that is not directly related to specific textbook content."

            # Generate response — try primary AI, then Gemini fallback, then raw text
            answer = None

            # Primary: configured AI client (Cohere)
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
                logger.info("Answer generated via primary AI (Cohere)")
            except Exception as e:
                logger.warning(f"Primary AI failed ({str(e)[:80]}), trying Gemini fallback")

            # Fallback 1: Groq (free — 500 req/day, very fast, llama-3.1-70b)
            if not answer:
                try:
                    from utils.groq_client import GroqClient
                    from config.settings import settings as _s
                    if _s.groq_api_key:
                        groq_client = GroqClient()
                        answer = groq_client.chat(
                            message=request.query,
                            context=context,
                            selected_text=request.user_selected_text or ""
                        )
                        logger.info("Answer generated via Groq fallback")
                    else:
                        logger.warning("GROQ_API_KEY not set — skipping Groq fallback")
                except Exception as ge:
                    logger.warning(f"Groq fallback failed ({str(ge)[:80]})")

            # Fallback 2: Gemini Flash (free — 1500 req/day)
            if not answer:
                try:
                    from utils.gemini_client import GeminiClient
                    from config.settings import settings as _s
                    if _s.gemini_api_key:
                        gemini = GeminiClient()
                        answer = gemini.chat(
                            message=request.query,
                            context=context,
                            selected_text=request.user_selected_text or ""
                        )
                        logger.info("Answer generated via Gemini fallback")
                    else:
                        logger.warning("GEMINI_API_KEY not set — skipping Gemini fallback")
                except Exception as ge:
                    logger.warning(f"Gemini fallback failed ({str(ge)[:80]})")

            # Fallback 2: show raw chunks (no LLM available)
            if not answer:
                q_lower = request.query.lower().strip()
                if q_lower in ["hi", "hello", "hey", "salam", "السلام"]:
                    answer = "Hello! I'm your AI assistant for the Physical AI and Humanoid Robotics textbook. How can I help you?"
                elif relevant_chunks:
                    lines = [f"Here is what the textbook says about **\"{request.query}\"**:\n"]
                    for chunk in relevant_chunks[:3]:
                        lines.append(f"**📖 Page {chunk['page_number']}:**\n{chunk['content'].strip()}\n")
                    answer = "\n".join(lines)
                else:
                    answer = f"I could not find information about '{request.query}' in the textbook. Please make sure the book has been indexed, or try a different question."

            # Format sources
            sources = []
            for chunk in relevant_chunks:
                sources.append({
                    "page": chunk["page_number"],
                    "content_snippet": chunk["content"][:200] + "..." if len(chunk["content"]) > 200 else chunk["content"],
                    "relevance_score": chunk["score"]
                })

            # Calculate tokens used (approximate)
            tokens_used = len(answer.split()) if answer else 0

            # Store the conversation in the database (async operation)
            # For now, we'll use a simple session ID based on book_id
            session_id = f"{request.book_id}_default_session"
            self.db_manager.add_message(session_id, "user", request.query)
            self.db_manager.add_message(session_id, "assistant", answer)

            # Calculate confidence based on relevance scores
            confidence = 0.5  # Default confidence
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

    # Maps our language names to Google Translate language codes
    _LANG_CODES = {
        'urdu': 'ur',
        'arabic': 'ar',
        'french': 'fr',
        'spanish': 'es',
        'german': 'de',
        'chinese': 'zh-CN',
        'hindi': 'hi',
    }

    def _google_translate(self, text: str, target_lang: str) -> str:
        """Free fallback translation using Google Translate via deep-translator."""
        from deep_translator import GoogleTranslator
        lang_code = self._LANG_CODES.get(target_lang, target_lang)
        # Google Translate has a 5000-char limit per call; split if needed
        if len(text) <= 4900:
            return GoogleTranslator(source='auto', target=lang_code).translate(text)
        # Split on sentence boundaries and translate in chunks
        import re as _re
        sentences = _re.split(r'(?<=[.!?])\s+', text)
        chunks, current = [], ''
        for s in sentences:
            if len(current) + len(s) + 1 <= 4900:
                current = (current + ' ' + s).strip()
            else:
                if current:
                    chunks.append(current)
                current = s
        if current:
            chunks.append(current)
        translated_parts = [
            GoogleTranslator(source='auto', target=lang_code).translate(chunk)
            for chunk in chunks
        ]
        return ' '.join(translated_parts)

    async def translate(self, request: TranslationRequest) -> TranslationResponse:
        """Translate text to target language (Cohere primary, Google Translate fallback)."""
        logger.info(f"Translating {len(request.text)} characters to {request.target_language}")

        import re as _re

        # Strip HTML tags — send only plain text to the AI
        plain_text = _re.sub(r'<[^>]+>', ' ', request.text)
        plain_text = _re.sub(r'\s+', ' ', plain_text).strip()

        # Cap at 3000 chars to stay within token limits
        if len(plain_text) > 3000:
            plain_text = plain_text[:3000]

        target_lang = request.target_language.lower()
        translated = None

        # --- Primary: Cohere AI ---
        try:
            if target_lang == 'urdu':
                instructions = (
                    "Translate the following text to Urdu using proper Arabic script (Nastaleeq). "
                    "Keep technical terms like ROS 2, NVIDIA, Isaac, VLA in English. "
                    "Output ONLY the Urdu translation, nothing else."
                )
            elif target_lang == 'arabic':
                instructions = (
                    "Translate the following text to Arabic. "
                    "Keep technical terms in English. "
                    "Output ONLY the Arabic translation, nothing else."
                )
            else:
                instructions = (
                    f"Translate the following text to {request.target_language}. "
                    "Keep technical terms in English. Output ONLY the translation, nothing else."
                )
            prompt = f"{instructions}\n\nText:\n{plain_text}"
            translated = self.ai_client.chat(message=prompt, context="")
            logger.info("Translation completed via Cohere")
        except Exception as ai_err:
            err_str = str(ai_err)
            is_rate_limit = "429" in err_str or "TooManyRequests" in err_str or "rate" in err_str.lower()
            if is_rate_limit:
                logger.warning("Cohere rate-limited — falling back to Google Translate")
            else:
                logger.warning(f"Cohere error — falling back to Google Translate: {err_str[:200]}")

        # --- Fallback: Google Translate (free, no API key) ---
        if not translated:
            try:
                translated = self._google_translate(plain_text, target_lang)
                logger.info("Translation completed via Google Translate (fallback)")
            except Exception as gt_err:
                logger.error(f"Google Translate fallback also failed: {gt_err}")
                from fastapi import HTTPException as _HTTPException
                raise _HTTPException(
                    status_code=503,
                    detail="Translation is temporarily unavailable. Please try again in a moment."
                )

        return TranslationResponse(
            translated_text=translated,
            source_language="english",
            target_language=request.target_language,
            character_count=len(translated)
        )