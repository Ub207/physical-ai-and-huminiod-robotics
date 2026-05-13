from google import genai
from google.genai import types
from typing import List
from config.settings import settings
import logging

logger = logging.getLogger(__name__)

class GeminiClient:
    def __init__(self):
        try:
            self.client = genai.Client(api_key=settings.gemini_api_key)
            self.generation_model_name = settings.gemini_generation_model
            self.embed_model_name = settings.gemini_embed_model
            logger.info(f"Initialized Gemini client with model: {settings.gemini_generation_model}")
        except Exception as e:
            logger.error(f"Error initializing Gemini client: {str(e)}")
            raise e

    def embed_texts(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Gemini.
        input_type: "search_document" for documents, "search_query" for queries (Cohere style)
        """
        # Map Cohere-style input_type to Gemini task_type
        task_type = "RETRIEVAL_DOCUMENT" if input_type == "search_document" else "RETRIEVAL_QUERY"

        try:
            embeddings = []
            for text in texts:
                response = self.client.models.embed_content(
                    model=self.embed_model_name,
                    content=text,
                    config=types.EmbedContentConfig(task_type=task_type)
                )
                embeddings.append(response.embeddings[0].values)
            return embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings with Gemini: {str(e)}")
            raise e

    def generate_response(self, prompt: str, max_tokens: int = 500) -> str:
        """
        Generate a response using Gemini's model.
        """
        try:
            response = self.client.models.generate_content(
                model=self.generation_model_name,
                contents=prompt,
                config=types.GenerateContentConfig(
                    max_output_tokens=max_tokens,
                    temperature=0.3
                )
            )
            return response.text if response.text else ""
        except Exception as e:
            logger.error(f"Error generating response with Gemini: {str(e)}")
            raise e

    def chat(self, message: str, context: str = "", selected_text: str = "") -> str:
        """
        Generate a response for the RAG chatbot with context using Gemini.
        """
        # Build the full prompt with context
        system = (
            "You are an expert AI tutor for a Physical AI and Humanoid Robotics textbook. "
            "Answer the student's question clearly and in detail using the provided textbook context. "
            "Structure your answer with: a direct definition or explanation first, then key points, then examples if relevant. "
            "Do NOT say 'the context says' or 'according to the context'. Just answer naturally. "
            "If the context does not cover the question, answer from your general knowledge about robotics/AI."
        )
        if context.strip():
            full_prompt = f"{system}\n\nTextbook Context:\n{context}\n\n"
        else:
            full_prompt = f"{system}\n\n"
        if selected_text:
            full_prompt += f"Selected text the student is asking about:\n{selected_text}\n\n"
        full_prompt += f"Student's question: {message}\n\nAnswer:"

        return self.generate_response(full_prompt)