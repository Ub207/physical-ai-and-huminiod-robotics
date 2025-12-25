import google.generativeai as genai
from typing import List
from config.settings import settings
import logging

logger = logging.getLogger(__name__)

class GeminiClient:
    def __init__(self):
        genai.configure(api_key=settings.gemini_api_key)
        self.model = genai.GenerativeModel(settings.gemini_generation_model)
        self.embed_model = genai.embed_content(
            model=settings.gemini_embed_model,
            content=["test"],  # Test call to verify model
            task_type="retrieval_document"
        )
        self.embed_model_name = settings.gemini_embed_model

    def embed_texts(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Gemini.
        input_type: "search_document" for documents, "search_query" for queries (Cohere style)
        """
        # Map Cohere-style input_type to Gemini task_type
        task_type = "retrieval_document" if input_type == "search_document" else "retrieval_query"

        try:
            response = genai.embed_content(
                model=self.embed_model_name,
                content=texts,
                task_type=task_type
            )
            # Return the embeddings from the response
            return response['embedding']
        except Exception as e:
            logger.error(f"Error generating embeddings with Gemini: {str(e)}")
            raise e

    def generate_response(self, prompt: str, max_tokens: int = 500) -> str:
        """
        Generate a response using Gemini's model.
        """
        try:
            response = self.model.generate_content(
                prompt,
                generation_config={
                    "max_output_tokens": max_tokens,
                    "temperature": 0.3
                }
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
        if context.strip():
            full_prompt = f"Context: {context}\n\n"
        else:
            # Provide a default context when no relevant context is found
            full_prompt = "You are an AI assistant for a Physical AI and Humanoid Robotics textbook. "
            full_prompt += "The following is a general question not directly related to specific textbook content.\n\n"

        if selected_text:
            full_prompt += f"User-selected text: {selected_text}\n\n"
        full_prompt += f"Question: {message}\n\n"
        full_prompt += "Please answer the question based on the provided context. If no specific textbook context is available, acknowledge the question and encourage the user to ask more specific questions about the Physical AI and Humanoid Robotics textbook content. Be concise and accurate."

        return self.generate_response(full_prompt)