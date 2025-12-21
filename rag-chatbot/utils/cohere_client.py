import cohere
from typing import List, Dict, Any
from config.settings import settings

class CohereClient:
    def __init__(self):
        self.client = cohere.Client(settings.cohere_api_key)
        self.embed_model = settings.cohere_embed_model
        self.generation_model = settings.cohere_generation_model

    def embed_texts(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere.
        input_type: "search_document" for documents, "search_query" for queries
        """
        response = self.client.embed(
            texts=texts,
            model=self.embed_model,
            input_type=input_type
        )
        # The response.embeddings is now a list of lists directly
        return response.embeddings

    def generate_response(self, prompt: str, max_tokens: int = 500) -> str:
        """
        Generate a response using Cohere's chat model.
        """
        response = self.client.chat(
            model=self.generation_model,
            message=prompt,
            max_tokens=max_tokens,
            temperature=0.3
        )
        return response.text if response.text else ""

    def chat(self, message: str, context: str = "", selected_text: str = "") -> str:
        """
        Generate a response for the RAG chatbot with context.
        """
        # Build the full prompt with context
        full_prompt = f"Context: {context}\n\n"
        if selected_text:
            full_prompt += f"User-selected text: {selected_text}\n\n"
        full_prompt += f"Question: {message}\n\n"
        full_prompt += "Please answer the question based on the provided context. Be concise and accurate."

        return self.generate_response(full_prompt)