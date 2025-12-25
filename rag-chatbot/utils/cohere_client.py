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
        # Try the configured model first, with fallbacks (excluding deprecated "command")
        models_to_try = [
            self.generation_model,
            "command-r-plus-08-2024",
            "command-r-plus",
            "command-r"
        ]

        last_error = None
        for model in models_to_try:
            try:
                response = self.client.chat(
                    model=model,
                    message=prompt,
                    max_tokens=max_tokens,
                    temperature=0.3
                )
                return response.text if response.text else ""
            except Exception as e:
                last_error = e
                continue  # Try the next model

        # If all models fail, raise the last error
        raise last_error

    def chat(self, message: str, context: str = "", selected_text: str = "") -> str:
        """
        Generate a response for the RAG chatbot with context.
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