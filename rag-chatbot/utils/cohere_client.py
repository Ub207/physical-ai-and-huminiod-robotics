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