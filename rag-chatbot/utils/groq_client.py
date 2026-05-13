from groq import Groq
from typing import List
from config.settings import settings
import logging

logger = logging.getLogger(__name__)

class GroqClient:
    MODEL = "llama-3.3-70b-versatile"

    def __init__(self):
        self.client = Groq(api_key=settings.groq_api_key)
        logger.info(f"Initialized Groq client with model: {self.MODEL}")

    def chat(self, message: str, context: str = "", selected_text: str = "") -> str:
        system_prompt = (
            "You are an expert AI tutor for a Physical AI and Humanoid Robotics textbook. "
            "Answer the student's question clearly and in detail using the provided textbook context. "
            "Structure your answer: start with a direct explanation, then key points, then examples if relevant. "
            "Answer naturally — do NOT say 'the context says' or 'according to the context'. "
            "If the context does not fully cover the question, use your general knowledge about robotics and AI."
        )

        user_content = ""
        if context.strip():
            user_content += f"Textbook Context:\n{context}\n\n"
        if selected_text:
            user_content += f"Selected text the student highlighted:\n{selected_text}\n\n"
        user_content += f"Student's question: {message}"

        response = self.client.chat.completions.create(
            model=self.MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_content},
            ],
            temperature=0.4,
            max_tokens=1024,
        )
        return response.choices[0].message.content or ""
