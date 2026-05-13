from groq import Groq
from typing import List
from config.settings import settings
import logging

logger = logging.getLogger(__name__)

class GroqClient:
    MODEL = "llama-3.3-70b-versatile"

    def __init__(self):
        self.client = Groq(api_key=settings.groq_api_key)

    def chat(self, message: str, context: str = "", selected_text: str = "") -> str:
        system_prompt = (
            "You are an expert AI tutor for a Physical AI and Humanoid Robotics textbook. "
            "Structure your answer: start with a direct explanation, then key points, then examples if relevant. "
            "Answer naturally — do NOT say 'the context says' or 'according to the context'. "
            "Be thorough and educational. Use the provided textbook context to give accurate answers."
        )

        user_content = ""
        if context.strip():
            user_content += f"Textbook context:\n{context}\n\n"
        if selected_text:
            user_content += f"User selected text: {selected_text}\n\n"
        user_content += f"Question: {message}"

        try:
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
        except Exception as e:
            logger.error(f"Groq API error: {str(e)}")
            raise e
