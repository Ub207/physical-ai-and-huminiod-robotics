from config.settings import settings
from .cohere_client import CohereClient
from .gemini_client import GeminiClient
import logging

logger = logging.getLogger(__name__)

class AIClientFactory:
    @staticmethod
    def create_client():
        """
        Create an AI client based on the configured provider.
        Currently supports 'gemini' or 'cohere' as the provider.
        """
        provider = settings.ai_provider.lower() if hasattr(settings, 'ai_provider') else 'cohere'

        if provider == 'gemini':
            logger.info("Initializing Gemini client")
            return GeminiClient()
        elif provider == 'cohere':
            logger.info("Initializing Cohere client")
            return CohereClient()
        else:
            logger.warning(f"Unknown AI provider: {provider}, defaulting to Cohere")
            return CohereClient()