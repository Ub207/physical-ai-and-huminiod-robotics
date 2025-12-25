import json
import os
from typing import Dict, Any
from api.main import app
from fastapi.testclient import TestClient

# Initialize the FastAPI app
client = TestClient(app)

def preprocess_request(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Preprocess the input payload for the RAG chatbot
    """
    return payload

def postprocess_response(response: Any) -> Dict[str, Any]:
    """
    Postprocess the response from the RAG chatbot
    """
    return response

def predict(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Main prediction function for Hugging Face Inference API
    """
    try:
        # Determine the endpoint based on the payload
        if 'query' in payload and 'book_id' in payload:
            # This is a query request
            response = client.post("/query", json=payload)
        elif 'book_id' in payload and 'content' in payload:
            # This is an ingestion request
            response = client.post("/ingest", json=payload)
        else:
            # Default to health check
            response = client.get("/health")

        if response.status_code == 200:
            return response.json()
        else:
            return {
                "error": f"Request failed with status {response.status_code}",
                "details": response.text
            }
    except Exception as e:
        return {
            "error": str(e)
        }

def inference(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Wrapper function for Hugging Face Inference API
    """
    processed_payload = preprocess_request(payload)
    result = predict(processed_payload)
    return postprocess_response(result)