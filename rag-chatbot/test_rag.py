import requests
import json

# Test the RAG chatbot
def test_rag_chatbot():
    base_url = "http://localhost:8000"

    # Test health endpoint
    try:
        response = requests.get(f"{base_url}/health")
        print(f"Health check: {response.status_code} - {response.json()}")
    except Exception as e:
        print(f"Health check failed: {e}")
        return

    # Test query
    query_data = {
        "book_id": "physical_ai_textbook",
        "query": "What is this book about?",
        "user_selected_text": None
    }

    try:
        response = requests.post(f"{base_url}/query",
                                headers={"Content-Type": "application/json"},
                                data=json.dumps(query_data))

        if response.status_code == 200:
            result = response.json()
            print(f"Query successful!")
            print(f"Answer: {result['answer'][:200]}...")  # Print first 200 chars
            print(f"Sources: {len(result['sources'])} relevant chunks found")
            print(f"Confidence: {result['confidence']:.2f}")
        else:
            print(f"Query failed: {response.status_code} - {response.text}")
    except Exception as e:
        print(f"Query test failed: {e}")

if __name__ == "__main__":
    test_rag_chatbot()