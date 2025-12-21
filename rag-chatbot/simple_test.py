import requests
import json

# Simple test to check if RAG is working
base_url = "http://localhost:8000"

# Test the query endpoint directly
query_data = {
    "book_id": "physical_ai_textbook",
    "query": "What is this book about?",
    "user_selected_text": None
}

try:
    print("Sending query to RAG system...")
    response = requests.post(f"{base_url}/query",
                           headers={"Content-Type": "application/json"},
                           data=json.dumps(query_data),
                           timeout=30)  # 30 second timeout

    print(f"Status Code: {response.status_code}")
    if response.status_code == 200:
        result = response.json()
        print(f"Answer: {result['answer'][:200]}...")
        print(f"Sources found: {len(result['sources'])}")
        print(f"Confidence: {result['confidence']}")

        if len(result['sources']) > 0:
            print("SUCCESS: RAG system is working and returning relevant sources!")
            print(f"First source content: {result['sources'][0]['content_snippet'][:150]}...")
        else:
            print("ISSUE: No sources found - retrieval may not be working properly")
    else:
        print(f"Error response: {response.text}")

except requests.exceptions.Timeout:
    print("Request timed out - this may indicate a processing issue")
except Exception as e:
    print(f"Error: {e}")