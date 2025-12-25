import requests
import json

# Test the RAG chatbot with a simple question
def test_simple_question():
    url = "http://localhost:8000/query"

    # Test with a simple question like "what is AI?"
    payload = {
        "book_id": "physical_ai_textbook",  # This is the default book ID
        "query": "what is AI?",
        "user_selected_text": None
    }

    headers = {
        "Content-Type": "application/json"
    }

    try:
        response = requests.post(url, data=json.dumps(payload), headers=headers)
        print("Test 1 - 'what is AI?'")
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.json()}")
        print()
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure the backend server is running on port 8000")

    # Test with another simple question
    payload2 = {
        "book_id": "physical_ai_textbook",
        "query": "hello",
        "user_selected_text": None
    }

    try:
        response2 = requests.post(url, data=json.dumps(payload2), headers=headers)
        print("Test 2 - 'hello'")
        print(f"Status Code: {response2.status_code}")
        print(f"Response: {response2.json()}")
        print()
    except Exception as e:
        print(f"Error: {e}")

    # Test with a more complex question that should find relevant context
    payload3 = {
        "book_id": "physical_ai_textbook",
        "query": "What is Physical AI?",
        "user_selected_text": None
    }

    try:
        response3 = requests.post(url, data=json.dumps(payload3), headers=headers)
        print("Test 3 - 'What is Physical AI?'")
        print(f"Status Code: {response3.status_code}")
        print(f"Response: {response3.json()}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_simple_question()