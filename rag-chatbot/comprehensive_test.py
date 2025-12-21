import requests
import json
import time

def test_rag_comprehensive():
    base_url = "http://localhost:8000"

    # Wait a moment for server to be ready
    time.sleep(2)

    # Test health endpoint
    try:
        response = requests.get(f"{base_url}/health")
        print(f"SUCCESS: Health check: {response.status_code} - {response.json()}")
    except Exception as e:
        print(f"FAILED: Health check failed: {e}")
        return

    # Test various queries to see if any return results
    test_queries = [
        {
            "name": "General topic",
            "data": {
                "book_id": "physical_ai_textbook",
                "query": "What is this book about?",
                "user_selected_text": None
            }
        },
        {
            "name": "Specific topic",
            "data": {
                "book_id": "physical_ai_textbook",
                "query": "Tell me about Physical AI and Humanoid Robotics",
                "user_selected_text": None
            }
        },
        {
            "name": "Module structure",
            "data": {
                "book_id": "physical_ai_textbook",
                "query": "What are the four core modules of this textbook?",
                "user_selected_text": None
            }
        },
        {
            "name": "Course structure",
            "data": {
                "book_id": "physical_ai_textbook",
                "query": "Describe the course structure",
                "user_selected_text": None
            }
        }
    ]

    for test in test_queries:
        try:
            print(f"\n--- Testing: {test['name']} ---")
            response = requests.post(f"{base_url}/query",
                                   headers={"Content-Type": "application/json"},
                                   data=json.dumps(test['data']))

            if response.status_code == 200:
                result = response.json()
                print(f"SUCCESS: Query successful!")
                print(f"  Answer length: {len(result['answer'])} chars")
                print(f"  Sources found: {len(result['sources'])}")
                print(f"  Confidence: {result['confidence']:.2f}")

                if len(result['sources']) > 0:
                    print(f"  SUCCESS: FOUND MATCHES - First source snippet: {result['sources'][0]['content_snippet'][:100]}...")
                    print(f"  SUCCESS: The RAG system is working!")
                    return True
                else:
                    print(f"  FAILED: No sources found for this query")
            else:
                print(f"FAILED: Query failed: {response.status_code} - {response.text}")
        except Exception as e:
            print(f"FAILED: Query test failed: {e}")

    print(f"\n--- Summary ---")
    print("If no queries returned sources, there may still be an issue with the retrieval component.")
    return False

if __name__ == "__main__":
    success = test_rag_comprehensive()
    if success:
        print("\n🎉 SUCCESS: RAG chatbot is working correctly!")
    else:
        print("\n⚠️  The RAG system may still have issues retrieving relevant content.")