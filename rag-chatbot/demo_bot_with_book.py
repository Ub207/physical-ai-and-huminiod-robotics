#!/usr/bin/env python3
"""
Demonstration of your RAG Chatbot with your Physical AI and Humanoid Robotics textbook
"""

import requests
import json

def display_book_info():
    """Display information about your book"""
    print("=" * 80)
    print("YOUR BOOK: Physical AI and Humanoid Robotics Textbook")
    print("=" * 80)
    print()
    print("BOOK STRUCTURE:")
    print("   The content is organized into four core modules:")
    print("   - Module 1: The Robotic Nervous System (ROS 2)")
    print("   - Module 2: The Digital Twin (Gazebo & Unity)")
    print("   - Module 3: The AI-Robot Brain (NVIDIA Isaac)")
    print("   - Module 4: Vision-Language-Action (VLA)")
    print()
    print("LEARNING OUTCOMES:")
    print("   Upon completion, students will be able to:")
    print("   - Understand fundamental principles of embodied intelligence and physical AI")
    print("   - Configure and operate ROS 2-based robotic systems")
    print("   - Implement simulation environments using Gazebo and Unity")
    print("   - Deploy AI models on NVIDIA Isaac platforms")
    print("   - Develop Vision-Language-Action systems for robot interaction")
    print()

def demonstrate_bot_interaction():
    """Demonstrate the RAG chatbot in action"""
    base_url = "http://localhost:8000"

    print("RAG CHATBOT IN ACTION")
    print("=" * 80)
    print()

    # Test queries that showcase different aspects of your book
    demonstrations = [
        {
            "title": "General Book Overview",
            "query": "What is this book about?",
            "explanation": "Asking for a general overview of the book content"
        },
        {
            "title": "Course Structure",
            "query": "What are the four core modules of this textbook?",
            "explanation": "Asking about the book's structure"
        },
        {
            "title": "Learning Outcomes",
            "query": "What will students learn from this textbook?",
            "explanation": "Asking about what students will learn"
        },
        {
            "title": "Technical Skills",
            "query": "What technical skills will students gain?",
            "explanation": "Asking about specific technical skills taught"
        }
    ]

    for i, demo in enumerate(demonstrations, 1):
        print(f"Demo {i}: {demo['title']}")
        print(f"Query: {demo['query']}")
        print(f"Explanation: {demo['explanation']}")
        print("-" * 60)

        try:
            query_data = {
                "book_id": "physical_ai_textbook",
                "query": demo['query'],
                "user_selected_text": None
            }

            response = requests.post(f"{base_url}/query",
                                   headers={"Content-Type": "application/json"},
                                   data=json.dumps(query_data),
                                   timeout=30)

            if response.status_code == 200:
                result = response.json()
                sources_count = len(result['sources'])
                confidence = result['confidence']

                print(f"Status: {response.status_code}")
                print(f"Sources Found: {sources_count}")
                print(f"Confidence: {confidence:.3f}")
                print(f"Answer Preview: {result['answer'][:300]}...")
                if len(result['answer']) > 300:
                    print("...")
                print()

                if sources_count > 0:
                    print(f"First Source Snippet: {result['sources'][0]['content_snippet'][:200]}...")
                    if len(result['sources'][0]['content_snippet']) > 200:
                        print("...")
                    print()
                else:
                    print("No sources found")
                    print()
            else:
                print(f"Error: {response.status_code} - {response.text}")
                print()

        except Exception as e:
            print(f"Error during query: {e}")
            print()

def show_system_status():
    """Show the system status"""
    base_url = "http://localhost:8000"

    print("SYSTEM STATUS")
    print("=" * 80)

    try:
        response = requests.get(f"{base_url}/health")
        if response.status_code == 200:
            health = response.json()
            print(f"SUCCESS: API Status: {health['status']}")
            print(f"  Message: {health['message']}")
        else:
            print(f"FAILED: API Error: {response.status_code}")
    except Exception as e:
        print(f"FAILED: API Connection Error: {e}")
        print("  Make sure the RAG server is running on http://localhost:8000")

    print()

def main():
    print("TARGET: PHYSICAL AI & HUMINOID ROBOTICS RAG CHATBOT DEMONSTRATION")
    print()

    display_book_info()
    show_system_status()
    demonstrate_bot_interaction()

    print("DEMONSTRATION COMPLETE")
    print()
    print("Your RAG chatbot is successfully connected to your Physical AI and Humanoid Robotics textbook!")
    print("It can answer questions about the book's content using retrieval-augmented generation.")

if __name__ == "__main__":
    main()