#!/usr/bin/env python3
"""
Script to ingest your book content into Qdrant vector database for RAG chatbot.
This script reads your book text file and processes it into vector embeddings.
"""

import os
import re
from pathlib import Path
from dotenv import load_dotenv

# Qdrant and Cohere imports
from qdrant_client import QdrantClient, models
import cohere

# Load environment variables from .env file
load_dotenv()

# Initialize Cohere and Qdrant clients globally
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
if not COHERE_API_KEY:
    raise ValueError("COHERE_API_KEY environment variable not set.")

cohere_client = cohere.Client(COHERE_API_KEY)

# Initialize Qdrant client
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not QDRANT_URL:
    # Fallback to local Qdrant if cloud URL not provided
    qdrant_client = QdrantClient(host=os.getenv("QDRANT_HOST", "localhost"), port=int(os.getenv("QDRANT_PORT", "6333")))
    print("Connecting to local Qdrant instance.")
else:
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY, prefer_grpc=False)
    print(f"Connecting to Qdrant Cloud at {QDRANT_URL}.")

# Qdrant collection details
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")  # Changed to match your collection name
VECTOR_SIZE = 1024 # Cohere embed-english-v3.0 is 1024 dimensions

def clean_markdown_content(content):
    """
    Remove Docusaurus-specific frontmatter and clean up markdown content
    """
    # Remove frontmatter (content between --- delimiters)
    content = re.sub(r'---\n.*?\n---\n', '', content, flags=re.DOTALL)

    # Remove relative links and convert to plain text
    content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)  # Replace [text](link) with text

    # Remove image references
    content = re.sub(r'!\[([^\]]*)\]\([^)]+\)', '', content)

    # Remove other markdown formatting while preserving content
    content = re.sub(r'\*\*(.*?)\*\*', r'\1', content)  # Bold
    content = re.sub(r'\*(.*?)\*', r'\1', content)      # Italic
    content = re.sub(r'~~(.*?)~~', r'\1', content)      # Strikethrough

    # Remove heading markers but keep the text
    content = re.sub(r'^#+\s*(.*)', r'\1', content, flags=re.MULTILINE)

    return content.strip()

def chunk_document(document_path: str, content: str, chunk_size: int = 500, chunk_overlap: int = 50):
    """
    Splits a document's content into smaller, overlapping chunks.
    This is a basic paragraph splitter; more advanced methods might use tokenizers.
    """
    chunks = []
    # Split by double newline to get paragraphs
    paragraphs = [p.strip() for p in content.split('\n\n') if p.strip()]

    current_chunk = []
    current_length = 0

    for paragraph in paragraphs:
        if current_length + len(paragraph) + 2 > chunk_size and current_chunk: # +2 for newline
            chunks.append({
                "document_path": document_path,
                "content": "\n\n".join(current_chunk)
            })
            # Add overlap: start new chunk with the last few paragraphs
            current_chunk = current_chunk[-max(1, len(current_chunk) * chunk_overlap // chunk_size):] # Simple overlap
            current_length = sum(len(p) + 2 for p in current_chunk)

        current_chunk.append(paragraph)
        current_length += len(paragraph) + 2

    if current_chunk:
        chunks.append({
            "document_path": document_path,
            "content": "\n\n".join(current_chunk)
        })
    return chunks

def generate_embeddings(texts: list[str], batch_size: int = 96) -> list[list[float]]:
    """
    Generates embeddings for a list of texts using the Cohere client.
    Processes in batches to avoid timeout issues.
    """
    print(f"Generating embeddings for {len(texts)} chunks in batches of {batch_size}...")
    all_embeddings = []

    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]
        print(f"Processing batch {i//batch_size + 1}/{(len(texts) - 1)//batch_size + 1}...")
        response = cohere_client.embed(texts=batch, model="embed-english-v3.0", input_type="search_document")
        all_embeddings.extend(response.embeddings)

    return all_embeddings

def upload_to_qdrant(chunks: list[dict], book_id: str = "physical_ai_textbook", batch_size: int = 50):
    """
    Uploads text chunks and their embeddings to Qdrant in batches.
    """
    # Ensure the collection exists
    try:
        # Check if collection exists first
        try:
            collection_info = qdrant_client.get_collection(collection_name=COLLECTION_NAME)
            print(f"Collection '{COLLECTION_NAME}' already exists with {collection_info.points_count} points.")
            # Delete existing collection to recreate
            qdrant_client.delete_collection(collection_name=COLLECTION_NAME)
            print(f"Deleted existing collection '{COLLECTION_NAME}'.")
        except Exception:
            print(f"Collection '{COLLECTION_NAME}' does not exist yet.")

        # Create new collection
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
        )
        print(f"Collection '{COLLECTION_NAME}' created successfully.")
    except Exception as e:
        print(f"Error managing collection: {e}")
        raise

    # Prepare points for upsertion
    chunk_contents = [chunk["content"] for chunk in chunks]
    embeddings = generate_embeddings(chunk_contents)

    # Upload in batches
    for batch_start in range(0, len(chunks), batch_size):
        batch_end = min(batch_start + batch_size, len(chunks))
        batch_points = []

        for i in range(batch_start, batch_end):
            chunk = chunks[i]
            batch_points.append(
                models.PointStruct(
                    id=i, # Simple incremental ID
                    vector=embeddings[i],
                    payload={
                        "book_id": book_id,  # Add the book_id to the payload
                        "document_path": chunk["document_path"],
                        "content": chunk["content"]
                    }
                )
            )

        print(f"Uploading batch {batch_start//batch_size + 1}/{(len(chunks) - 1)//batch_size + 1} ({len(batch_points)} points) to Qdrant collection '{COLLECTION_NAME}'...")
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            wait=True,
            points=batch_points,
        )

    print("Upload to Qdrant complete.")

def extract_text_from_file(file_path: str):
    """
    Extract content from a text file
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
        content = clean_markdown_content(content)
        return [{
            "path": file_path,
            "content": content
        }]

def main():
    print("Converting book content to RAG-compatible format and ingesting into Qdrant...")

    # Extract content from your book file
    book_file_path = "physical_ai_textbook.txt"
    if not os.path.exists(book_file_path):
        print(f"Error: Book file {book_file_path} not found!")
        return

    all_documents = extract_text_from_file(book_file_path)

    # Chunk documents for RAG
    all_chunks = []
    for doc in all_documents:
        chunks = chunk_document(doc["path"], doc["content"])
        all_chunks.extend(chunks)

    # Upload to Qdrant
    book_id = "physical_ai_textbook"
    if all_chunks:
        upload_to_qdrant(all_chunks, book_id=book_id)
        print(f"Book ingestion complete! Processed {len(all_documents)} documents, created {len(all_chunks)} chunks for book_id: {book_id}.")
    else:
        print("No content chunks generated. Skipping Qdrant upload.")

if __name__ == "__main__":
    main()