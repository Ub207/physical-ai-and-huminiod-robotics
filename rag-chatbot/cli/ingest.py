import asyncio
import argparse
import sys
from pathlib import Path

from api.rag_service import RAGService
from models.schemas import IngestionRequest
from config.settings import settings
from utils.helpers import generate_id, validate_file_type

async def main():
    parser = argparse.ArgumentParser(description='Ingest a book into the RAG system')
    parser.add_argument('--file', type=str, required=True, help='Path to the book file (PDF, EPUB, or TXT)')
    parser.add_argument('--book-id', type=str, help='Unique ID for the book (auto-generated if not provided)')
    parser.add_argument('--title', type=str, help='Title of the book')
    parser.add_argument('--author', type=str, help='Author of the book')
    parser.add_argument('--chunk-size', type=int, default=512, help='Size of text chunks (default: 512)')
    parser.add_argument('--chunk-overlap', type=int, default=50, help='Overlap between chunks (default: 50)')

    args = parser.parse_args()

    # Validate file
    file_path = Path(args.file)
    if not file_path.exists():
        print(f"Error: File {args.file} does not exist")
        sys.exit(1)

    if not validate_file_type(str(file_path)):
        print(f"Error: Unsupported file type. Supported formats: PDF, EPUB, TXT")
        sys.exit(1)

    # Generate book ID if not provided
    book_id = args.book_id or generate_id()

    # Extract title from filename if not provided
    title = args.title or file_path.stem

    # Process the book file
    print(f"Processing {file_path.name}...")
    try:
        # Create ingestion request
        request = IngestionRequest(
            book_id=book_id,
            title=title,
            author=args.author or "Unknown Author",
            file_path=str(file_path),
            chunk_size=args.chunk_size,
            chunk_overlap=args.chunk_overlap
        )

        # Initialize RAG service and ingest
        rag_service = RAGService()
        result = await rag_service.ingest_book(request)

        print(f"✅ Book ingestion completed successfully:")
        print(f"   • Book ID: {book_id}")
        print(f"   • Title: {title}")
        print(f"   • Total chunks: {result.get('total_chunks', 'Unknown')}")
        print(f"   • Status: {result.get('status', 'unknown')}")

    except Exception as e:
        print(f"❌ Error processing book: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())