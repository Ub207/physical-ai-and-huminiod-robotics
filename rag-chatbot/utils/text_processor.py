import re
from typing import List, Tuple
from config.settings import settings

class TextProcessor:
    @staticmethod
    def chunk_text(text: str, chunk_size: int = settings.chunk_size, chunk_overlap: int = settings.chunk_overlap) -> List[str]:
        """
        Split text into overlapping chunks using a recursive approach.
        """
        chunks = []
        start = 0

        while start < len(text):
            # Determine the end position
            end = start + chunk_size

            # If we're at the end, just take the remaining text
            if end >= len(text):
                chunk = text[start:]
                if chunk.strip():
                    chunks.append(chunk)
                break

            # Try to break at sentence boundary
            chunk = text[start:end]

            # Find the last sentence boundary within the chunk
            sentence_end = max(
                chunk.rfind('. '),
                chunk.rfind('? '),
                chunk.rfind('! '),
                chunk.rfind('\n'),
                chunk.rfind('.\n'),
                chunk.rfind('?\n'),
                chunk.rfind('!\n')
            )

            # If we found a sentence boundary and it's not at the beginning
            if sentence_end > 0 and sentence_end < len(chunk):
                # Adjust to include the punctuation
                sentence_end += 1
                chunk = chunk[:sentence_end]
                end = start + sentence_end

            # If no good break point found, just take the chunk_size characters
            if chunk.strip():
                chunks.append(chunk)

            # Move start position with overlap
            start = end - chunk_overlap

            # Ensure we make progress
            if start >= end:
                start = end

        return chunks

    @staticmethod
    def clean_text(text: str) -> str:
        """
        Clean and normalize text.
        """
        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text)
        # Remove special characters but keep basic punctuation
        text = re.sub(r'[^\w\s\.\,\!\?\;\:\-\n]', ' ', text)
        # Normalize whitespace
        text = ' '.join(text.split())
        return text

    @staticmethod
    def extract_pages_from_pdf(pdf_path: str) -> List[Tuple[int, str]]:
        """
        Extract text from PDF with page numbers.
        """
        import pdfplumber

        pages = []
        with pdfplumber.open(pdf_path) as pdf:
            for page_num, page in enumerate(pdf.pdf.pages, 1):
                text = page.extract_text()
                if text and text.strip():
                    # Clean the extracted text
                    cleaned_text = TextProcessor.clean_text(text)
                    pages.append((page_num, cleaned_text))
        return pages

    @staticmethod
    def extract_chapters_from_epub(epub_path: str) -> List[Tuple[int, str]]:
        """
        Extract text from EPUB with chapter numbers.
        """
        import ebooklib
        from ebooklib import epub
        from bs4 import BeautifulSoup

        chapters = []
        book = epub.read_epub(epub_path)
        chapter_num = 1

        for item in book.get_items():
            if item.get_type() == ebooklib.ITEM_DOCUMENT:
                soup = BeautifulSoup(item.get_content(), 'html.parser')
                text = soup.get_text()

                if text and text.strip():
                    # Clean the extracted text
                    cleaned_text = TextProcessor.clean_text(text)
                    chapters.append((chapter_num, cleaned_text))
                    chapter_num += 1
        return chapters

    @staticmethod
    def extract_from_txt(txt_path: str) -> List[Tuple[int, str]]:
        """
        Extract text from TXT file.
        """
        with open(txt_path, 'r', encoding='utf-8') as file:
            content = file.read()

        # For plain text files, we'll split into sections of ~1000 characters each
        sections = []
        section_size = 1000
        section_num = 1

        for i in range(0, len(content), section_size):
            section_text = content[i:i + section_size]
            if section_text.strip():
                # Clean the extracted text
                cleaned_text = TextProcessor.clean_text(section_text)
                sections.append((section_num, cleaned_text))
                section_num += 1

        return sections