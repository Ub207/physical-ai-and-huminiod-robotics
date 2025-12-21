import React from 'react';
import RagChatWidget from '@site/src/components/rag-chatbot/RagChatWidget';

// This component is rendered once for the entire app
// It wraps the entire application
export default function Root({ children }) {
  return (
    <>
      {children}
      {/* RAG Chatbot Widget - available on all pages */}
      <RagChatWidget apiUrl="http://localhost:8000" bookId="physical-ai-textbook" />
    </>
  );
}