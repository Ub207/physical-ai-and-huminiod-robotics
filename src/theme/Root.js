import React from 'react';
import RagChatWidget from '@site/src/components/chat/RagChatWidget';

// This component is rendered once for the entire app
// It wraps the entire application
export default function Root({ children }) {
  // Always use Hugging Face backend (deployed)
  const apiUrl = 'https://ubaid-ai-rag-chatbot.hf.space';

  return (
    <>
      {children}
      {/* RAG Chatbot Widget - available on all pages, will initialize only if backend is available */}
      <RagChatWidget apiUrl={apiUrl} />
    </>
  );
}