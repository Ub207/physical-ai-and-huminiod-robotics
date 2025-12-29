import React from 'react';
import RagChatWidget from '@site/src/components/chat/RagChatWidget';
import AuthWidget from '@site/src/components/translation/AuthWidget';

// This component is rendered once for the entire app
// It wraps the entire application
export default function Root({ children }) {
  // Use localhost for local dev, HF Space for production
  const apiUrl = typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://ubaid-ai-rag-chatbot.hf.space';

  return (
    <>
      {/* Auth Widget - top bar for login/logout and bonus points */}
      <AuthWidget />
      {children}
      {/* RAG Chatbot Widget - available on all pages, will initialize only if backend is available */}
      <RagChatWidget apiUrl={apiUrl} />
    </>
  );
}
