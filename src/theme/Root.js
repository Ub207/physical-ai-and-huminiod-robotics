import React from 'react';
import RagChatWidget from '@site/src/components/chat/RagChatWidget';

// This component is rendered once for the entire app
// It wraps the entire application
export default function Root({ children }) {
  // Determine API URL based on environment
  const apiUrl = typeof window !== 'undefined'
    ? window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1'
      ? 'http://localhost:8001'  // Local development backend
      : 'https://your-rag-chatbot-backend.onrender.com'  // Update this with your deployed backend URL from Render
    : 'http://localhost:8001';  // Default for SSR

  return (
    <>
      {children}
      {/* RAG Chatbot Widget - available on all pages, will initialize only if backend is available */}
      <RagChatWidget apiUrl={apiUrl} />
    </>
  );
}