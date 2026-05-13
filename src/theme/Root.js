import React from 'react';
import RagChatWidget from '@site/src/components/chat/RagChatWidget';
import { LanguageProvider } from '@site/src/contexts/LanguageProvider';

// Error Boundary for Language Context failures
class LanguageErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error) {
    return { hasError: true, error };
  }

  componentDidCatch(error, errorInfo) {
    console.error('Language Context Error:', error, errorInfo);
  }

  render() {
    if (this.state.hasError) {
      return (
        <div style={{ padding: '20px', textAlign: 'center' }}>
          <h2>Language System Error</h2>
          <p>The language switching feature encountered an error. Please refresh the page.</p>
          <button onClick={() => window.location.reload()}>Refresh Page</button>
        </div>
      );
    }

    return this.props.children;
  }
}

// This component is rendered once for the entire app
// It wraps the entire application
export default function Root({ children }) {
  // Use localhost for local dev, HF Space for production
  const isLocal = typeof window !== 'undefined' && (
    window.location.hostname === 'localhost' ||
    window.location.hostname === '127.0.0.1'
  );

  const apiUrl = isLocal
    ? 'http://localhost:8000'
    : 'https://ubaid-ai-rag-chatbot.hf.space';

  console.log('RAG Chatbot API URL:', apiUrl);

  return (
    <LanguageErrorBoundary>
      <LanguageProvider apiUrl={apiUrl}>
        {children}
        {/* RAG Chatbot Widget - available on all pages */}
        <RagChatWidget apiUrl={apiUrl} />
      </LanguageProvider>
    </LanguageErrorBoundary>
  );
}
