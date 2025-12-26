import React, { useEffect } from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

function createChatWidgetScript(apiUrl, bookId) {
  // Properly escape the apiUrl and bookId to prevent syntax errors
  const escapedApiUrl = apiUrl.replace(/[\\"]/g, '\\$&');
  const escapedBookId = bookId.replace(/[\\"]/g, '\\$&');

  return `
(function () {
  if (window.__RAG_WIDGET_LOADED__) return;
  window.__RAG_WIDGET_LOADED__ = true;

  class RAGChatWidget {
    constructor(options = {}) {
      this.apiUrl = options.apiUrl || "${escapedApiUrl}";
      this.bookId = options.bookId || "${escapedBookId}";
      this.containerId = options.containerId || 'rag-chat-container';
      this.userSelectedText = null;

      this.initializeWidget();
    }

    initializeWidget() {
      let container = document.getElementById(this.containerId);
      if (!container) {
        container = document.createElement('div');
        container.id = this.containerId;
        document.body.appendChild(container);
      }

      this.addStyles();

      container.innerHTML =
        '<div id="rag-chat-widget" class="rag-chat-hidden">' +
        '<div class="rag-chat-header">' +
        '<h3>Physical AI Assistant</h3>' +
        '<button id="rag-close-btn" class="rag-close-btn">&times;</button>' +
        '</div>' +

        '<div id="rag-chat-messages" class="rag-chat-messages"></div>' +

        '<div class="rag-input-area">' +
        '<div class="rag-text-selection-indicator" id="rag-text-selection-indicator" style="display:none;">' +
        'Using selected text as context' +
        '</div>' +

        '<div class="rag-input-container">' +
        '<input type="text" id="rag-user-input" class="rag-user-input" placeholder="Ask a question about Physical AI...">' +
        '<button id="rag-send-btn" class="rag-send-btn">Send</button>' +
        '</div>' +
        '</div>' +
        '</div>' +

        '<button id="rag-open-btn" class="rag-open-btn">💬 AI</button>';

      this.setupEventListeners();
      this.setupTextSelection();
    }

    addStyles() {
      if (document.getElementById('rag-chat-styles')) return;

      const styles = document.createElement('style');
      styles.id = 'rag-chat-styles';
      styles.textContent =
        '#rag-chat-widget { ' +
        'position: fixed; ' +
        'bottom: 20px; ' +
        'right: 20px; ' +
        'width: 350px; ' +
        'height: 500px; ' +
        'background: white; ' +
        'border: 1px solid #ddd; ' +
        'border-radius: 10px; ' +
        'box-shadow: 0 4px 12px rgba(0,0,0,0.15); ' +
        'display: flex; ' +
        'flex-direction: column; ' +
        'z-index: 10000; ' +
        'font-family: -apple-system, BlinkMacSystemFont, \\'Segoe UI\\', Roboto, sans-serif; ' +
        '} ' +

        '.rag-chat-hidden { display:none!important; } ' +

        '.rag-chat-header { ' +
        'background:#4f46e5; ' +
        'color:white; ' +
        'padding:12px; ' +
        'border-radius:10px 10px 0 0; ' +
        'display:flex; ' +
        'justify-content:space-between; ' +
        'align-items:center; ' +
        '} ' +

        '.rag-close-btn { ' +
        'background:none; ' +
        'border:none; ' +
        'font-size:22px; ' +
        'color:white; ' +
        'cursor:pointer; ' +
        '} ' +

        '.rag-chat-messages { ' +
        'flex:1; ' +
        'padding:12px; ' +
        'overflow-y:auto; ' +
        'display:flex; ' +
        'flex-direction:column; ' +
        'gap:8px; ' +
        'background:#f9fafb; ' +
        '} ' +

        '.rag-message { ' +
        'max-width:80%; ' +
        'padding:10px 14px; ' +
        'border-radius:16px; ' +
        'font-size:14px; ' +
        'line-height:1.4; ' +
        '} ' +

        '.rag-user-message { ' +
        'align-self:flex-end; ' +
        'background:#4f46e5; ' +
        'color:white; ' +
        '} ' +

        '.rag-bot-message { ' +
        'align-self:flex-start; ' +
        'background:#e5e7eb; ' +
        'color:#374151; ' +
        '} ' +

        '.rag-input-area { padding:12px; border-top:1px solid #eee; } ' +

        '.rag-input-container { ' +
        'display:flex; ' +
        'gap:8px; ' +
        '} ' +

        '.rag-user-input { ' +
        'flex:1; ' +
        'padding:10px 14px; ' +
        'border-radius:20px; ' +
        'border:1px solid #ddd; ' +
        '} ' +

        '.rag-open-btn { ' +
        'position:fixed; ' +
        'bottom:20px; ' +
        'right:20px; ' +
        'width:60px; ' +
        'height:60px; ' +
        'border-radius:50%; ' +
        'font-size:18px; ' +
        'background:#4f46e5; ' +
        'color:white; ' +
        'border:none; ' +
        'cursor:pointer; ' +
        'box-shadow:0 4px 12px rgba(0,0,0,0.2); ' +
        'z-index:9999; ' +
        '} ' +

        '.rag-typing-indicator { ' +
        'background:#e5e7eb; ' +
        'padding:8px 12px; ' +
        'border-radius:18px; ' +
        'font-size:13px; ' +
        '}';

      document.head.appendChild(styles);
    }

    setupEventListeners() {
      const openBtn = document.getElementById('rag-open-btn');
      const closeBtn = document.getElementById('rag-close-btn');
      const widget = document.getElementById('rag-chat-widget');
      const sendBtn = document.getElementById('rag-send-btn');
      const input = document.getElementById('rag-user-input');

      openBtn.onclick = () => {
        widget.classList.remove('rag-chat-hidden');
        openBtn.style.display = 'none';
      };

      closeBtn.onclick = () => {
        widget.classList.add('rag-chat-hidden');
        openBtn.style.display = 'flex';
      };

      const send = () => {
        const message = input.value.trim();
        if (!message) return;

        this.addMessage(message, 'user');
        input.value = '';

        const typingId = this.showTypingIndicator();

        const body = {
          book_id: this.bookId,
          query: message,
          user_selected_text: this.userSelectedText || null
        };

        fetch(this.apiUrl + '/query', {
          method:'POST',
          headers:{'Content-Type':'application/json'},
          body:JSON.stringify(body)
        })
        .then(res => {
          if (!res.ok) {
            throw new Error('API error: ' + res.status);
          }
          return res.json();
        })
        .then(data => {
          this.removeTypingIndicator(typingId);
          this.addMessage(data.answer || 'No response', 'bot', data.sources);
        })
        .catch(error => {
          this.removeTypingIndicator(typingId);
          console.error('Chat API Error:', error);
          this.addMessage("I'm having trouble connecting to the AI service. Please make sure the backend server is running on port 8001.", 'bot');
        });
      };

      sendBtn.onclick = send;
      input.addEventListener('keypress', e => e.key === 'Enter' && send());
    }

    setupTextSelection() {
      document.addEventListener('mouseup', () => {
        const sel = window.getSelection().toString().trim();
        const indicator = document.getElementById('rag-text-selection-indicator');

        if (sel) {
          this.userSelectedText = sel;
          indicator.style.display = 'block';
        } else {
          this.userSelectedText = null;
          indicator.style.display = 'none';
        }
      });
    }

    addMessage(text, sender) {
      const container = document.getElementById('rag-chat-messages');
      const div = document.createElement('div');
      div.className = 'rag-message rag-' + sender + '-message';
      div.textContent = text;
      container.appendChild(div);
      container.scrollTop = container.scrollHeight;
    }

    showTypingIndicator() {
      const container = document.getElementById('rag-chat-messages');
      const id = 'typing-' + Date.now();
      const div = document.createElement('div');
      div.id = id;
      div.className = 'rag-typing-indicator';
      div.innerText = 'Thinking...';
      container.appendChild(div);
      container.scrollTop = container.scrollHeight;
      return id;
    }

    removeTypingIndicator(id) {
      const el = document.getElementById(id);
      if (el) el.remove();
    }
  }

  // Initialize the widget regardless of API availability
  window.RAGWidget = new RAGChatWidget({
    apiUrl: "${escapedApiUrl}",
    bookId: "${escapedBookId}"
  });

})();
  `;
}

export default function RagChatWidget({ apiUrl = 'https://ubaid-ai-rag-chatbot.hf.space', bookId = 'physical_ai_textbook' }) {
  const { siteConfig } = useDocusaurusContext();

  useEffect(() => {
    const script = document.createElement("script");
    script.innerHTML = createChatWidgetScript(apiUrl, bookId);
    document.body.appendChild(script);

    return () => script.remove();
  }, [apiUrl, bookId]);

  return null;
}
