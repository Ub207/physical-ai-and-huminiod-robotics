/**
 * Embeddable RAG Chatbot Widget
 * This script can be embedded in any webpage to provide a chat interface
 * that connects to the RAG backend.
 */

class RAGChatWidget {
  constructor(options = {}) {
    this.apiUrl = options.apiUrl || 'http://localhost:8000';
    this.bookId = options.bookId;
    this.containerId = options.containerId || 'rag-chat-container';
    this.userSelectedText = null;

    this.initializeWidget();
  }

  initializeWidget() {
    // Create the chat widget container if it doesn't exist
    let container = document.getElementById(this.containerId);
    if (!container) {
      container = document.createElement('div');
      container.id = this.containerId;
      document.body.appendChild(container);
    }

    // Add CSS styles
    this.addStyles();

    // Create widget HTML
    container.innerHTML = `
      <div id="rag-chat-widget" class="rag-chat-hidden">
        <div class="rag-chat-header">
          <h3>Book Assistant</h3>
          <button id="rag-close-btn" class="rag-close-btn">&times;</button>
        </div>
        <div id="rag-chat-messages" class="rag-chat-messages"></div>
        <div class="rag-input-area">
          <div class="rag-text-selection-indicator" id="rag-text-selection-indicator" style="display: none;">
            Using selected text as context
          </div>
          <div class="rag-input-container">
            <input type="text" id="rag-user-input" class="rag-user-input" placeholder="Ask a question about the book...">
            <button id="rag-send-btn" class="rag-send-btn">Send</button>
          </div>
        </div>
      </div>
      <button id="rag-open-btn" class="rag-open-btn">💬</button>
    `;

    // Set up event listeners
    this.setupEventListeners();

    // Set up text selection monitoring
    this.setupTextSelection();
  }

  addStyles() {
    if (document.getElementById('rag-chat-styles')) return;

    const styles = document.createElement('style');
    styles.id = 'rag-chat-styles';
    styles.textContent = `
      #rag-chat-widget {
        position: fixed;
        bottom: 20px;
        right: 20px;
        width: 350px;
        height: 500px;
        background: white;
        border: 1px solid #ddd;
        border-radius: 10px;
        box-shadow: 0 4px 12px rgba(0,0,0,0.15);
        display: flex;
        flex-direction: column;
        z-index: 10000;
        font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
      }

      .rag-chat-hidden {
        display: none !important;
      }

      .rag-chat-header {
        background: #4f46e5;
        color: white;
        padding: 15px;
        border-top-left-radius: 10px;
        border-top-right-radius: 10px;
        display: flex;
        justify-content: space-between;
        align-items: center;
      }

      .rag-close-btn {
        background: none;
        border: none;
        color: white;
        font-size: 20px;
        cursor: pointer;
        padding: 0;
        width: 24px;
        height: 24px;
        display: flex;
        align-items: center;
        justify-content: center;
      }

      .rag-chat-messages {
        flex: 1;
        padding: 15px;
        overflow-y: auto;
        display: flex;
        flex-direction: column;
        gap: 10px;
        background: #f9fafb;
      }

      .rag-message {
        max-width: 80%;
        padding: 10px 15px;
        border-radius: 18px;
        font-size: 14px;
        line-height: 1.4;
      }

      .rag-user-message {
        align-self: flex-end;
        background: #4f46e5;
        color: white;
        border-bottom-right-radius: 5px;
      }

      .rag-bot-message {
        align-self: flex-start;
        background: #e5e7eb;
        color: #374151;
        border-bottom-left-radius: 5px;
      }

      .rag-input-area {
        padding: 15px;
        background: white;
        border-top: 1px solid #eee;
      }

      .rag-text-selection-indicator {
        background: #fef3c7;
        color: #92400e;
        padding: 5px 10px;
        border-radius: 4px;
        font-size: 12px;
        margin-bottom: 10px;
        text-align: center;
      }

      .rag-input-container {
        display: flex;
        gap: 10px;
      }

      .rag-user-input {
        flex: 1;
        padding: 10px 15px;
        border: 1px solid #ddd;
        border-radius: 20px;
        font-size: 14px;
        outline: none;
      }

      .rag-user-input:focus {
        border-color: #4f46e5;
      }

      .rag-send-btn {
        background: #4f46e5;
        color: white;
        border: none;
        border-radius: 20px;
        padding: 10px 15px;
        cursor: pointer;
        font-size: 14px;
      }

      .rag-send-btn:hover {
        background: #4338ca;
      }

      .rag-open-btn {
        position: fixed;
        bottom: 20px;
        right: 20px;
        width: 60px;
        height: 60px;
        border-radius: 50%;
        background: #4f46e5;
        color: white;
        border: none;
        font-size: 24px;
        cursor: pointer;
        box-shadow: 0 4px 12px rgba(0,0,0,0.15);
        z-index: 9999;
        display: flex;
        align-items: center;
        justify-content: center;
      }

      .rag-typing-indicator {
        align-self: flex-start;
        background: #e5e7eb;
        color: #374151;
        padding: 10px 15px;
        border-radius: 18px;
        font-size: 14px;
        border-bottom-left-radius: 5px;
      }

      .rag-source {
        font-size: 11px;
        color: #6b7280;
        margin-top: 5px;
      }
    `;
    document.head.appendChild(styles);
  }

  setupEventListeners() {
    // Open/close chat
    const openBtn = document.getElementById('rag-open-btn');
    const closeBtn = document.getElementById('rag-close-btn');
    const widget = document.getElementById('rag-chat-widget');

    openBtn.addEventListener('click', () => {
      widget.classList.remove('rag-chat-hidden');
      openBtn.style.display = 'none';
    });

    closeBtn.addEventListener('click', () => {
      widget.classList.add('rag-chat-hidden');
      openBtn.style.display = 'flex';
    });

    // Send message
    const sendBtn = document.getElementById('rag-send-btn');
    const userInput = document.getElementById('rag-user-input');

    const sendMessage = () => {
      const message = userInput.value.trim();
      if (!message) return;

      this.addMessage(message, 'user');
      userInput.value = '';

      // Show typing indicator
      const typingId = this.showTypingIndicator();

      // Prepare request
      const requestData = {
        book_id: this.bookId,
        query: message
      };

      // Add user selected text if available
      if (this.userSelectedText) {
        requestData.user_selected_text = this.userSelectedText;
      }

      // Send to backend
      fetch(`${this.apiUrl}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(requestData)
      })
      .then(response => response.json())
      .then(data => {
        this.removeTypingIndicator(typingId);
        this.addMessage(data.answer, 'bot', data.sources);
      })
      .catch(error => {
        this.removeTypingIndicator(typingId);
        this.addMessage('Sorry, I encountered an error. Please try again.', 'bot');
        console.error('Error:', error);
      });
    };

    sendBtn.addEventListener('click', sendMessage);
    userInput.addEventListener('keypress', (e) => {
      if (e.key === 'Enter') {
        sendMessage();
      }
    });
  }

  setupTextSelection() {
    // Monitor text selection on the page
    document.addEventListener('mouseup', () => {
      const selectedText = window.getSelection().toString().trim();

      if (selectedText) {
        this.userSelectedText = selectedText;
        this.showTextSelectionIndicator();
      } else {
        this.userSelectedText = null;
        this.hideTextSelectionIndicator();
      }
    });
  }

  showTextSelectionIndicator() {
    const indicator = document.getElementById('rag-text-selection-indicator');
    if (indicator) {
      indicator.style.display = 'block';
    }
  }

  hideTextSelectionIndicator() {
    const indicator = document.getElementById('rag-text-selection-indicator');
    if (indicator) {
      indicator.style.display = 'none';
    }
  }

  addMessage(text, sender, sources = null) {
    const messagesContainer = document.getElementById('rag-chat-messages');

    const messageDiv = document.createElement('div');
    messageDiv.className = `rag-message rag-${sender}-message`;
    messageDiv.textContent = text;

    // Add sources if provided
    if (sources && sources.length > 0) {
      const sourcesDiv = document.createElement('div');
      sourcesDiv.className = 'rag-source';

      // Show first source as reference
      const firstSource = sources[0];
      sourcesDiv.textContent = `Reference: Page ${firstSource.page}`;

      messageDiv.appendChild(sourcesDiv);
    }

    messagesContainer.appendChild(messageDiv);
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  showTypingIndicator() {
    const messagesContainer = document.getElementById('rag-chat-messages');

    const typingDiv = document.createElement('div');
    typingDiv.id = `typing-${Date.now()}`;
    typingDiv.className = 'rag-typing-indicator';
    typingDiv.innerHTML = 'Thinking...';

    messagesContainer.appendChild(typingDiv);
    messagesContainer.scrollTop = messagesContainer.scrollHeight;

    return typingDiv.id;
  }

  removeTypingIndicator(typingId) {
    const typingElement = document.getElementById(typingId);
    if (typingElement) {
      typingElement.remove();
    }
  }

  // Public method to set book ID
  setBookId(bookId) {
    this.bookId = bookId;
  }

  // Public method to show the chat
  show() {
    const widget = document.getElementById('rag-chat-widget');
    const openBtn = document.getElementById('rag-open-btn');

    widget.classList.remove('rag-chat-hidden');
    openBtn.style.display = 'none';
  }

  // Public method to hide the chat
  hide() {
    const widget = document.getElementById('rag-chat-widget');
    const openBtn = document.getElementById('rag-open-btn');

    widget.classList.add('rag-chat-hidden');
    openBtn.style.display = 'flex';
  }
}

// Make the widget available globally
window.RAGChatWidget = RAGChatWidget;

// Auto-initialize if data attributes are present
document.addEventListener('DOMContentLoaded', () => {
  const widgetElement = document.querySelector('[data-rag-chat]');
  if (widgetElement) {
    const options = {
      apiUrl: widgetElement.getAttribute('data-api-url') || 'http://localhost:8000',
      bookId: widgetElement.getAttribute('data-book-id'),
      containerId: widgetElement.getAttribute('data-container-id') || 'rag-chat-container'
    };

    // Initialize the widget
    window.ragChat = new RAGChatWidget(options);
  }
});