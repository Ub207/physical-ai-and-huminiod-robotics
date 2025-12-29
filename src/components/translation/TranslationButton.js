import React, { useState, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

/**
 * Translation button component for chapter-level Urdu translation
 * - Only visible to logged-in users
 * - Awards 50 bonus points on first translation per chapter
 * - Caches translations in localStorage (7-day TTL)
 * - Rate limited to 1 request per 2 seconds
 */
export default function TranslationButton() {
  const { siteConfig } = useDocusaurusContext();
  const [currentUser, setCurrentUser] = useState(null);
  const [isTranslated, setIsTranslated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [lastRequestTime, setLastRequestTime] = useState(0);
  const [originalContent, setOriginalContent] = useState(null); // Store original HTML for toggle

  // API URL - use localhost for dev, HF Space for production
  const apiUrl = typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://ubaid-ai-rag-chatbot.hf.space';

  useEffect(() => {
    // CRITICAL: Reset state on mount - each chapter is independent
    setIsTranslated(false);
    setOriginalContent(null);
    setError(null);

    // Check authentication status
    const user = localStorage.getItem('currentUser');
    setCurrentUser(user);

    // Check if current chapter is already translated (from cache)
    if (user) {
      const chapterId = getChapterId();
      const cached = getCachedTranslation(chapterId);
      if (cached) {
        // Store original content before applying cached translation
        const original = getChapterContent();
        setOriginalContent(original);
        setChapterContent(cached);
        setIsTranslated(true);
      }
    }

    // Cleanup: when component unmounts (user navigates away), ensure state resets
    return () => {
      setIsTranslated(false);
      setOriginalContent(null);
    };
  }, []); // Empty dependency array = run once on mount per chapter

  const getChapterId = () => {
    // Extract chapter ID from URL path
    if (typeof window === 'undefined') return 'unknown';
    const path = window.location.pathname;
    // Remove base URL and trailing slashes
    return path.replace(/^\/physical-ai-humanoid-robotics\//, '').replace(/\/$/, '') || 'introduction';
  };

  const getCachedTranslation = (chapterId) => {
    try {
      const cacheKey = `translation_cache_${chapterId}_urdu`;
      const cached = localStorage.getItem(cacheKey);
      if (!cached) return null;

      const { content, timestamp } = JSON.parse(cached);
      const TTL = 7 * 24 * 60 * 60 * 1000; // 7 days in milliseconds

      if (Date.now() - timestamp > TTL) {
        // Cache expired
        localStorage.removeItem(cacheKey);
        return null;
      }

      return content;
    } catch (e) {
      return null;
    }
  };

  const setCachedTranslation = (chapterId, content) => {
    try {
      const cacheKey = `translation_cache_${chapterId}_urdu`;
      localStorage.setItem(cacheKey, JSON.stringify({
        content,
        timestamp: Date.now()
      }));
    } catch (e) {
      console.error('Failed to cache translation:', e);
    }
  };

  const awardBonusPoints = (chapterId) => {
    try {
      const bonusData = JSON.parse(localStorage.getItem('userBonusPoints') || '{}');

      if (!bonusData[currentUser]) {
        bonusData[currentUser] = { totalPoints: 0, translationHistory: [] };
      }

      // Check if user already earned points for this chapter
      const alreadyAwarded = bonusData[currentUser].translationHistory.some(
        entry => entry.chapterId === chapterId
      );

      if (!alreadyAwarded) {
        bonusData[currentUser].totalPoints += 50;
        bonusData[currentUser].translationHistory.push({
          chapterId,
          timestamp: new Date().toISOString(),
          pointsAwarded: 50,
          language: 'urdu'
        });

        localStorage.setItem('userBonusPoints', JSON.stringify(bonusData));

        // Trigger UI update
        window.dispatchEvent(new Event('storage'));

        return true; // Points awarded
      }

      return false; // Already awarded
    } catch (e) {
      console.error('Failed to award bonus points:', e);
      return false;
    }
  };

  const getChapterContent = () => {
    // Get the main article content
    if (typeof document === 'undefined') return '';
    const article = document.querySelector('article');
    return article ? article.innerHTML : '';
  };

  const setChapterContent = (html) => {
    if (typeof document === 'undefined') return;
    const article = document.querySelector('article');
    if (article) {
      article.innerHTML = html;
    }
  };

  const translateContent = async () => {
    const chapterId = getChapterId();

    // Rate limiting check
    const now = Date.now();
    if (now - lastRequestTime < 2000) {
      setError('Please wait 2 seconds between translation requests');
      setTimeout(() => setError(null), 3000);
      return;
    }
    setLastRequestTime(now);

    // Check cache first
    const cached = getCachedTranslation(chapterId);
    if (cached) {
      setChapterContent(cached);
      setIsTranslated(true);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const content = getChapterContent();

      // Store original content for toggle functionality
      if (!originalContent) {
        setOriginalContent(content);
      }

      // Extract text content while preserving structure
      const tempDiv = document.createElement('div');
      tempDiv.innerHTML = content;

      // Get text content (this will need refinement for better formatting preservation)
      const textContent = tempDiv.textContent || tempDiv.innerText;

      const response = await fetch(`${apiUrl}/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: textContent,
          target_language: 'urdu',
          preserve_formatting: true
        })
      });

      if (!response.ok) {
        throw new Error(`Translation failed: ${response.status}`);
      }

      const data = await response.json();
      const translatedHtml = formatTranslatedContent(data.translated_text);

      // Cache the translation
      setCachedTranslation(chapterId, translatedHtml);

      // Update the page
      setChapterContent(translatedHtml);
      setIsTranslated(true);

      // Award bonus points
      const pointsAwarded = awardBonusPoints(chapterId);

      if (pointsAwarded) {
        // Show success message
        showNotification('🎉 Translation complete! +50 bonus points earned!', 'success');
      }

    } catch (err) {
      console.error('Translation error:', err);
      setError('Translation failed. Please try again.');
      setTimeout(() => setError(null), 5000);
    } finally {
      setIsLoading(false);
    }
  };

  const formatTranslatedContent = (text) => {
    // Basic formatting - wrap in paragraphs
    // This is a simplified version; real implementation would preserve markdown structure
    return `<div style="direction: rtl; text-align: right; font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', sans-serif;">${text.split('\n').map(p => `<p>${p}</p>`).join('')}</div>`;
  };

  const showOriginal = () => {
    // Restore original content without page reload
    if (originalContent) {
      setChapterContent(originalContent);
      setIsTranslated(false);
    } else {
      // Fallback: reload if original content not stored (shouldn't happen)
      if (typeof window !== 'undefined') {
        window.location.reload();
      }
    }
  };

  const showNotification = (message, type = 'info') => {
    // Simple notification (could be enhanced with a toast library)
    if (typeof window !== 'undefined') {
      alert(message);
    }
  };

  // Don't render if user is not logged in
  if (!currentUser) {
    return null;
  }

  return (
    <div style={styles.container}>
      {error && <div style={styles.error}>{error}</div>}

      <div style={styles.buttonGroup}>
        {!isTranslated ? (
          <button
            onClick={translateContent}
            disabled={isLoading}
            style={{...styles.button, ...styles.translateBtn}}
          >
            {isLoading ? '⏳ Translating...' : '🌐 Translate to Urdu'}
          </button>
        ) : (
          <>
            <button
              onClick={showOriginal}
              style={{...styles.button, ...styles.originalBtn}}
            >
              🔙 Show Original English
            </button>
            <span style={styles.badge}>✅ Translated to Urdu</span>
          </>
        )}
      </div>
    </div>
  );
}

const styles = {
  container: {
    padding: '16px',
    backgroundColor: '#f8f9fa',
    borderRadius: '8px',
    marginBottom: '24px',
    border: '1px solid #dee2e6'
  },
  buttonGroup: {
    display: 'flex',
    alignItems: 'center',
    gap: '12px',
    flexWrap: 'wrap'
  },
  button: {
    padding: '10px 20px',
    fontSize: '15px',
    fontWeight: '500',
    border: 'none',
    borderRadius: '6px',
    cursor: 'pointer',
    transition: 'all 0.2s',
    display: 'inline-flex',
    alignItems: 'center',
    gap: '6px'
  },
  translateBtn: {
    backgroundColor: '#4f46e5',
    color: 'white'
  },
  originalBtn: {
    backgroundColor: '#6c757d',
    color: 'white'
  },
  badge: {
    fontSize: '13px',
    color: '#28a745',
    backgroundColor: '#d4edda',
    padding: '4px 12px',
    borderRadius: '12px',
    fontWeight: '500'
  },
  error: {
    padding: '8px 12px',
    backgroundColor: '#f8d7da',
    color: '#721c24',
    borderRadius: '4px',
    marginBottom: '12px',
    fontSize: '14px'
  }
};
