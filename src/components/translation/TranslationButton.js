import React, { useState, useCallback } from 'react';

/**
 * Translation button component for chapter-level Urdu translation
 * - Visible to ALL users (no login required)
 * - Translates chapter to Urdu on button click
 * - Caches translations in localStorage (7-day TTL)
 */
export default function TranslationButton() {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [translated, setTranslated] = useState(false);

  // API URL
  const apiUrl = typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://ubaid-ai-rag-chatbot.hf.space';

  // Get chapter ID from URL
  const getChapterId = useCallback(() => {
    if (typeof window === 'undefined') return 'unknown';
    const path = window.location.pathname;
    return path.replace(/^\/physical-ai-humanoid-robotics\//, '').replace(/\/$/, '') || 'introduction';
  }, []);

  // Get chapter content from DOM
  const getChapterContent = useCallback(() => {
    if (typeof document === 'undefined') return '';
    const article = document.querySelector('article');
    return article ? article.innerHTML : '';
  }, []);

  // Set chapter content in DOM
  const setChapterContent = useCallback((html) => {
    if (typeof document === 'undefined') return;
    const article = document.querySelector('article');
    if (article) {
      article.innerHTML = html;
    }
  }, []);

  // Save translation to cache
  const saveToCache = useCallback((chapterId, html) => {
    try {
      const cacheKey = `translation_cache_${chapterId}_urdu`;
      localStorage.setItem(cacheKey, JSON.stringify({
        content: html,
        timestamp: Date.now()
      }));
    } catch (e) {
      console.error('Failed to cache:', e);
    }
  }, []);

  // Check cache
  const getCachedTranslation = useCallback((chapterId) => {
    try {
      const cacheKey = `translation_cache_${chapterId}_urdu`;
      const cached = localStorage.getItem(cacheKey);
      if (!cached) return null;
      const { content, timestamp } = JSON.parse(cached);
      const TTL = 7 * 24 * 60 * 60 * 1000;
      if (Date.now() - timestamp > TTL) {
        localStorage.removeItem(cacheKey);
        return null;
      }
      return content;
    } catch (e) {
      return null;
    }
  }, []);

  // Translate content
  const translate = useCallback(async () => {
    const chapterId = getChapterId();
    const originalContent = getChapterContent();

    if (!originalContent) {
      setError('Content not found');
      return;
    }

    // Check cache first
    const cached = getCachedTranslation(chapterId);
    if (cached) {
      setChapterContent(cached);
      setTranslated(true);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${apiUrl}/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: originalContent,
          target_language: 'urdu'
        })
      });

      if (!response.ok) {
        let errMsg = `Translation failed (${response.status})`;
        try {
          const errData = await response.json();
          if (errData.detail) errMsg = errData.detail;
        } catch (_) {}
        throw new Error(errMsg);
      }

      const data = await response.json();
      const translatedText = data.translated_text || '';

      if (!translatedText) {
        throw new Error('Empty translation received');
      }

      // Format for RTL display
      const translatedHtml = `<div style="direction: rtl; text-align: right; font-family: 'Jameel Noori Nastaleeq', 'Lateef', sans-serif; font-size: 1.1rem; line-height: 1.8;">${translatedText}</div>`;

      // Cache and display
      saveToCache(chapterId, translatedHtml);
      setChapterContent(translatedHtml);
      setTranslated(true);

    } catch (err) {
      console.error('Translation error:', err);
      setError(err.message || 'Translation failed');
    } finally {
      setIsLoading(false);
    }
  }, [getChapterId, getChapterContent, apiUrl, getCachedTranslation, saveToCache, setChapterContent]);

  // Show original English
  const showOriginal = useCallback(() => {
    const chapterId = getChapterId();
    const cacheKey = `translation_cache_${chapterId}_urdu`;
    localStorage.removeItem(cacheKey);
    window.location.reload();
  }, [getChapterId]);

  return (
    <div style={styles.container}>
      {error && <div style={styles.error}>{error}</div>}

      <div style={styles.buttonGroup}>
        {!translated ? (
          <button
            onClick={translate}
            disabled={isLoading}
            style={styles.translateBtn}
          >
            {isLoading ? '⏳ Translating...' : '🌐 Translate to Urdu'}
          </button>
        ) : (
          <>
            <button onClick={showOriginal} style={styles.originalBtn}>
              🔙 Show Original English
            </button>
            <span style={styles.badge}>✅ Translated</span>
          </>
        )}
      </div>
    </div>
  );
}

const styles = {
  container: {
    padding: '16px',
    backgroundColor: '#f0f9ff',
    borderRadius: '8px',
    marginBottom: '24px',
    border: '1px solid #bae6fd'
  },
  buttonGroup: {
    display: 'flex',
    alignItems: 'center',
    gap: '12px',
    flexWrap: 'wrap'
  },
  translateBtn: {
    padding: '10px 20px',
    fontSize: '15px',
    fontWeight: '500',
    backgroundColor: '#0284c7',
    color: 'white',
    border: 'none',
    borderRadius: '6px',
    cursor: 'pointer',
    transition: 'all 0.2s'
  },
  originalBtn: {
    padding: '10px 20px',
    fontSize: '15px',
    fontWeight: '500',
    backgroundColor: '#64748b',
    color: 'white',
    border: 'none',
    borderRadius: '6px',
    cursor: 'pointer'
  },
  badge: {
    fontSize: '13px',
    color: '#0ea5e9',
    backgroundColor: '#e0f2fe',
    padding: '4px 12px',
    borderRadius: '12px',
    fontWeight: '500'
  },
  error: {
    padding: '8px 12px',
    backgroundColor: '#fef2f2',
    color: '#dc2626',
    borderRadius: '4px',
    marginBottom: '12px',
    fontSize: '14px'
  }
};
