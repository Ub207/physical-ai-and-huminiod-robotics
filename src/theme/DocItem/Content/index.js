import React, { useEffect, useState } from 'react';
import Content from '@theme-original/DocItem/Content';
import TranslationButton from '@site/src/components/translation/TranslationButton';
import { useLanguage } from '@site/src/hooks/useLanguage';

export default function ContentWrapper(props) {
  const { currentLanguage } = useLanguage();
  const [translatedContent, setTranslatedContent] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const apiUrl = typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://ubaid-ai-rag-chatbot.hf.space';

  // Get chapter ID
  const getChapterId = () => {
    if (typeof window === 'undefined') return 'unknown';
    const path = window.location.pathname;
    return path.replace(/^\/physical-ai-humanoid-robotics\//, '').replace(/\/$/, '') || 'introduction';
  };

  // Get cached translation
  const getCached = (chapterId) => {
    try {
      const cacheKey = `translation_cache_${chapterId}_urdu`;
      const cached = localStorage.getItem(cacheKey);
      if (cached) {
        const { content, timestamp } = JSON.parse(cached);
        const TTL = 30 * 24 * 60 * 60 * 1000; // 30 days
        if (Date.now() - timestamp < TTL && content) {
          return content;
        }
      }
    } catch (e) {
      console.error('Cache read error:', e);
    }
    return null;
  };

  // Save to cache
  const saveToCache = (chapterId, content) => {
    try {
      const cacheKey = `translation_cache_${chapterId}_urdu`;
      localStorage.setItem(cacheKey, JSON.stringify({
        content,
        timestamp: Date.now()
      }));
    } catch (e) {
      console.error('Cache write error:', e);
    }
  };

  // Translate function
  const translate = async (text, chapterId) => {
    try {
      const response = await fetch(`${apiUrl}/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: text,
          target_language: 'urdu'
        })
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();
      const translated = data.translated_text || '';

      if (translated) {
        const html = `<div style="direction: rtl; text-align: right; font-family: 'Jameel Noori Nastaleeq', 'Lateef', sans-serif;">${translated}</div>`;
        saveToCache(chapterId, html);
        return html;
      }
    } catch (err) {
      console.error('Translation error:', err);
      throw err;
    }
    return null;
  };

  // When language is Urdu, translate or load from cache
  useEffect(() => {
    if (currentLanguage !== 'urdu') {
      setTranslatedContent(null);
      return;
    }

    const chapterId = getChapterId();
    const article = document.querySelector('article');

    if (!article) return;

    const originalHtml = article.innerHTML;

    // Check cache first
    const cached = getCached(chapterId);
    if (cached) {
      setTranslatedContent(cached);
      return;
    }

    // Need to translate
    setIsLoading(true);
    setError(null);

    translate(originalHtml, chapterId)
      .then((translated) => {
        if (translated) {
          setTranslatedContent(translated);
        } else {
          setError('Translation failed');
        }
      })
      .catch((err) => {
        setError(err.message || 'Translation error');
      })
      .finally(() => {
        setIsLoading(false);
      });

  }, [currentLanguage]);

  // Apply translation to DOM
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const article = document.querySelector('article');
    if (!article) return;

    if (currentLanguage === 'urdu' && translatedContent) {
      article.innerHTML = translatedContent;
    }
  }, [translatedContent, currentLanguage]);

  return (
    <>
      {/* Translation button */}
      <TranslationButton />

      {/* Loading indicator */}
      {isLoading && (
        <div style={{
          padding: '1rem',
          textAlign: 'center',
          background: '#eff6ff',
          borderRadius: '8px',
          marginBottom: '1rem',
          color: '#1e3a8a'
        }}>
          🔄 Translating to Urdu...
        </div>
      )}

      {/* Error message */}
      {error && (
        <div style={{
          padding: '1rem',
          background: '#fee2e2',
          borderRadius: '8px',
          marginBottom: '1rem',
          color: '#991b1b'
        }}>
          ❌ Translation failed: {error}
        </div>
      )}

      {/* Original content */}
      <Content {...props} />
    </>
  );
}
