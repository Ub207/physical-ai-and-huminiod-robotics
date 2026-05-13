import { useState, useEffect, useCallback, useRef } from 'react';
import { useLanguage } from './useLanguage';

/**
 * useTranslation - Hook for translating chapter content
 *
 * Handles translation API calls, caching, and error management
 * Reuses cache from 001-urdu-translation feature (FR-009)
 *
 * @param {string} chapterId - Unique chapter identifier
 * @param {string} content - Original English content
 * @returns {Object} { translatedContent, isLoading, error, translate }
 */
export function useTranslation(chapterId, content) {
  const { currentLanguage, _setIsTranslating, _setTranslationError } = useLanguage();
  const [translatedContent, setTranslatedContent] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const prevContentRef = useRef(content);

  // Update ref when content changes
  useEffect(() => {
    if (content !== prevContentRef.current) {
      prevContentRef.current = content;
      // Reset translation state when content changes
      if (currentLanguage === 'urdu') {
        setTranslatedContent(null);
      }
    }
  }, [content, currentLanguage]);

  // Cache key shared with 001-urdu-translation (FR-009, FR-011)
  const getCacheKey = useCallback((chapId, lang) => {
    return `translation_cache_${chapId}_${lang}`;
  }, []);

  // Get cached translation
  const getCachedTranslation = useCallback((chapId, lang) => {
    if (typeof window === 'undefined') return null;

    try {
      const cacheKey = getCacheKey(chapId, lang);
      const cached = localStorage.getItem(cacheKey);

      if (cached) {
        const parsed = JSON.parse(cached);
        // Check if cache is still valid (7-day TTL per plan.md)
        const now = Date.now();
        const cacheAge = now - (parsed.timestamp || 0);
        const sevenDays = 7 * 24 * 60 * 60 * 1000;

        if (cacheAge < sevenDays && parsed.content) {
          return parsed.content;
        } else {
          // Cache expired, remove it
          localStorage.removeItem(cacheKey);
        }
      }
    } catch (err) {
      console.error('Cache read error:', err);
    }

    return null;
  }, [getCacheKey]);

  // Save translation to cache
  const setCachedTranslation = useCallback((chapId, lang, translatedText) => {
    if (typeof window === 'undefined') return;

    try {
      const cacheKey = getCacheKey(chapId, lang);
      const cacheData = {
        content: translatedText,
        timestamp: Date.now(),
        version: '1.0'
      };

      localStorage.setItem(cacheKey, JSON.stringify(cacheData));
    } catch (err) {
      console.error('Cache write error:', err);
      // Handle localStorage quota exceeded (FR-013)
      if (err.name === 'QuotaExceededError') {
        console.warn('localStorage quota exceeded. Translation will not be cached.');
      }
    }
  }, [getCacheKey]);

  // Translate content via API
  const translate = useCallback(async () => {
    if (!chapterId || !content || currentLanguage === 'english') {
      setTranslatedContent(null);
      return;
    }

    // Check cache first (FR-009)
    const cached = getCachedTranslation(chapterId, currentLanguage);
    if (cached) {
      setTranslatedContent(cached);
      setIsLoading(false);
      setError(null);
      return;
    }

    // Fetch translation from API
    setIsLoading(true);
    setError(null);
    if (_setIsTranslating) _setIsTranslating(true);

    try {
      // Use same API endpoint as 001-urdu-translation
      const apiUrl = typeof window !== 'undefined' && window.location.hostname === 'localhost'
        ? 'http://localhost:8000'
        : 'https://ubaid-ai-rag-chatbot.hf.space';

      console.log('Calling translation API for chapter:', chapterId);

      const response = await fetch(`${apiUrl}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          text: content,
          target_language: currentLanguage
        })
      });

      if (!response.ok) {
        const errorText = await response.text();
        throw new Error(`Translation API error: ${response.status} - ${errorText}`);
      }

      const data = await response.json();
      const translated = data.translated_text || data.answer || '';

      if (!translated) {
        throw new Error('Empty translation received');
      }

      console.log('Translation received, length:', translated.length);

      // Cache the translation (FR-009)
      setCachedTranslation(chapterId, currentLanguage, translated);

      setTranslatedContent(translated);
      setError(null);
      if (_setTranslationError) _setTranslationError(null);
    } catch (err) {
      console.error('Translation error:', err);
      setError(err.message || 'Translation failed. Please try again.');
      if (_setTranslationError) _setTranslationError(err.message);
      setTranslatedContent(null);
    } finally {
      setIsLoading(false);
      if (_setIsTranslating) _setIsTranslating(false);
    }
  }, [chapterId, content, currentLanguage, getCachedTranslation, setCachedTranslation, _setIsTranslating, _setTranslationError]);

  // Auto-translate when language changes (FR-008)
  useEffect(() => {
    if (currentLanguage === 'urdu' && content && content.length > 0) {
      // Check cache first
      const cached = getCachedTranslation(chapterId, currentLanguage);
      if (cached) {
        setTranslatedContent(cached);
        return;
      }
      // If no cache, translate
      translate();
    } else {
      setTranslatedContent(null);
      setIsLoading(false);
      setError(null);
    }
  }, [currentLanguage, content, chapterId, translate, getCachedTranslation]);

  return {
    translatedContent,
    isLoading,
    error,
    translate
  };
}

export default useTranslation;
