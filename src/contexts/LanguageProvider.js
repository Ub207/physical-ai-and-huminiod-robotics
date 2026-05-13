import React, { useState, useEffect, useMemo, useCallback } from 'react';
import PropTypes from 'prop-types';
import { LanguageContext } from './LanguageContext';
import { getLanguagePreference, setLanguagePreference } from '../utils/languageStorage';
import { setHtmlDir } from '../utils/rtlUtils';

/**
 * LanguageProvider - Global language state management
 *
 * Manages language preference, persistence, and RTL support
 * Performance: <200ms UI response per constitution requirements
 */
export function LanguageProvider({ children, initialLanguage, apiUrl }) {
  const [currentLanguage, setCurrentLanguage] = useState(() => {
    // Initialize from localStorage or prop
    if (initialLanguage) return initialLanguage;
    if (typeof window !== 'undefined') {
      return getLanguagePreference() || 'english';
    }
    return 'english';
  });

  const [isTranslating, setIsTranslating] = useState(false);
  const [translationError, setTranslationError] = useState(null);

  // Initialize RTL direction immediately on mount
  useEffect(() => {
    if (typeof window !== 'undefined') {
      const savedLang = getLanguagePreference();
      const lang = savedLang || currentLanguage;
      setHtmlDir(lang === 'urdu' ? 'rtl' : 'ltr');
      console.log('[Language Init] RTL set to:', lang === 'urdu' ? 'rtl' : 'ltr');
    }
  }, []); // Run once on mount

  // Persist language preference and update HTML dir attribute
  useEffect(() => {
    if (typeof window !== 'undefined') {
      const startTime = performance.now();

      // Save to localStorage
      setLanguagePreference(currentLanguage);

      // Update HTML dir attribute for RTL support
      setHtmlDir(currentLanguage === 'urdu' ? 'rtl' : 'ltr');

      // Performance logging (FR-017)
      const endTime = performance.now();
      const duration = endTime - startTime;

      if (process.env.NODE_ENV === 'development') {
        console.log(`[Language Switch] ${currentLanguage} - ${duration.toFixed(2)}ms`);
      }

      // Analytics logging in production
      if (process.env.NODE_ENV === 'production' && typeof window.gtag !== 'undefined') {
        window.gtag('event', 'language_switch', {
          language: currentLanguage,
          duration_ms: duration
        });
      }
    }
  }, [currentLanguage]);

  // Language setter with error handling
  const setLanguage = useCallback((language) => {
    if (language !== 'english' && language !== 'urdu') {
      console.error(`Invalid language: ${language}. Must be 'english' or 'urdu'.`);
      return;
    }

    setTranslationError(null);
    setCurrentLanguage(language);
  }, []);

  // Retry translation after error
  const retryTranslation = useCallback(() => {
    setTranslationError(null);
    setIsTranslating(false);
    // Trigger re-translation by toggling language state
    setCurrentLanguage(prev => prev);
  }, []);

  // Memoize context value to prevent unnecessary re-renders
  const contextValue = useMemo(() => ({
    currentLanguage,
    setLanguage,
    isTranslating,
    translationError,
    retryTranslation,
    // Internal setters for hooks
    _setIsTranslating: setIsTranslating,
    _setTranslationError: setTranslationError
  }), [currentLanguage, setLanguage, isTranslating, translationError, retryTranslation]);

  return (
    <LanguageContext.Provider value={contextValue}>
      {children}
    </LanguageContext.Provider>
  );
}

LanguageProvider.propTypes = {
  children: PropTypes.node.isRequired,
  initialLanguage: PropTypes.oneOf(['english', 'urdu']),
  apiUrl: PropTypes.string
};

LanguageProvider.defaultProps = {
  initialLanguage: null,
  apiUrl: null
};

export default LanguageProvider;
