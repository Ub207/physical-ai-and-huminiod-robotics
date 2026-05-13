import { createContext } from 'react';

/**
 * Language Context for global language state management
 * Provides language switching functionality across the entire application
 */
export const LanguageContext = createContext({
  // Current active language ('english' | 'urdu')
  currentLanguage: 'english',

  // Function to change language
  setLanguage: () => {
    throw new Error('LanguageProvider not mounted. Wrap your app with LanguageProvider.');
  },

  // Loading state during translation
  isTranslating: false,

  // Translation error state
  translationError: null,

  // Retry failed translation
  retryTranslation: () => {
    throw new Error('LanguageProvider not mounted. Wrap your app with LanguageProvider.');
  }
});

export default LanguageContext;
