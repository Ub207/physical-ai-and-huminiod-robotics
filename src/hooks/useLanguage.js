import { useContext } from 'react';
import { LanguageContext } from '../contexts/LanguageContext';

/**
 * useLanguage - Custom hook to consume LanguageContext
 *
 * Provides access to current language state and switching functionality
 *
 * @returns {Object} Language context value
 * @throws {Error} If used outside LanguageProvider
 *
 * @example
 * function MyComponent() {
 *   const { currentLanguage, setLanguage, isTranslating } = useLanguage();
 *
 *   return (
 *     <button onClick={() => setLanguage('urdu')} disabled={isTranslating}>
 *       Switch to Urdu
 *     </button>
 *   );
 * }
 */
export function useLanguage() {
  const context = useContext(LanguageContext);

  if (!context) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }

  return context;
}

export default useLanguage;
