/**
 * languageStorage - localStorage utilities for language preference
 *
 * Handles secure storage and retrieval of user language preference
 * Namespaced key to prevent collisions (FR-006)
 */

const LANGUAGE_PREFERENCE_KEY = 'userLanguagePreference';

/**
 * Get user language preference from localStorage or sessionStorage fallback
 *
 * @returns {string|null} 'english' | 'urdu' | null
 */
export function getLanguagePreference() {
  if (typeof window === 'undefined') return null;

  try {
    // Try localStorage first
    const preference = localStorage.getItem(LANGUAGE_PREFERENCE_KEY);
    if (preference === 'english' || preference === 'urdu') {
      return preference;
    }

    // Fallback to sessionStorage
    const sessionPreference = sessionStorage.getItem(LANGUAGE_PREFERENCE_KEY);
    if (sessionPreference === 'english' || sessionPreference === 'urdu') {
      return sessionPreference;
    }

    return null;
  } catch (error) {
    console.error('Error reading language preference:', error);
    return null;
  }
}

/**
 * Set user language preference in localStorage
 *
 * @param {string} language - 'english' | 'urdu'
 * @returns {boolean} Success status
 */
export function setLanguagePreference(language) {
  if (typeof window === 'undefined') return false;

  if (language !== 'english' && language !== 'urdu') {
    console.error(`Invalid language: ${language}`);
    return false;
  }

  try {
    localStorage.setItem(LANGUAGE_PREFERENCE_KEY, language);
    return true;
  } catch (error) {
    console.error('Error setting language preference:', error);

    // Handle localStorage quota exceeded (FR-013)
    if (error.name === 'QuotaExceededError') {
      console.warn('localStorage quota exceeded. Language preference will not persist across sessions.');

      // Fallback to sessionStorage
      try {
        sessionStorage.setItem(LANGUAGE_PREFERENCE_KEY, language);
        console.info('Using sessionStorage fallback for language preference.');
        return true;
      } catch (sessionError) {
        console.error('sessionStorage also failed:', sessionError);
        return false;
      }
    }

    return false;
  }
}

/**
 * Clear user language preference from localStorage
 *
 * @returns {boolean} Success status
 */
export function clearLanguagePreference() {
  if (typeof window === 'undefined') return false;

  try {
    localStorage.removeItem(LANGUAGE_PREFERENCE_KEY);
    sessionStorage.removeItem(LANGUAGE_PREFERENCE_KEY); // Clear fallback too
    return true;
  } catch (error) {
    console.error('Error clearing language preference:', error);
    return false;
  }
}

/**
 * Check if localStorage is available and has space
 *
 * @returns {boolean} True if localStorage is available with space
 */
export function isLocalStorageAvailable() {
  if (typeof window === 'undefined') return false;

  try {
    const testKey = '__localStorage_test__';
    localStorage.setItem(testKey, 'test');
    localStorage.removeItem(testKey);
    return true;
  } catch (error) {
    return false;
  }
}

export default {
  getLanguagePreference,
  setLanguagePreference,
  clearLanguagePreference,
  isLocalStorageAvailable
};
