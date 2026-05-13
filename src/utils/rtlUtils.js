/**
 * rtlUtils - RTL (Right-to-Left) support utilities
 *
 * Manages HTML dir attribute for full RTL support (FR-016)
 * Supports Urdu, Arabic, and other RTL languages
 */

/**
 * Set HTML dir attribute for RTL support
 *
 * @param {string} direction - 'ltr' | 'rtl'
 */
export function setHtmlDir(direction) {
  if (typeof document === 'undefined') return;

  if (direction !== 'ltr' && direction !== 'rtl') {
    console.error(`Invalid direction: ${direction}. Must be 'ltr' or 'rtl'.`);
    return;
  }

  try {
    // Set direction on html element
    document.documentElement.setAttribute('dir', direction);
    document.documentElement.setAttribute('lang', direction === 'rtl' ? 'ur' : 'en');

    // Also set on body for better CSS support
    document.body.setAttribute('dir', direction);
    document.body.setAttribute('lang', direction === 'rtl' ? 'ur' : 'en');

    // Add/remove RTL class for CSS targeting
    if (direction === 'rtl') {
      document.body.classList.add('rtl-mode');
      document.documentElement.classList.add('rtl-mode');
    } else {
      document.body.classList.remove('rtl-mode');
      document.documentElement.classList.remove('rtl-mode');
    }

    console.log('[RTL] Direction set to:', direction);
  } catch (error) {
    console.error('Error setting HTML dir attribute:', error);
  }
}

/**
 * Get current HTML dir attribute
 *
 * @returns {string} 'ltr' | 'rtl'
 */
export function getHtmlDir() {
  if (typeof document === 'undefined') return 'ltr';

  return document.documentElement.getAttribute('dir') || 'ltr';
}

/**
 * Check if current direction is RTL
 *
 * @returns {boolean} True if RTL
 */
export function isRTL() {
  return getHtmlDir() === 'rtl';
}

/**
 * Get direction for a specific language
 *
 * @param {string} language - Language code
 * @returns {string} 'ltr' | 'rtl'
 */
export function getDirectionForLanguage(language) {
  const rtlLanguages = ['urdu', 'arabic', 'hebrew', 'persian', 'ur', 'ar', 'he', 'fa'];
  return rtlLanguages.includes(language.toLowerCase()) ? 'rtl' : 'ltr';
}

export default {
  setHtmlDir,
  getHtmlDir,
  isRTL,
  getDirectionForLanguage
};
