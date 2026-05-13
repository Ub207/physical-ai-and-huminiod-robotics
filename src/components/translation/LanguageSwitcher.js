import React from 'react';
import { useLanguage } from '@site/src/hooks/useLanguage';
import styles from './LanguageSwitcher.module.css';

/**
 * LanguageSwitcher - Global language dropdown component
 *
 * Allows users to switch between English and Urdu for the entire textbook
 * Features:
 * - Dropdown UI with checkmark for active language (FR-015)
 * - Dark blue theme (#1e3a8a, #111827) - NO GREEN
 * - Accessible (ARIA labels, keyboard navigation)
 * - Mobile responsive
 * - <200ms UI response time
 */
export default function LanguageSwitcher() {
  const { currentLanguage, setLanguage, isTranslating } = useLanguage();

  const handleLanguageChange = (event) => {
    const newLanguage = event.target.value;
    if (newLanguage !== currentLanguage) {
      setLanguage(newLanguage);
    }
  };

  return (
    <div className={styles.languageSwitcherContainer}>
      <label htmlFor="language-select" className={styles.visuallyHidden}>
        Select language for textbook
      </label>
      <select
        id="language-select"
        className={styles.languageDropdown}
        value={currentLanguage}
        onChange={handleLanguageChange}
        disabled={isTranslating}
        aria-label="Select language"
        aria-describedby="language-description"
      >
        <option value="english" className={styles.languageOption}>
          {currentLanguage === 'english' ? '✓ ' : ''}English
        </option>
        <option value="urdu" className={styles.languageOption}>
          {currentLanguage === 'urdu' ? '✓ ' : ''}اردو (Urdu)
        </option>
      </select>
      <span id="language-description" className={styles.visuallyHidden}>
        Changes the language for all chapters in the textbook. Current language: {currentLanguage}
      </span>
      {isTranslating && (
        <span className={styles.loadingIndicator} aria-live="polite">
          Translating...
        </span>
      )}
    </div>
  );
}
