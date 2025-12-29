# Feature Specification: Chapter-Level Urdu Translation

**Feature Branch**: `001-urdu-translation`
**Created**: 2025-12-29
**Status**: Draft
**Input**: User description: "Introduce a Chapter-level Urdu Translation Feature where participants can earn up to 50 extra bonus points if they enable Urdu translation by pressing a button at the start of a chapter. Translation must be accurate, readable, preserve formatting, and be future-expandable for more languages."

## Clarifications

### Session 2025-12-29

- Q: Authentication System - how should users be authenticated? → A: localStorage mock authentication (username only, no passwords) - Fast MVP to validate UX first, can upgrade to production auth later
- Q: Translation Provider - which translation API should we use? → A: Google Gemini API (reuse existing RAG backend infrastructure and API keys)
- Q: Bonus Points Storage - where/how should bonus points be tracked? → A: localStorage (client-side, consistent with auth approach, acceptable for educational MVP)
- Q: Translation Caching Strategy - should we cache translations? → A: localStorage cache with 7-day TTL (balance between freshness and API cost)
- Q: Rate Limiting - limit translation requests per user? → A: Simple client-side throttling (1 request per 2 seconds, prevent rapid clicking)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Logged-in User Translates Chapter to Urdu (Priority: P1)

A logged-in student reads a chapter in English, presses the Urdu translation button at the start of the chapter, and the entire chapter content (including headings, bold text, code blocks) translates dynamically into Urdu. The system awards 50 bonus points for this action (once per chapter).

**Why this priority**: This is the core functionality that delivers immediate value to Urdu-speaking students and provides the bonus incentive mechanism.

**Independent Test**: Can be fully tested by logging in, navigating to any chapter, clicking the translation button, and verifying Urdu content appears with preserved formatting and bonus points are awarded.

**Acceptance Scenarios**:

1. **Given** a logged-in user viewing a chapter in English, **When** they click the "Translate to Urdu" button, **Then** the chapter content translates to Urdu with all formatting preserved
2. **Given** a logged-in user translates a chapter for the first time, **When** translation completes, **Then** system awards 50 bonus points and logs the action
3. **Given** a logged-in user has already translated a chapter, **When** they translate the same chapter again, **Then** translation works but no additional bonus points are awarded

---

### User Story 2 - Anonymous User Cannot See Translation Button (Priority: P2)

An anonymous (not logged-in) user views a chapter and does not see any translation button, ensuring the bonus points feature is only available to authenticated users.

**Why this priority**: Essential for access control and preventing abuse, but secondary to core translation functionality.

**Independent Test**: Can be tested by opening the site in incognito mode, navigating to any chapter, and verifying no translation button appears.

**Acceptance Scenarios**:

1. **Given** an anonymous user viewing a chapter, **When** the page loads, **Then** no translation button is visible
2. **Given** an anonymous user, **When** they log in from a chapter page, **Then** the translation button appears dynamically

---

### User Story 3 - Toggle Back to Original Language (Priority: P3)

A logged-in user who has translated a chapter to Urdu can toggle back to the original English version without losing their place or requiring page reload.

**Why this priority**: Enhances user experience but not critical for MVP. Users can achieve similar result by refreshing the page.

**Independent Test**: Can be tested by translating a chapter to Urdu, then clicking "Show Original" button, and verifying English content reappears.

**Acceptance Scenarios**:

1. **Given** a chapter is displayed in Urdu, **When** user clicks "Show Original English", **Then** content switches back to English without page reload
2. **Given** a chapter is toggled between languages multiple times, **When** user toggles, **Then** scroll position and reading progress are preserved

---

### Edge Cases

- What happens when translation API fails or times out?
- How does system handle chapters with mixed content (text + images + code blocks + LaTeX)?
- What happens if user rapidly clicks translation button multiple times (rate limiting)?
- How does system handle very long chapters (>10,000 words)?
- What happens when user's bonus points quota is reached (if there's a maximum)?
- How does system handle partial translations when API returns incomplete results?
- What happens when user loses internet connection during translation?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a "Translate to Urdu" button at the start of each chapter ONLY for logged-in users
- **FR-002**: System MUST translate entire chapter content to Urdu when button is clicked, preserving all formatting (headings, bold, italics, code blocks, lists, tables)
- **FR-003**: System MUST award 50 bonus points to logged-in user on FIRST translation of each chapter
- **FR-004**: System MUST prevent awarding bonus points for subsequent translations of the same chapter by the same user
- **FR-005**: System MUST log translation actions with timestamp, user ID, chapter ID, and bonus points awarded
- **FR-006**: System MUST handle translation failures gracefully and show user-friendly error messages
- **FR-007**: System MUST preserve code blocks untranslated (only comments within code may be translated if technically feasible)
- **FR-008**: System MUST provide visual feedback during translation (loading indicator)
- **FR-009**: System MUST be architected to support future addition of other languages (Arabic, Hindi, etc.)
- **FR-010**: System MUST use localStorage-based mock authentication for MVP (username stored client-side, no password validation, simple login/logout UI)
- **FR-011**: System MUST use Google Gemini API for translation (leveraging existing RAG backend at `https://ubaid-ai-rag-chatbot.hf.space` or `http://localhost:8000`)
- **FR-012**: System MUST store bonus points in localStorage with structure: `{userId: {totalPoints: number, translationHistory: [{chapterId, timestamp, pointsAwarded}]}}`
- **FR-013**: System MUST cache translated content in localStorage with 7-day TTL to reduce API costs and improve performance
- **FR-014**: System MUST implement client-side rate limiting (minimum 2-second gap between translation requests) to prevent API abuse

### Key Entities

- **User**: Authenticated participant with username (stored in localStorage key `currentUser`), bonus points balance, translation history
- **Chapter**: Document content with unique ID derived from URL path (e.g., `/introduction` → `introduction`), original markdown content
- **TranslationAction**: Event record stored in localStorage tracking userId, chapterId, timestamp, language, pointsAwarded
- **BonusPoints**: localStorage structure `userBonusPoints` containing `{userId: {totalPoints: number, translationHistory: Array}}`

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Logged-in users can successfully translate any chapter to Urdu in under 10 seconds for chapters <5000 words
- **SC-002**: Translation preserves 100% of formatting elements (headings, bold, code blocks verified via automated tests)
- **SC-003**: Bonus points are awarded correctly with 0% false positives (no duplicate awards for same chapter)
- **SC-004**: System handles translation failures gracefully with <1 second error response time
- **SC-005**: Anonymous users cannot access translation feature (0% unauthorized access rate)
- **SC-006**: Translation accuracy meets minimum 85% semantic similarity score (measured via automated quality checks)
- **SC-007**: System can be extended to add a new language in <4 hours of developer time (architecture extensibility metric)

## Non-Functional Requirements

### Performance
- Translation API response time: p95 < 8 seconds for typical chapter (3000 words)
- UI remains responsive during translation (non-blocking)
- Page load time impact: <200ms additional overhead for translation button rendering

### Security
- Authentication: localStorage-based (username only for MVP, no sensitive data stored)
- Bonus points manipulation: Client-side only for MVP (acceptable risk for educational prototype, note: upgrade to server-side validation for production)
- Translation API keys must be stored securely (environment variables, secret management)
- Note: Current MVP security model is suitable for educational/demo purposes only

### Reliability
- Translation service availability: 99% uptime expected
- Fallback behavior when translation service is down: show friendly error, allow retry
- Idempotency: multiple translation requests for same chapter should not cause duplicate point awards

### Cost Management
- Google Gemini API free tier: 15 requests/minute, 1500 requests/day (sufficient for MVP with <100 active users)
- Estimated cost for production: ~$0 for MVP usage, scales to ~$7/1M requests if exceeding free tier
- Consider caching translated chapters to reduce API calls (tradeoff: storage vs API cost)

### Observability
- Log all translation requests with user ID, chapter ID, language, timestamp
- Track translation success/failure rates
- Monitor API response times and error rates
- Track bonus points awarded per user and per chapter

## Out of Scope

- Real-time translation (on-the-fly as user scrolls)
- Translation of images or diagrams
- Translation of embedded videos or external content
- AI-generated alternative explanations in target language
- User-customizable translation preferences (formal vs informal tone)
- Translation history/comparison view
- Offline translation support

## Open Questions / Clarifications Needed

1. **Authentication System**: How are users authenticated? Do we need to build auth from scratch or integrate with existing system?
2. **Translation Provider**: Which translation API should we use? Budget constraints?
3. **Bonus Points Backend**: Where/how are bonus points stored and tracked? Database schema?
4. **Caching Strategy**: Should we cache translations? For how long? Storage limits?
5. **Error Recovery**: What should happen if translation partially completes?
6. **Rate Limiting**: Should we limit translation requests per user per time period?
7. **Translation Quality**: Do we need human review/validation of translations?
8. **Mobile Responsiveness**: Does translation button need special mobile UI treatment?
