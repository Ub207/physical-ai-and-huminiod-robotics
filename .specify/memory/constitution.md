# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Multilingual UX Ethics & Fairness
**Principle**: Language should never be a barrier to learning Physical AI and robotics. All language implementations must uphold dignity, accuracy, and equal access.

**Requirements**:
- **Language Parity**: Translations must preserve technical accuracy, formatting, and pedagogical intent across all languages
- **No Degradation**: Non-English modes must not compromise performance, readability, or feature availability
- **Cultural Sensitivity**: RTL (Right-to-Left) languages require proper text alignment, interface mirroring, and culturally appropriate UX patterns
- **Inclusive Design**: Language switcher must be discoverable, accessible, and require zero authentication barriers
- **Consistency**: Partial translations are prohibited; entire book must switch atomically or clearly indicate mixed-language state
- **Privacy**: Language preference storage must respect user privacy (no tracking, no analytics without consent)

### II. Accessibility & Universal Design
**Principle**: Educational content must be accessible to all learners regardless of language, ability, or device.

**Requirements**:
- **Keyboard Navigation**: All language controls must be keyboard-accessible (WCAG 2.1 AA minimum)
- **Screen Reader Support**: ARIA labels in user's selected language with proper semantic markup
- **Visual Clarity**: Minimum contrast ratios (4.5:1 text, 3:1 UI), distinguishable active states for language switcher
- **Mobile Responsive**: Language switcher must function on all screen sizes (320px+)
- **Low Bandwidth**: Translation caching required to minimize network requests in bandwidth-constrained environments
- **Offline Fallback**: Graceful degradation when translation API unavailable

### III. Code Quality & Maintainability
**Principle**: Production-grade, elegant, scalable solutions only. No hacky or temporary code.

**Requirements**:
- **Separation of Concerns**: Translation logic isolated from presentation layer
- **Testability**: All language-switching logic must be unit-testable
- **Type Safety**: TypeScript/PropTypes for all bilingual components
- **Error Boundaries**: React error boundaries to prevent UI crashes from translation failures
- **Performance Budget**: Language switch <200ms UI response, translation <10s for typical chapter
- **State Management**: Prefer React Context for global language state; avoid prop drilling
- **Cache Strategy**: Aggressive caching with TTL, shared cache across features (001 + 002)

### IV. Modern UI/UX Standards
**Principle**: Professional, polished, production-ready interfaces matching industry standards.

**Requirements**:
- **Design System**: Consistent spacing (8px grid), rounded corners (8px/12px), smooth transitions (200-300ms)
- **Color Palette**: NO GREEN theme; use Dark Blue (#1e3a8a) / Black (#111827) premium feel
- **Typography**: Clear hierarchy, readable font sizes (16px body minimum), line-height 1.6
- **Feedback**: Immediate visual feedback for all interactions (hover, active, loading states)
- **Loading States**: Skeleton screens or spinners for async operations, never blank screens
- **Error Handling**: User-friendly error messages with actionable recovery steps

### V. Architecture for Scale
**Principle**: Build for today's requirements with tomorrow's expansion in mind.

**Requirements**:
- **Extensibility**: Support 5+ languages without architectural changes (Arabic, Hindi, Spanish, French, Chinese)
- **Modularity**: Language switcher as standalone component, reusable across any Docusaurus site
- **API Design**: Translation API must accept `lang` parameter for future language expansion
- **Feature Flags**: Ability to enable/disable languages per deployment environment
- **Versioning**: Semantic versioning for translation API, backward compatibility guaranteed

### VI. Test-First Development (NON-NEGOTIABLE)
**Principle**: All bilingual features must have comprehensive test coverage before production deployment.

**Required Tests**:
- **Unit Tests**: Language context provider, localStorage persistence, cache logic
- **Integration Tests**: Language switcher + chapter translation flow, cache sharing between 001/002
- **E2E Tests**: Full user journey (anonymous user selects Urdu → navigates 5 chapters → switches back to English)
- **Accessibility Tests**: Automated axe-core checks for language switcher
- **Performance Tests**: Language switch <200ms, translation <10s benchmarks
- **Edge Cases**: localStorage full, API timeout, network offline, rapid language switching

### VII. Security & Privacy
**Principle**: User data and preferences must be protected at all times.

**Requirements**:
- **No PII in Translations**: User data must never be sent to translation API
- **Secure Storage**: localStorage keys namespaced to prevent collisions
- **API Security**: Translation API must validate input length, sanitize content
- **Content Integrity**: Translated content must preserve code blocks, links, and formatting without XSS vulnerabilities
- **Rate Limiting**: Client-side rate limiting to prevent API abuse

## Development Workflow

### Phase 1: Specification (CURRENT)
1. ✅ Create constitution with multilingual principles
2. Run `/sp.clarify` → Identify ambiguities, ask targeted questions (2-5 max)
3. Run `/sp.analyze` → Deep technical feasibility analysis
4. Refine `specs/002-bilingual-book-mode/spec.md` based on clarifications

### Phase 2: Planning
1. Run `/sp.plan` → Generate detailed architecture in `plan.md`
2. Review architectural decisions against constitution principles
3. If architecturally significant decisions detected → suggest `/sp.adr <title>`

### Phase 3: Task Breakdown
1. Run `/sp.tasks` → Generate actionable tasks in `tasks.md`
2. Each task must include:
   - Clear acceptance criteria
   - Independent test cases
   - Estimated complexity (S/M/L)
   - Dependencies (if any)

### Phase 4: Implementation
1. Run `/sp.implement` → Execute tasks from `tasks.md`
2. TDD cycle: Write tests → Implement → Verify
3. Create PHR after each implementation milestone
4. Zero tolerance for breaking changes to existing features

### Phase 5: Validation
1. Run all tests (unit, integration, E2E, accessibility)
2. Manual QA against success criteria
3. Performance benchmarks (language switch <200ms, translation <10s)
4. Cross-browser testing (Chrome, Firefox, Safari, Edge)

## Quality Gates

### Pre-Commit Checks
- ✅ All tests pass
- ✅ No TypeScript errors
- ✅ ESLint/Prettier formatting
- ✅ No console.log or debugger statements
- ✅ ARIA labels present on all interactive elements

### Pre-Merge Checks
- ✅ Code review approved by 1+ reviewer
- ✅ Constitution compliance verified
- ✅ Performance benchmarks met
- ✅ Accessibility audit passed (axe-core)
- ✅ Manual testing on mobile + desktop

### Pre-Production Checks
- ✅ Staging deployment successful
- ✅ E2E tests pass in staging environment
- ✅ Load testing completed (100 concurrent users)
- ✅ Rollback plan documented

## Non-Negotiable Constraints

### What We Will NOT Do
- ❌ Partial/incomplete translations (must be atomic site-wide switch)
- ❌ Green color theme (conflicts with branding)
- ❌ Temporary/hacky solutions (technical debt prohibited)
- ❌ Breaking existing 001-urdu-translation feature
- ❌ Gating language switcher behind authentication
- ❌ Storing language preference on server (localStorage only for now)
- ❌ Auto-translating upfront (on-demand per chapter only)
- ❌ Mixed-language mode (English + Urdu side-by-side in same chapter)

### Performance Budgets
- Language switch UI update: <200ms (hard limit)
- Chapter translation (cache miss): <10s (p95)
- Chapter translation (cache hit): <50ms (hard limit)
- Navigation to new chapter (Urdu mode): <500ms + translation time
- Translation cache size: <5MB per language per 40 chapters
- Memory footprint: <50MB additional for language context

## Success Definition

This feature is complete when:
1. ✅ Any user (logged-in or anonymous) can switch language from header in <200ms
2. ✅ Entire book displays consistently in selected language across all chapters
3. ✅ Language preference persists across browser sessions (localStorage)
4. ✅ RTL support works correctly for Urdu (text alignment, interface mirroring)
5. ✅ Zero conflicts with 001-urdu-translation (both features coexist peacefully)
6. ✅ All tests pass (unit, integration, E2E, accessibility)
7. ✅ Performance budgets met (<200ms UI, <10s translation)
8. ✅ Production deployment successful on Vercel
9. ✅ Mobile responsive on all devices (320px+ screens)
10. ✅ Architecture supports 5+ languages with zero breaking changes

## Governance

- **Constitution Authority**: This document supersedes all other project practices and coding standards
- **Amendments**: Any changes to constitution require documentation in `history/adr/` with rationale
- **Compliance**: All pull requests must explicitly verify alignment with constitution principles
- **PHR Mandatory**: Prompt History Records required for all specification, planning, and implementation work
- **ADR Suggestions**: Architectural decisions must be surfaced to user for documentation approval
- **Human-as-Tool**: Agent must invoke user for clarification when ambiguous, not assume or invent solutions

**Version**: 2.0.0
**Ratified**: 2025-12-29
**Last Amended**: 2025-12-29
**Feature Context**: 002-bilingual-book-mode (Global Language Switcher)
