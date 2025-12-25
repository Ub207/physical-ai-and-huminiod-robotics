# RAG Chatbot Constitution

## Core Principles

### I. Content-First Architecture
The RAG chatbot exists to serve book content with accuracy and reliability:
- Book content is the single source of truth
- All answers must be grounded in indexed content
- No hallucinations - if content doesn't exist, admit it
- Source attribution is mandatory for every answer

### II. Vector Search Quality
Embedding and retrieval quality directly impacts user experience:
- Embeddings must be 1024-dimensional for optimal accuracy
- Chunk size: 500 characters with 50 character overlap for context
- Minimum 5 sources retrieved per query
- Confidence threshold: 60% minimum for reliable answers
- Re-indexing required when content changes

### III. API-First Design
Backend API is the foundation for all interfaces:
- FastAPI with automatic OpenAPI documentation
- RESTful endpoints with consistent JSON responses
- CORS enabled for cross-origin frontend integration
- Health checks and monitoring endpoints required
- Response time target: <800ms for 95th percentile

### IV. Multi-Provider Support
Never lock into single AI provider:
- Support both Cohere and Google Gemini
- Provider selection via environment variable
- Graceful fallback between models
- Consistent interface regardless of provider
- Monitor usage and costs across providers

### V. Environment-Based Configuration
All secrets and settings via environment variables:
- Never hardcode API keys or credentials
- .env file for local development
- Platform secrets for production deployment
- Validation on startup - fail fast if misconfigured
- .env.example maintained as template

### VI. Database Separation
Vector and relational databases serve different purposes:
- Qdrant for vector embeddings and similarity search
- Neon PostgreSQL for metadata and conversation history
- Each database connection validated independently
- Graceful degradation if one service unavailable
- Connection pooling for efficiency

### VII. Testing Before Deployment
Quality gates prevent broken deployments:
- Environment validation tests required
- Vector store connectivity tests required
- Sample query tests with known good answers
- Performance benchmarks documented
- Confidence score validation (>60% threshold)

## Code Quality Standards

### Python Code Standards
- Type hints for all function signatures
- Docstrings for public functions and classes
- PEP 8 compliance (enforced by linting)
- Error handling with specific exception types
- Logging at appropriate levels (INFO, WARNING, ERROR)

### API Response Format
All API responses must include:
```json
{
  "answer": "Generated answer text",
  "sources": ["source1", "source2"],
  "confidence": 0.85,
  "metadata": {
    "query_time_ms": 432,
    "chunks_searched": 661
  }
}
```

### Error Handling
- Client errors (4xx): User input validation failures
- Server errors (5xx): System failures (database, API)
- Detailed error messages in development
- Generic error messages in production
- All errors logged with stack traces

## Performance Standards

### Query Performance
- Embedding generation: <200ms (p95)
- Vector search: <150ms (p95)
- Answer generation: <500ms (p95)
- Total query time: <800ms (p95)
- First query cold start acceptable: <2s

### Indexing Performance
- 100 chunks/minute minimum
- Progress reporting every 50 chunks
- Automatic retry on transient failures
- Validation after indexing complete
- Index creation: <5 minutes for typical book

### Resource Limits
- Maximum chunk size: 1000 characters
- Maximum query length: 500 characters
- Maximum sources returned: 10
- Qdrant collection limit: 10M vectors
- Database connection pool: 10 connections

## Security Requirements

### API Key Management
- API keys stored in environment variables only
- Keys rotated every 90 days
- Separate keys for development/production
- No API keys in logs or error messages
- Rate limiting to prevent abuse

### Data Privacy
- No user data stored without consent
- Conversation history optional
- Personal information redacted from logs
- GDPR compliance for EU users
- Data retention policy: 30 days default

### Network Security
- HTTPS only in production
- CORS restricted to known domains
- No sensitive data in URLs or query parameters
- Authentication required for admin endpoints
- DDoS protection via rate limiting

## Deployment Standards

### Platform Support
Primary platforms:
1. Render (production FastAPI hosting)
2. Hugging Face Spaces (public demos)
3. Railway (development/staging)

Requirements for all platforms:
- Dockerfile for containerized deployment
- Health check endpoint at /health
- Graceful shutdown handling
- Environment variable configuration
- Logging to stdout/stderr

### Deployment Checklist
Before deploying to production:
- [ ] All tests passing locally
- [ ] Environment variables documented
- [ ] API keys configured for production
- [ ] CORS origins updated for production domain
- [ ] Database connections tested
- [ ] Sample queries validated
- [ ] Performance benchmarks met
- [ ] Error tracking configured
- [ ] Monitoring dashboards created

### Rollback Plan
- Previous version tagged in git
- Database migrations reversible
- Feature flags for new functionality
- Rollback procedure documented
- Maximum downtime: 5 minutes

## Observability

### Logging Requirements
Log levels and content:
- **DEBUG**: Detailed execution flow (dev only)
- **INFO**: Query requests, responses, performance
- **WARNING**: Degraded performance, low confidence
- **ERROR**: API failures, connection errors
- **CRITICAL**: Service unavailable, data corruption

### Metrics to Track
- Query volume per hour/day
- Average response time
- Confidence score distribution
- Error rate by type
- API provider costs
- Cache hit rates

### Alerting Thresholds
Trigger alerts when:
- Error rate >5% over 5 minutes
- Average response time >1000ms
- Confidence scores <50% for >10% queries
- Database connection failures
- API rate limit warnings
- Disk space <20%

## Development Workflow

### Feature Development
1. Create spec in specs/rag-chatbot/spec.md
2. Design plan in specs/rag-chatbot/plan.md
3. Break into tasks in specs/rag-chatbot/tasks.md
4. Implement with TDD approach
5. Test locally with /rag.test
6. Document in COMMANDS.md if user-facing
7. Create PR with tests and documentation

### Code Review Requirements
All PRs must have:
- [ ] Tests covering new functionality
- [ ] Documentation updated
- [ ] No hardcoded secrets
- [ ] Error handling implemented
- [ ] Performance benchmarks met
- [ ] CLAUDE.md principles followed

### Quality Gates
Cannot merge without:
- All tests passing
- Code review approved
- No linting errors
- Documentation complete
- Performance validated

## Governance

### Constitution Authority
- This constitution supersedes all other practices
- All development decisions must align with principles
- Amendments require team consensus
- Version controlled in git
- Reviewed quarterly

### Amendment Process
To amend this constitution:
1. Propose change with rationale
2. Document impact on existing code
3. Get team approval
4. Update version number
5. Communicate to all team members
6. Create migration plan if needed

### Compliance
- All code reviews verify constitution compliance
- Automated checks where possible (linting, tests)
- Violations must be justified and documented
- Technical debt tracked and prioritized
- Regular audits of codebase

## Version History

**Version**: 1.0.0
**Ratified**: 2025-12-25
**Last Amended**: 2025-12-25

**Amendments**:
- 1.0.0 (2025-12-25): Initial constitution for RAG chatbot system
