# Vercel Serverless Deployment Guide

## Overview
This guide deploys the RAG chatbot backend as Vercel Serverless Functions.

## Important Notes

⚠️ **Vercel Serverless Limitations:**
- Cold start: 10-15 seconds on first request
- Execution timeout: 10 seconds (free tier)
- Memory: 1024 MB
- Not ideal for heavy AI workloads

✅ **Best for:**
- Testing and demos
- Low-traffic applications
- Quick prototypes

## Deployment Steps

### 1. Sign Up on Vercel (FREE)
1. Go to: https://vercel.com
2. Click "Sign Up"
3. Choose "Continue with GitHub"
4. Authorize Vercel

### 2. Create New Project
1. Click "Add New..." → "Project"
2. Import your GitHub repository:
   - `Ub207/physical-ai-and-huminiod-robotics`
3. Click "Import"

### 3. Configure Project
**Framework Preset:** Other
**Root Directory:** `rag-chatbot` ⚠️ IMPORTANT!
**Build Command:** Leave empty
**Output Directory:** Leave empty

### 4. Add Environment Variables
Click "Environment Variables" and add:

```
COHERE_API_KEY = your_cohere_key
QDRANT_URL = your_qdrant_url
QDRANT_API_KEY = your_qdrant_key
NEON_DATABASE_URL = your_neon_url
AI_PROVIDER = cohere
QDRANT_COLLECTION_NAME = book_chunks
```

### 5. Deploy
- Click "Deploy"
- Wait 2-3 minutes
- Your API will be at: `https://your-project.vercel.app`

### 6. Test Endpoints
```bash
# Health check
curl https://your-project.vercel.app/health

# Query test
curl -X POST https://your-project.vercel.app/query \
  -H "Content-Type: application/json" \
  -d '{"book_id": "physical_ai_textbook", "query": "What is Physical AI?"}'
```

## Expected Behavior

**First Request (Cold Start):**
- Takes 10-15 seconds
- Initializes Python runtime
- Loads dependencies
- Connects to Qdrant/Cohere

**Subsequent Requests (Warm):**
- Takes 2-5 seconds
- Much faster response
- Container stays warm for ~5 minutes

## Performance Tips

1. **Keep Functions Warm:**
   - Use cron job to ping /health every 5 minutes
   - UptimeRobot.com (free) works well

2. **Optimize Cold Starts:**
   - Minimize dependencies in requirements.txt
   - Use lazy loading where possible

3. **Handle Timeouts:**
   - Vercel free tier: 10 second limit
   - If query takes longer, it will timeout
   - Consider caching responses

## Troubleshooting

### Issue: 504 Gateway Timeout
**Cause:** Query takes >10 seconds
**Solution:**
- Use Pro plan ($20/month) for 60 second timeout
- Or implement response caching
- Or use different platform for backend

### Issue: Cold start too slow
**Cause:** Large dependencies
**Solution:**
- Implement /health endpoint pinging
- Consider splitting into microservices

### Issue: Out of memory
**Cause:** Large model loading
**Solution:**
- Reduce batch sizes
- Use streaming responses
- Upgrade to Pro ($20/month)

## Alternative: Vercel + External Backend

**Better Architecture:**
1. Frontend: Vercel (static site) ✅
2. Backend: Different platform (Railway/Render)
3. This avoids serverless limitations

## Cost

**Free Tier:**
- 100 GB bandwidth/month
- 100,000 invocations/month
- 10 second execution limit
- More than enough for testing!

**Pro Tier ($20/month):**
- 1 TB bandwidth
- Unlimited invocations
- 60 second execution limit
- Better for production

---

**Recommendation:** Use Vercel for frontend, deploy backend elsewhere for better performance.
