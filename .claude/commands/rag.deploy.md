---
description: Deploy the RAG chatbot backend to cloud platforms (Render, Hugging Face, etc.)
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

This command guides deployment of the RAG chatbot backend to various cloud platforms.

1. **Pre-deployment Checklist**:
   - [ ] Local testing completed successfully
   - [ ] All environment variables documented
   - [ ] Requirements.txt is up to date
   - [ ] Dockerfile exists and tested (if using Docker)
   - [ ] Production API keys obtained
   - [ ] Database connections configured for production
   - [ ] CORS settings configured for frontend domain

2. **Choose Deployment Platform**:

   Ask user which platform to deploy to:
   - **Render** (recommended for FastAPI)
   - **Hugging Face Spaces** (good for demos)
   - **Railway** (easy deployment)
   - **Fly.io** (edge computing)
   - **AWS/GCP/Azure** (enterprise)

3. **Platform-Specific Deployment**:

   ### Option A: Render Deployment

   a. **Create render.yaml**:
   ```yaml
   services:
     - type: web
       name: rag-chatbot-backend
       env: python
       region: oregon
       plan: free
       buildCommand: pip install -r requirements.txt
       startCommand: uvicorn api.main:app --host 0.0.0.0 --port $PORT
       envVars:
         - key: PYTHON_VERSION
           value: 3.11.5
         - key: AI_PROVIDER
           sync: false
         - key: COHERE_API_KEY
           sync: false
         - key: QDRANT_URL
           sync: false
         - key: QDRANT_API_KEY
           sync: false
         - key: NEON_DATABASE_URL
           sync: false
   ```

   b. **Deploy to Render**:
   ```bash
   # Option 1: GitHub integration
   # - Connect repo to Render
   # - Render auto-deploys on push

   # Option 2: Manual deployment
   # - Create new Web Service
   # - Connect GitHub repo
   # - Add environment variables
   # - Deploy
   ```

   ### Option B: Hugging Face Spaces

   a. **Create Dockerfile.hf**:
   ```dockerfile
   FROM python:3.11-slim

   WORKDIR /app

   COPY requirements.txt .
   RUN pip install --no-cache-dir -r requirements.txt

   COPY . .

   EXPOSE 7860

   CMD ["uvicorn", "api.main:app", "--host", "0.0.0.0", "--port", "7860"]
   ```

   b. **Create huggingface.yml**:
   ```yaml
   title: Physical AI RAG Chatbot
   emoji: 🤖
   colorFrom: purple
   colorTo: yellow
   sdk: docker
   dockerFile: Dockerfile.hf
   pinned: false
   license: mit
   ```

   c. **Deploy to Hugging Face**:
   ```bash
   # Install Hugging Face CLI
   pip install huggingface_hub

   # Login
   huggingface-cli login

   # Create Space
   huggingface-cli repo create rag-chatbot --type space --space_sdk docker

   # Push code
   git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/rag-chatbot
   git push hf main
   ```

   ### Option C: Railway Deployment

   a. **Create railway.json**:
   ```json
   {
     "$schema": "https://railway.app/railway.schema.json",
     "build": {
       "builder": "NIXPACKS"
     },
     "deploy": {
       "startCommand": "uvicorn api.main:app --host 0.0.0.0 --port $PORT",
       "restartPolicyType": "ON_FAILURE",
       "restartPolicyMaxRetries": 10
     }
   }
   ```

   b. **Deploy to Railway**:
   ```bash
   # Install Railway CLI
   npm install -g @railway/cli

   # Login
   railway login

   # Initialize
   railway init

   # Add environment variables
   railway variables set AI_PROVIDER=cohere
   railway variables set COHERE_API_KEY=your_key

   # Deploy
   railway up
   ```

4. **Configure Environment Variables**:

   For the selected platform, add these variables:
   ```
   Required:
   - AI_PROVIDER=cohere
   - COHERE_API_KEY=xxx
   - COHERE_EMBED_MODEL=embed-english-v3.0
   - COHERE_GENERATION_MODEL=command-r-plus
   - QDRANT_URL=https://xxx.cloud.qdrant.io
   - QDRANT_API_KEY=xxx
   - QDRANT_COLLECTION_NAME=book_chunks
   - NEON_DATABASE_URL=postgresql://xxx

   Optional:
   - UPLOAD_FOLDER=uploads
   - MAX_CONTENT_LENGTH=16777216
   - ALLOWED_EXTENSIONS=pdf,epub,txt
   ```

5. **Update Frontend Configuration**:

   After deployment, update the frontend to use production URL:

   ```javascript
   // src/theme/Root.js
   const apiUrl = typeof window !== 'undefined'
     ? window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1'
       ? 'http://localhost:8001'
       : 'https://your-deployed-backend.onrender.com'  // UPDATE THIS
     : 'http://localhost:8001';
   ```

6. **Verify Deployment**:

   a. **Test Health Endpoint**:
   ```bash
   curl https://your-backend-url.com/health
   ```

   b. **Test Query Endpoint**:
   ```bash
   curl -X POST https://your-backend-url.com/query \
     -H "Content-Type: application/json" \
     -d '{
       "book_id": "physical_ai_textbook",
       "query": "What is Physical AI?"
     }'
   ```

   c. **Check API Documentation**:
   ```
   https://your-backend-url.com/docs
   ```

7. **Post-Deployment Tasks**:
   - [ ] Test all API endpoints from production
   - [ ] Verify frontend can connect to backend
   - [ ] Check CORS settings allow frontend domain
   - [ ] Monitor logs for errors
   - [ ] Set up health check monitoring
   - [ ] Configure custom domain (optional)
   - [ ] Enable HTTPS (usually automatic)
   - [ ] Set up error tracking (Sentry, etc.)

## Deployment URLs

After deployment, you'll receive:
- **Backend API**: `https://your-app.platform.com`
- **API Docs**: `https://your-app.platform.com/docs`
- **Health Check**: `https://your-app.platform.com/health`

## Usage Examples

```bash
# Deploy to Render
/rag.deploy render

# Deploy to Hugging Face
/rag.deploy huggingface

# Deploy to Railway
/rag.deploy railway

# Update deployment
/rag.deploy --update
```

## Platform Comparison

| Platform | Free Tier | Deployment | Best For |
|----------|-----------|------------|----------|
| Render | 750 hrs/month | GitHub auto-deploy | Production APIs |
| Hugging Face | Yes | Git push | AI/ML demos |
| Railway | $5 credit | CLI or GitHub | Quick prototypes |
| Fly.io | Yes | CLI deploy | Edge computing |

## CORS Configuration

Ensure CORS is configured in `api/main.py`:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://your-frontend-domain.com",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Troubleshooting

### Build Failures
- Check Python version in platform settings
- Verify requirements.txt is complete
- Check build logs for specific errors

### Runtime Errors
- Verify all environment variables are set
- Check database connection strings
- Review application logs

### Connection Timeouts
- Increase timeout settings
- Check Qdrant API limits
- Verify network connectivity

### CORS Errors
- Add frontend domain to allowed origins
- Check if credentials are needed
- Verify HTTP/HTTPS protocol matches

## Monitoring and Logs

After deployment, monitor:
- **Request logs**: Track API usage
- **Error logs**: Identify issues
- **Performance**: Response times, memory usage
- **Database**: Connection pool, query times
- **External APIs**: Cohere/Qdrant usage and limits

## Security Considerations

- Use environment variables for secrets (never hardcode)
- Enable HTTPS only in production
- Implement rate limiting
- Add API authentication if needed
- Regularly rotate API keys
- Monitor for suspicious activity

## Cost Estimates

Free tiers should handle:
- ~10-50 requests/day
- Small to medium traffic
- Development and testing

For production:
- Render Starter: $7/month
- Railway: Pay per usage ($0.000463/GB-hour)
- Hugging Face: Free for public spaces

## Notes

- First deploy may take 5-10 minutes
- Free tiers may have cold start delays (30-60s)
- Production requires paid plans for better performance
- Keep deployment URLs secret if not public
- Update frontend after backend deployment
