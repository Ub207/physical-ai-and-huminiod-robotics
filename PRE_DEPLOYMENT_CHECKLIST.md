# Pre-Deployment Checklist

## ✅ Completed Setup

- [x] **Updated `vercel.json`** - Added build commands, security headers, and API proxy
- [x] **Created `.vercelignore`** - Excludes backend and unnecessary files from Vercel
- [x] **Created `render.yaml`** - Automated Render deployment configuration
- [x] **Created backend `.gitignore`** - Prevents committing sensitive files
- [x] **Secured `.env.example`** - Removed real API keys, added placeholders
- [x] **Updated CORS settings** - Configured for Vercel + localhost in `api/main.py`
- [x] **Created `DEPLOYMENT.md`** - Comprehensive step-by-step deployment guide
- [x] **Verified local build** - `npm run build` succeeds ✅

---

## 📋 Pre-Deployment Checks

Before deploying, verify:

### Code Quality
- [ ] All changes committed to Git
- [ ] `.env` file is NOT committed (check with `git status`)
- [ ] `.env.example` contains only placeholder values
- [ ] No console.log or debug code in production files

### Dependencies
- [ ] All dependencies listed in `package.json` (frontend)
- [ ] All dependencies listed in `requirements.txt` (backend)
- [ ] No security vulnerabilities (`npm audit`)

### Environment Variables
You'll need these for Render deployment:
- [ ] Cohere API key ready
- [ ] Qdrant URL and API key ready
- [ ] Neon database URL ready
- [ ] All values are correct and tested

---

## 🚀 Deployment Steps

### Step 1: Deploy Frontend to Vercel

1. **Push to GitHub**
   ```bash
   git add .
   git commit -m "Configure for Vercel and Render deployment"
   git push origin main
   ```

2. **Deploy to Vercel**
   - Option A: Via dashboard at [vercel.com](https://vercel.com)
   - Option B: Via CLI: `vercel --prod`

3. **Verify deployment**
   - [ ] Site loads at Vercel URL
   - [ ] Documentation pages work
   - [ ] No 404 errors

### Step 2: Deploy Backend to Render

1. **Create Render account** at [render.com](https://render.com)

2. **Create Web Service**
   - Connect GitHub repository
   - Root directory: `rag-chatbot`
   - Runtime: Python 3
   - Build: `pip install -r requirements.txt`
   - Start: `uvicorn api.main:app --host 0.0.0.0 --port $PORT`

3. **Set environment variables** (CRITICAL)
   ```
   COHERE_API_KEY=<your-actual-key>
   QDRANT_URL=<your-qdrant-url>
   QDRANT_API_KEY=<your-actual-key>
   NEON_DATABASE_URL=<your-neon-url>
   ```

4. **Deploy and wait** (5-10 minutes)

### Step 3: Connect Frontend & Backend

1. **Get your Render URL** (e.g., `https://rag-chatbot-api.onrender.com`)

2. **Update `vercel.json`**
   - Replace `YOUR_RENDER_APP_NAME` with your actual Render app name
   - Commit and push to redeploy Vercel

3. **Test integration**
   - [ ] Frontend can call backend API
   - [ ] No CORS errors

---

## ✅ Post-Deployment Verification

### Frontend (Vercel)
- [ ] `https://your-app.vercel.app` loads
- [ ] All pages accessible
- [ ] Dark mode works
- [ ] Mobile responsive
- [ ] Lighthouse score > 90

### Backend (Render)
- [ ] `https://your-app.onrender.com/health` returns 200
- [ ] `https://your-app.onrender.com/` returns API info
- [ ] Database connection works
- [ ] Qdrant connection works

### Integration
- [ ] RAG chatbot widget loads on frontend
- [ ] Can submit queries
- [ ] Receives responses
- [ ] No console errors

---

## 🔒 Security Checklist

- [ ] `.env` is in `.gitignore`
- [ ] No API keys in repository
- [ ] CORS only allows Vercel domain
- [ ] HTTPS enabled (automatic)
- [ ] Environment variables set in Render UI

---

## 🐛 Common Issues

### Vercel Build Fails
```bash
# Test locally first
npm run build

# Check which dependencies are missing
npm install
```

### Render Deployment Fails
- Check Python version (3.11.7)
- Verify `requirements.txt` is complete
- Review build logs in Render dashboard

### CORS Errors
- Verify Vercel URL is in `api/main.py` CORS origins
- Check browser DevTools Console for exact error

### Backend Timeout
- Render free tier spins down after 15 min
- First request may take 30-60s
- Consider upgrading to paid tier

---

## 📊 Expected URLs

After deployment, you should have:

- **Frontend**: `https://physical-ai-and-huminiod-robotics-c82y.vercel.app`
- **Backend**: `https://your-chosen-name.onrender.com`
- **API Proxy**: `https://your-app.vercel.app/api/*` → Backend

---

## 🎯 Success Criteria

Your deployment is successful when:
- ✅ Frontend loads without errors
- ✅ All documentation is accessible
- ✅ Backend health check responds
- ✅ RAG chatbot accepts and processes queries
- ✅ No security warnings or CORS errors
- ✅ Performance is acceptable (< 3s load time)

---

## 📞 Need Help?

Refer to:
- **Deployment guide**: `DEPLOYMENT.md`
- **Vercel docs**: https://vercel.com/docs
- **Render docs**: https://render.com/docs
- **Troubleshooting section** in DEPLOYMENT.md

---

## 🎉 Ready to Deploy!

All configuration files are in place. Follow the steps above to deploy safely.

**Estimated deployment time**: 15-20 minutes total

Good luck! 🚀
