# Deployment Guide - Physical AI & Humanoid Robotics

## 🎯 Overview

This project deploys in two parts:
1. **Frontend (Docusaurus)** → Vercel
2. **Backend (RAG Chatbot)** → Render

---

## 🚀 Part 1: Deploy Frontend to Vercel

### Prerequisites
- GitHub account
- Vercel account (free tier is sufficient)

### Steps

#### 1. Connect to Vercel

**Option A: Via Vercel Dashboard (Recommended)**
1. Go to [vercel.com](https://vercel.com)
2. Click **"Add New Project"**
3. Click **"Import Git Repository"**
4. Select your GitHub repository: `physical-ai-and-huminiod-robotics`
5. Configure project:
   - **Framework Preset**: Other
   - **Root Directory**: `./` (leave as default)
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`
   - **Install Command**: `npm install`
6. Click **"Deploy"**

**Option B: Via Vercel CLI**
```bash
# Install Vercel CLI globally
npm install -g vercel

# Navigate to project root
cd "d:\physical-ai and huminiod-robotics"

# Deploy
vercel

# Follow prompts and select defaults
```

#### 2. Verify Frontend Deployment

After deployment completes (2-3 minutes):
- ✅ Visit the provided Vercel URL (e.g., `https://your-project.vercel.app`)
- ✅ Check homepage loads correctly
- ✅ Navigate to documentation pages
- ✅ Verify dark mode toggle works
- ✅ Check mobile responsiveness

#### 3. Note Your Vercel URL

Save your Vercel deployment URL - you'll need it for backend CORS configuration.

---

## 🐍 Part 2: Deploy Backend to Render

### Prerequisites
- Render account (free tier available at [render.com](https://render.com))
- GitHub repository access

### Steps

#### 1. Create Render Account
1. Go to [render.com](https://render.com)
2. Sign up with GitHub

#### 2. Create New Web Service

1. Click **"New +"** → **"Web Service"**
2. Connect your GitHub repository
3. Configure the service:
   - **Name**: `rag-chatbot-api` (or your preferred name)
   - **Region**: Choose closest to your users
   - **Branch**: `main` (or your default branch)
   - **Root Directory**: `rag-chatbot`
   - **Runtime**: Python 3
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn api.main:app --host 0.0.0.0 --port $PORT`

#### 3. Set Environment Variables

In Render dashboard, add these environment variables:

**Required:**
```bash
COHERE_API_KEY=<your-actual-cohere-key>
QDRANT_URL=<your-qdrant-url>
QDRANT_API_KEY=<your-actual-qdrant-key>
NEON_DATABASE_URL=<your-neon-database-url>
```

**Default values (already configured):**
```bash
COHERE_EMBED_MODEL=embed-english-v3.0
COHERE_GENERATION_MODEL=command-r-plus
QDRANT_COLLECTION_NAME=book_chunks
UPLOAD_FOLDER=/opt/render/project/src/uploads
MAX_CONTENT_LENGTH=16777216
ALLOWED_EXTENSIONS=pdf,epub,txt
```

> **Note**: These are stored in the `.env` file locally but must be added manually in Render's UI.

#### 4. Deploy

1. Click **"Create Web Service"**
2. Wait for deployment (5-10 minutes first time)
3. Monitor logs for any errors

#### 5. Verify Backend Deployment

Once deployed, test these endpoints:

```bash
# Health check
curl https://your-app.onrender.com/health

# Root endpoint
curl https://your-app.onrender.com/

# Should return: {"message": "RAG Chatbot API for Published Books"}
```

---

## 🔗 Part 3: Connect Frontend & Backend

### 1. Update Vercel Configuration

1. Go to your Vercel project dashboard
2. Navigate to **Settings** → **Environment Variables**
3. Add (optional, if needed by frontend):
   ```
   NEXT_PUBLIC_API_URL=https://your-app.onrender.com
   ```

### 2. Update vercel.json with Render URL

In `vercel.json`, replace `YOUR_RENDER_APP_NAME` with your actual Render app name:

```json
{
  "rewrites": [
    {
      "source": "/api/:path*",
      "destination": "https://your-actual-app-name.onrender.com/:path*"
    }
  ]
}
```

Then redeploy to Vercel:
```bash
# Commit changes
git add vercel.json
git commit -m "Update backend API URL"
git push

# Vercel will auto-deploy
```

### 3. Verify Integration

Test the integration:
1. Visit your Vercel frontend
2. Open browser DevTools (F12) → Console tab
3. Try using the RAG chatbot widget
4. Check for CORS or API errors

---

## ✅ Post-Deployment Checklist

### Security
- [ ] Verify `.env` is in `.gitignore` and not committed
- [ ] Confirm `.env.example` has placeholder values only
- [ ] Check CORS settings allow only your Vercel domain
- [ ] Review Render environment variables are set
- [ ] Enable HTTPS (automatic on both platforms)

### Functionality
- [ ] Frontend loads without errors
- [ ] All documentation pages accessible
- [ ] Backend `/health` endpoint returns 200
- [ ] RAG chatbot queries work
- [ ] No CORS errors in browser console

### Performance
- [ ] Frontend Lighthouse score > 90
- [ ] API response time < 3 seconds
- [ ] Images and assets load quickly

---

## 🔄 Continuous Deployment

Both platforms auto-deploy on git push:

- **Vercel**: Deploys automatically when you push to `main` branch
- **Render**: Auto-deploys when changes detected in `rag-chatbot/` directory

To trigger manual deployment:
- **Vercel**: Push to GitHub or use `vercel --prod`
- **Render**: Click "Manual Deploy" in dashboard

---

## 🐛 Troubleshooting

### Frontend Issues

**Build Fails**
```bash
# Test locally first
npm run build

# Check logs in Vercel dashboard
# Common fix: ensure all dependencies in package.json
```

**Pages Don't Load**
- Check `baseUrl` in `docusaurus.config.js` matches deployment URL
- Verify `outputDirectory` is set to `build` in `vercel.json`

### Backend Issues

**Deployment Fails**
- Check Python version is compatible (3.11.7)
- Verify all packages in `requirements.txt`
- Review Render logs for specific errors

**API Returns 500 Errors**
- Verify environment variables are set correctly in Render
- Check database connection string format
- Test Qdrant and Cohere credentials
- Review application logs in Render dashboard

**CORS Errors**
- Ensure your Vercel URL is in the CORS `allow_origins` list
- Check `api/main.py` CORS configuration
- Verify wildcards like `https://*.vercel.app` are included

**Timeout Errors**
- Render free tier instances spin down after 15 min of inactivity
- First request after idle period may take 30-60 seconds
- Consider upgrading to paid tier for always-on instances

---

## 💰 Cost Breakdown

| Service | Free Tier | Paid Option |
|---------|-----------|-------------|
| **Vercel** | Unlimited (Hobby) | $20/mo (Pro) |
| **Render** | 750 hrs/mo | $7/mo (Starter) |
| **Neon DB** | 0.5GB storage | $19/mo |
| **Qdrant Cloud** | 1GB cluster | $25/mo |

**Total for Free Tier**: $0/month (with limitations)
**Recommended Production**: ~$50-70/month

---

## 🔐 Security Best Practices

1. **Never commit `.env` files**
2. **Use environment variables** in deployment platforms
3. **Rotate API keys** regularly
4. **Enable 2FA** on all platforms
5. **Review CORS** settings before production
6. **Monitor logs** for suspicious activity
7. **Set up alerts** for deployment failures

---

## 📊 Monitoring

### Vercel Analytics
- Enable in Vercel dashboard → Analytics
- Track page views, performance, Web Vitals

### Render Metrics
- View in Render dashboard → Metrics
- Monitor CPU, memory, response times
- Set up alerts for downtime

### Optional: Add Sentry
```bash
# For error tracking
npm install @sentry/react
```

---

## 🆘 Support

**Vercel Support:**
- Docs: https://vercel.com/docs
- Community: https://github.com/vercel/vercel/discussions

**Render Support:**
- Docs: https://render.com/docs
- Community: https://community.render.com

**Project Issues:**
- GitHub: https://github.com/Ub207/physical-ai-and-huminiod-robotics/issues

---

## 🎉 Success!

If you've completed all steps:
- ✅ Frontend is live on Vercel
- ✅ Backend is running on Render
- ✅ RAG chatbot is functional
- ✅ All security measures in place

**Next Steps:**
1. Share your Vercel URL
2. Gather user feedback
3. Monitor performance metrics
4. Plan future enhancements

Happy deploying! 🚀
