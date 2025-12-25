# PythonAnywhere Deployment Guide

## Overview
Deploy RAG chatbot backend on PythonAnywhere - Free Python hosting!

## Features
✅ **100% FREE** - No credit card needed
✅ **Always-on** - No cold starts
✅ **Python 3.11** support
✅ **512MB storage** free tier
✅ **Custom domain** support
✅ **Perfect for FastAPI/Flask**

## Deployment Steps

### Step 1: Sign Up (FREE)
1. Go to: **https://www.pythonanywhere.com/registration/register/beginner/**
2. Fill in details:
   - Username: `ubaidai` (or your choice)
   - Email: your email
   - Password: create password
3. Click **"Register"**
4. Confirm email

### Step 2: Open Bash Console
1. Login to PythonAnywhere
2. Go to **"Consoles"** tab
3. Click **"Bash"** to open terminal

### Step 3: Clone Repository
In the Bash console:
```bash
git clone https://github.com/Ub207/physical-ai-and-huminiod-robotics.git
cd physical-ai-and-huminiod-robotics/rag-chatbot
```

### Step 4: Create Virtual Environment
```bash
python3.11 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### Step 5: Create WSGI File
Go to **"Web"** tab → **"Add a new web app"**
- Domain: `ubaidai.pythonanywhere.com`
- Python version: **3.11**
- Framework: **Manual configuration**

Edit WSGI file (`/var/www/ubaidai_pythonanywhere_com_wsgi.py`):

```python
import sys
import os

# Add project directory
project_home = '/home/ubaidai/physical-ai-and-huminiod-robotics/rag-chatbot'
if project_home not in sys.path:
    sys.path.insert(0, project_home)

# Set environment variables
os.environ['COHERE_API_KEY'] = 'your_key_here'
os.environ['QDRANT_URL'] = 'your_url_here'
os.environ['QDRANT_API_KEY'] = 'your_key_here'
os.environ['NEON_DATABASE_URL'] = 'your_db_url_here'
os.environ['AI_PROVIDER'] = 'cohere'
os.environ['QDRANT_COLLECTION_NAME'] = 'book_chunks'

# Import FastAPI app
from api.main import app as application
```

### Step 6: Configure Virtualenv
In Web tab:
- **Virtualenv path**: `/home/ubaidai/physical-ai-and-huminiod-robotics/rag-chatbot/venv`
- Click **"Reload"** button

### Step 7: Your API URL
```
https://ubaidai.pythonanywhere.com
```

Test endpoints:
```bash
curl https://ubaidai.pythonanywhere.com/health
```

## Environment Variables

Edit WSGI file to add your secrets:
```python
os.environ['COHERE_API_KEY'] = 'sk-...'
os.environ['QDRANT_URL'] = 'https://...'
os.environ['QDRANT_API_KEY'] = 'qdrant_...'
os.environ['NEON_DATABASE_URL'] = 'postgresql://...'
```

## Update Code
When you update GitHub repo:
```bash
cd ~/physical-ai-and-huminiod-robotics/rag-chatbot
git pull origin main
source venv/bin/activate
pip install -r requirements.txt
```
Then click **"Reload"** in Web tab

## Limitations (Free Tier)

❌ **CPU seconds**: 100/day (resets daily)
❌ **Always-on tasks**: Not in free tier
❌ **Outbound internet**: Limited sites only
✅ **BUT**: Perfect for API hosting!

## Performance

- **No cold starts** (always running)
- **Fast response** (< 1 second)
- **Stable uptime** (99%+)
- **Great for demos**

## Cost

**Free Tier:**
- 512MB disk space
- 1 web app
- Python 3.11
- More than enough!

**Paid ($5/month):**
- More CPU time
- Always-on tasks
- More storage

## Troubleshooting

### Issue: Import errors
**Solution:**
```bash
source venv/bin/activate
pip install --upgrade -r requirements.txt
```
Reload web app

### Issue: Environment variables not working
**Solution:** Check WSGI file has correct os.environ lines

### Issue: 502 Bad Gateway
**Solution:** Check error log in Web tab

---

**Your Backend URL:**
```
https://ubaidai.pythonanywhere.com
```

Replace `ubaidai` with your chosen username!
