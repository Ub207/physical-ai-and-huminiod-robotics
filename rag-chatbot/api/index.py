"""
Vercel Serverless adapter for FastAPI application.
This file wraps the FastAPI app for Vercel's serverless environment.
"""
from api.main import app

# Vercel requires a handler function
handler = app

# For local testing
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
