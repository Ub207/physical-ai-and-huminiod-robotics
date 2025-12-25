import os
from api.main import app

# This allows Hugging Face Spaces to run the FastAPI app
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 7860)))