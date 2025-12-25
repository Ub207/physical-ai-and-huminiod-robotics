import gradio as gr
import os
from api.rag_service import RAGService
from models.schemas import QueryRequest
import asyncio
import json

# Initialize the RAG service
rag_service = RAGService()

def query_backend(book_id: str, query: str):
    """Function to handle queries to the RAG backend"""
    try:
        # Create a query request object
        request = QueryRequest(book_id=book_id, query=query)

        # Since rag_service.query is async, we need to run it in an event loop
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        # Run the async query method
        result = loop.run_until_complete(rag_service.query(request))

        # Return the answer and sources
        return result.answer, json.dumps(result.sources, indent=2)
    except Exception as e:
        return f"Error processing query: {str(e)}", "[]"

# Create Gradio interface
with gr.Blocks(title="RAG Chatbot Backend", theme=gr.themes.Soft()) as demo:
    gr.Markdown("# RAG Chatbot Backend API")
    gr.Markdown("Backend API for the RAG Chatbot system")

    with gr.Row():
        with gr.Column():
            book_id_input = gr.Textbox(label="Book ID", value="physical_ai_textbook")
            query_input = gr.Textbox(label="Query", lines=3)
            submit_btn = gr.Button("Submit Query", variant="primary")

        with gr.Column():
            answer_output = gr.Textbox(label="Answer", interactive=False, lines=5)
            sources_output = gr.Textbox(label="Sources (JSON)", interactive=False, lines=8)

    submit_btn.click(
        fn=query_backend,
        inputs=[book_id_input, query_input],
        outputs=[answer_output, sources_output]
    )

    gr.Markdown("## API Endpoints")
    gr.Markdown("- `/query` - Query the RAG system")
    gr.Markdown("- `/ingest` - Ingest books into the system")
    gr.Markdown("- `/health` - Health check endpoint")

# For Hugging Face Spaces, we run with gradio
if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=int(os.environ.get("PORT", 7860)))