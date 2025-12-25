# Use the official Python image
FROM python:3.11.7

# Set the working directory
WORKDIR /app

# Copy the requirements file
COPY requirements.txt .

# Install the dependencies
RUN pip install -r requirements.txt

# Copy the rest of the application
COPY . .

# Expose the port
EXPOSE 7860

# Start the application
CMD ["python", "app.py"]
