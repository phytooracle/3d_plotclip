
# Use official Python 3.10 image as base
FROM python:3.10-slim

# Set working directory
WORKDIR /app
COPY . /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    libgl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --no-cache-dir \
    open3d \
    numpy \
    pyproj

# Set entrypoint
ENTRYPOINT ["python", "/app/3d_plot_clip.py"]
