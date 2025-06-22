# SAM2 Data Engine Dockerfile
FROM pytorch/pytorch:2.7.1-cuda12.6-cudnn9-runtime

# Install system dependencies for GUI applications
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libxrender1 \
    libxrandr2 \
    libxss1 \
    libxcursor1 \
    libxcomposite1 \
    libasound2 \
    libxi6 \
    libxtst6 \
    libqt5gui5 \
    libqt5core5a \
    libqt5widgets5 \
    qt5-gtk-platformtheme \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the current directory contents
COPY . .

# Set environment variables for GUI
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

# Run the main GUI application
CMD ["python", "main.py"]
