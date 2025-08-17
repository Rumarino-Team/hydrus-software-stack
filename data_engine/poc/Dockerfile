# SAM2 Data Engine Dockerfile
FROM pytorch/pytorch:2.7.1-cuda12.6-cudnn9-runtime

# Install system dependencies for GUI applications and git
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
    libxcb1 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-render0 \
    libxcb-shape0 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libxcb-cursor0 \
    x11-apps \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Clone SAM2 repository
RUN git clone https://github.com/facebookresearch/segment-anything-2.git /app/third_party/sam2

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies including SAM2
RUN pip install --no-cache-dir -r requirements.txt
RUN cd /app/third_party/sam2 && pip install -e .

# Create models directory and config directory
RUN mkdir -p /app/models /app/.config /app/sam2_models

# Set environment variables for Ultralytics
ENV YOLO_CONFIG_DIR=/app/.config
ENV YOLO_CONFIG_DIR=/app/.config

# Download SAM2 models during build time
RUN python3 -c "\
import sys; \
sys.path.append('/app/third_party/sam2'); \
import os; \
import urllib.request; \
os.makedirs('/app/sam2_models', exist_ok=True); \
checkpoint_urls = { \
    'sam2_hiera_tiny.pt': 'https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_tiny.pt', \
    'sam2_hiera_small.pt': 'https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_small.pt', \
    'sam2_hiera_base_plus.pt': 'https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_base_plus.pt', \
    'sam2_hiera_large.pt': 'https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_large.pt' \
}; \
[urllib.request.urlretrieve(url, f'/app/sam2_models/{filename}') for filename, url in checkpoint_urls.items() if not os.path.exists(f'/app/sam2_models/{filename}')]; \
print('All SAM2 models downloaded')"

# Download YOLOv8 models during build time (keeping YOLO for dataset export)
RUN python -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt'); print('YOLOv8 nano model downloaded')"
RUN python -c "from ultralytics import YOLO; model = YOLO('yolov8s.pt'); print('YOLOv8 small model downloaded')"
RUN python -c "from ultralytics import YOLO; model = YOLO('yolov8m.pt'); print('YOLOv8 medium model downloaded')"

# Copy the current directory contents
COPY . .

# Set environment variables for GUI
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

# Run the main GUI application
CMD ["python", "main.py"]
