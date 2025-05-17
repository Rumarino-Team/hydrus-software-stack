#!/usr/bin/env python3
"""
Detection Viewer - A Flask web application to display camera feed with detection bounding boxes

This app connects to the FastAPI backend to receive video frames and detection data,
then draws bounding boxes on the frames and displays them in a web interface.
"""

import os
import cv2
import numpy as np
import requests
import json
from flask import Flask, render_template, Response
import threading
import time
import logging

app = Flask(__name__)
app.logger.setLevel(logging.INFO)

# Configuration
API_BASE_URL = "http://localhost:8000"  # Your FastAPI server
REFRESH_RATE = 0.1  # seconds

# Global variables to store the latest frame and detections
current_frame = None
current_detections = []
update_lock = threading.Lock()

class DetectionUpdater:
    def __init__(self):
        self.running = True
        self.thread = threading.Thread(target=self.update_loop)
        self.thread.daemon = True
    
    def start(self):
        self.thread.start()
    
    def update_loop(self):
        while self.running:
            try:
                self.update_detections()
                self.update_frame()
                time.sleep(REFRESH_RATE)
            except Exception as e:
                app.logger.error(f"Error in update loop: {str(e)}")
                time.sleep(1)  # Wait before retry
    
    def update_detections(self):
        try:
            response = requests.get(f"{API_BASE_URL}/detections/stream", stream=True, timeout=0.5)
            if response.status_code == 200:
                for line in response.iter_lines():
                    if line:
                        # SSE format: data: {...}
                        if line.startswith(b'data:'):
                            data_str = line[5:].strip()  # Remove 'data:' prefix
                            data = json.loads(data_str)
                            with update_lock:
                                global current_detections
                                current_detections = data
                            break  # Just get one update and return
        except requests.exceptions.RequestException as e:
            app.logger.error(f"Error fetching detections: {str(e)}")
    
    def update_frame(self):
        try:
            response = requests.get(f"{API_BASE_URL}/video_feed", stream=True, timeout=0.5)
            if response.status_code == 200:
                bytes_data = bytes()
                for chunk in response.iter_content(chunk_size=1024):
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        
                        # Decode image
                        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        
                        # Store the frame
                        with update_lock:
                            global current_frame
                            current_frame = frame
                        break  # Just get one frame and return
        except requests.exceptions.RequestException as e:
            app.logger.error(f"Error fetching video frame: {str(e)}")

def draw_bounding_boxes(frame, detections):
    """Draw bounding boxes on frame from detections data"""
    if frame is None or not detections:
        return frame
    
    # Make a copy to avoid modifying the original
    annotated_frame = frame.copy()
    
    # Colors for different detectors
    colors = {
        'color_detector': (0, 0, 255),    # Red for color detector
        'yolo_detector': (0, 255, 0)      # Green for YOLO detector
    }
    
    # Draw detections for each detector
    for detector_data in detections:
        detector_name = detector_data.get('detector_name', '')
        color = colors.get(detector_name, (255, 255, 255))  # Default to white
        
        for detection in detector_data.get('detections', []):
            bbox = detection.get('bounding_box', {})
            x = bbox.get('x_offset', 0)
            y = bbox.get('y_offset', 0)
            w = bbox.get('width', 0)
            h = bbox.get('height', 0)
            confidence = detection.get('confidence', 0)
            cls = detection.get('cls', '?')
            
            # Draw rectangle
            cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), color, 2)
            
            # Create label with class and confidence
            label = f"{detector_name}: {cls} ({confidence:.2f})"
            
            # Add text background for better visibility
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated_frame, (x, y - 20), (x + label_w, y), color, -1)
            
            # Add text
            cv2.putText(annotated_frame, label, (x, y - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    
    return annotated_frame

def generate_frames():
    """Generate frames with bounding boxes for video streaming"""
    while True:
        # Get the current frame and detections with thread safety
        with update_lock:
            if current_frame is not None:
                frame = current_frame.copy()
            else:
                frame = None
            detections = current_detections.copy()
        
        if frame is not None:
            # Draw bounding boxes on the frame
            annotated_frame = draw_bounding_boxes(frame, detections)
            
            # Encode the frame as JPEG
            _, buffer = cv2.imencode('.jpg', annotated_frame)
            frame_bytes = buffer.tobytes()
            
            # Yield the frame in multipart/x-mixed-replace format
            yield (b'--frame\r\n'
                  b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            # Return an empty image if no frame is available
            empty_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(empty_img, "Waiting for video...", (150, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            _, buffer = cv2.imencode('.jpg', empty_img)
            frame_bytes = buffer.tobytes()
            
            yield (b'--frame\r\n'
                  b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            # Wait a bit before trying again
            time.sleep(0.5)

@app.route('/')
def index():
    """Render the home page"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Route for the video feed"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Create and start the updater thread
    updater = DetectionUpdater()
    updater.start()
    
    # Run the Flask app
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)