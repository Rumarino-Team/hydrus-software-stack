#!/bin/bash
# Run the Flask web application for detection visualization

# Change to the script directory
cd "$(dirname "$0")"

# Make the script executable
chmod +x detection_viewer.py

# Run the Flask application
python3 detection_viewer.py