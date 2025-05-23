#!/usr/bin/env python3
"""
Download ROS bag files for the Hydrus Software Stack.
This script checks if there are any rosbags in the rosbags/ directory,
and if not, offers to download the default one mentioned in the README.
"""

import os
import sys
import requests
from pathlib import Path
import subprocess
import re
import gdown

# Constants
DEFAULT_ROSBAG_URL = "https://drive.google.com/file/d/16Lr-CbW1rW6rKh8_mWClTQMIjm2u0y8X/view?usp=drive_link"
DOWNLOAD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rosbags")

def check_rosbags_exist():
    """Check if any .bag files exist in the rosbags directory"""
    if not os.path.exists(DOWNLOAD_DIR):
        os.makedirs(DOWNLOAD_DIR)
        print(f"Created directory {DOWNLOAD_DIR}")
        return False
    
    bag_files = list(Path(DOWNLOAD_DIR).glob("*.bag"))
    return len(bag_files) > 0

def extract_file_id(url):
    """Extract the file ID from a Google Drive URL"""
    pattern = r"drive\.google\.com/file/d/([a-zA-Z0-9_-]+)"
    match = re.search(pattern, url)
    if match:
        return match.group(1)
    return None

def download_rosbag(url=DEFAULT_ROSBAG_URL):
    """Download the rosbag file from the given URL using gdown"""
    print(f"Downloading rosbag from {url}")
    print("This may take a few minutes depending on your internet connection...")
    
    file_id = extract_file_id(url)
    if not file_id:
        print("Error: Could not extract file ID from the URL.")
        return False
    
    output_path = os.path.join(DOWNLOAD_DIR, "zed2i_camera.bag")
    
    try:
        # Use gdown to download from Google Drive
        gdown.download(id=file_id, output=output_path, quiet=False)
        
        if os.path.exists(output_path) and os.path.getsize(output_path) > 0:
            print(f"Successfully downloaded rosbag to {output_path}")
            return True
        else:
            print("Error: Download failed or file is empty.")
            return False
    except Exception as e:
        print(f"Error downloading file: {e}")
        return False

def install_dependencies():
    """Install required dependencies for downloading"""
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "gdown", "requests"])
        return True
    except subprocess.CalledProcessError:
        print("Failed to install dependencies. Please install them manually:")
        print("pip install gdown requests")
        return False

def main():
    """Main function to check for and download rosbag files"""
    print("Checking for ROS bag files...")
    
    if check_rosbags_exist():
        print("ROS bag files already exist in the rosbags directory.")
        return 0
    
    print("No ROS bag files found.")
    while True:
        choice = input("Would you like to download the default ZED2i Camera rosbag? (y/n): ").lower()
        if choice in ['y', 'yes']:
            if not install_dependencies():
                return 1
            
            if download_rosbag():
                print("Download complete! You can now play the rosbag.")
                return 0
            else:
                print("Download failed. Please download the file manually from:")
                print(DEFAULT_ROSBAG_URL)
                return 1
        elif choice in ['n', 'no']:
            print("No files downloaded. Please add rosbag files manually to the rosbags directory.")
            return 0
        else:
            print("Invalid input. Please enter 'y' or 'n'.")

if __name__ == "__main__":
    sys.exit(main())