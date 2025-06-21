#!/usr/bin/env python3
"""
Launch script for the SAM2 Data Engine
"""

import os
import sys
from pathlib import Path

# Add the data_engine directory to Python path
data_engine_dir = Path(__file__).parent
sys.path.insert(0, str(data_engine_dir))


def check_dependencies():
    """Check if all required dependencies are installed"""
    missing_deps = []

    try:
        import cv2
    except ImportError:
        missing_deps.append("opencv-python")

    try:
        import numpy
    except ImportError:
        missing_deps.append("numpy")

    try:
        import torch
    except ImportError:
        missing_deps.append("torch")

    try:
        from PySide6 import QtWidgets
    except ImportError:
        missing_deps.append("PySide6")

    try:
        from ultralytics import SAM
    except ImportError:
        missing_deps.append("ultralytics")

    try:
        from PIL import Image
    except ImportError:
        missing_deps.append("Pillow")

    if missing_deps:
        print("Missing dependencies:")
        for dep in missing_deps:
            print(f"  - {dep}")
        print("\nPlease install missing dependencies:")
        print(f"pip install {' '.join(missing_deps)}")
        print("\nOr run the setup script:")
        print("python setup.py")
        return False

    return True


def main():
    """Main launch function"""
    print("SAM2 Data Engine")
    print("=" * 30)

    # Check dependencies
    if not check_dependencies():
        print("\nDependency check failed!")
        return 1

    print("✓ All dependencies found")

    # Check for CUDA
    try:
        import torch

        if torch.cuda.is_available():
            print(f"✓ CUDA available: {torch.cuda.get_device_name()}")
        else:
            print("⚠ CUDA not available, using CPU")
    except:
        print("⚠ Could not check CUDA availability")

    # Launch the application
    try:
        print("\nLaunching Data Engine...")
        from main import main as app_main

        return app_main()
    except ImportError as e:
        print(f"Error importing main application: {e}")
        return 1
    except Exception as e:
        print(f"Error launching application: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
