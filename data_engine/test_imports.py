#!/usr/bin/env python3
"""
Quick test to verify imports and basic functionality
"""

import sys

print("Testing basic imports...")

try:
    import torch

    print(f"✓ PyTorch {torch.__version__}")
except ImportError as e:
    print(f"✗ PyTorch: {e}")

try:
    import cv2

    print(f"✓ OpenCV {cv2.__version__}")
except ImportError as e:
    print(f"✗ OpenCV: {e}")

try:
    import importlib.util

    ultralytics_spec = importlib.util.find_spec("ultralytics")
    if ultralytics_spec is not None:
        print("✓ Ultralytics (for YOLO only)")
    else:
        print("✗ Ultralytics: not found")
except ImportError as e:
    print(f"✗ Ultralytics: {e}")

try:
    from PySide6.QtWidgets import QApplication

    print("✓ PySide6")
except ImportError as e:
    print(f"✗ PySide6: {e}")

print("\nTesting SAM2 imports...")
try:
    sys.path.append("/app/third_party/sam2")
    from sam2.sam2_video_predictor import SAM2VideoPredictor

    print("✓ SAM2VideoPredictor")
except ImportError as e:
    print(f"✗ SAM2VideoPredictor: {e}")

try:
    from sam2.build_sam import build_sam2_video_predictor

    print("✓ build_sam2_video_predictor")
except ImportError as e:
    print(f"✗ build_sam2_video_predictor: {e}")

print("\nTesting GUI module imports...")
try:
    from gui.sam2_video_worker import SAM2VideoWorker

    print("✓ SAM2VideoWorker")
except ImportError as e:
    print(f"✗ SAM2VideoWorker: {e}")

try:
    from gui.main_window import DataEngineMainWindow

    print("✓ DataEngineMainWindow")
except ImportError as e:
    print(f"✗ DataEngineMainWindow: {e}")

print("\nAll import tests completed!")
