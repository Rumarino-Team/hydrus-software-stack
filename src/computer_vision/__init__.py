"""
Computer Vision Library for Hydrus

This package provides computer vision functionality including:
- YOLO object detection
- Color filtering
- Depth estimation
- SLAM capabilities
- 3D point calculations
"""

from .custom_types import Point3D, Rotation3D, Detection, Detections
from .detection_core import (
    ColorFilterConfig,
    YOLOModelManager,
    DetectionPipelineManager
)
from .depth_estimation import DepthEstimator
from .slam import MonocularSLAM

__version__ = "0.1.0"

__all__ = [
    # Types
    "Point3D",
    "Rotation3D", 
    "Detection",
    "Detections",
    # Detection Core
    "ColorFilterConfig",
    "YOLOModelManager",
    "DetectionPipelineManager",
    # Other modules
    "DepthEstimator",
    "MonocularSLAM",
]
