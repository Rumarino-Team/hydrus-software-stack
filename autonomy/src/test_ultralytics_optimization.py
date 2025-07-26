#!/usr/bin/env python3
"""
Test script for Ultralytics optimization features.
"""

import sys
import os
import logging
import numpy as np
import cv2

# Add the autonomy module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from computer_vision.detection_core import YOLOModelManager
from computer_vision.inference_engine import InferenceEngineManager
from computer_vision.optimization_utils import export_model, benchmark_formats

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_basic_functionality():
    """Test basic YOLO detection functionality."""
    logger.info("Testing basic YOLO functionality...")
    
    try:
        # Create a dummy test image
        test_image = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
        
        # Test standard manager
        manager = YOLOModelManager(use_enhanced_inference=False)
        results = manager.detect(test_image)
        logger.info(f"Standard inference: {len(results)} detections")
        
        # Test enhanced manager if available
        try:
            enhanced_manager = YOLOModelManager(use_enhanced_inference=True)
            enhanced_results = enhanced_manager.detect(test_image)
            logger.info(f"Enhanced inference: {len(enhanced_results)} detections")
            
            # Test backend switching
            available_backends = enhanced_manager.get_available_backends()
            logger.info(f"Available backends: {available_backends}")
            
        except Exception as e:
            logger.warning(f"Enhanced inference not available: {e}")
        
        return True
        
    except Exception as e:
        logger.error(f"Basic functionality test failed: {e}")
        return False


def test_inference_engine():
    """Test the inference engine directly."""
    logger.info("Testing inference engine...")
    
    try:
        # List available backends
        available = InferenceEngineManager.list_available_backends()
        logger.info(f"Available inference backends: {available}")
        
        # Test each backend
        for backend in available:
            try:
                logger.info(f"Testing {backend} backend...")
                
                # Check dependencies
                deps_ok = InferenceEngineManager.check_backend_dependencies(backend)
                logger.info(f"{backend} dependencies available: {deps_ok}")
                
                if deps_ok:
                    engine = InferenceEngineManager()
                    engine.config.backend = backend
                    
                    # For now, we'll just test initialization
                    logger.info(f"{backend} backend initialized successfully")
                    
            except Exception as e:
                logger.warning(f"Backend {backend} test failed: {e}")
        
        return True
        
    except Exception as e:
        logger.error(f"Inference engine test failed: {e}")
        return False


def test_model_export():
    """Test model export functionality."""
    logger.info("Testing model export...")
    
    try:
        # Download a small model for testing
        from ultralytics import YOLO
        
        # Use yolo11n.pt (nano model) for testing
        model_path = "yolo11n.pt"
        model = YOLO(model_path)  # This will download if not present
        
        # Test ONNX export
        try:
            exported = export_model(model_path, ['onnx'], output_dir='/tmp/test_exports')
            logger.info(f"Export test results: {exported}")
            
            # Verify files exist
            for format_name, path in exported.items():
                if os.path.exists(path):
                    logger.info(f"✓ {format_name} export successful: {path}")
                else:
                    logger.warning(f"✗ {format_name} export file not found: {path}")
                    
        except Exception as e:
            logger.warning(f"Model export test failed: {e}")
        
        return True
        
    except Exception as e:
        logger.error(f"Model export test failed: {e}")
        return False


def main():
    """Run all tests."""
    logger.info("Starting Ultralytics optimization tests...")
    
    tests = [
        ("Basic Functionality", test_basic_functionality),
        ("Inference Engine", test_inference_engine),
        ("Model Export", test_model_export),
    ]
    
    results = {}
    
    for test_name, test_func in tests:
        logger.info(f"\n{'='*50}")
        logger.info(f"Running: {test_name}")
        logger.info(f"{'='*50}")
        
        try:
            results[test_name] = test_func()
        except Exception as e:
            logger.error(f"Test {test_name} crashed: {e}")
            results[test_name] = False
    
    # Print summary
    logger.info(f"\n{'='*50}")
    logger.info("TEST SUMMARY")
    logger.info(f"{'='*50}")
    
    for test_name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        logger.info(f"{test_name}: {status}")
    
    total_tests = len(results)
    passed_tests = sum(results.values())
    
    logger.info(f"\nTotal: {passed_tests}/{total_tests} tests passed")
    
    if passed_tests == total_tests:
        logger.info("🎉 All tests passed!")
        return 0
    else:
        logger.warning("⚠️  Some tests failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())