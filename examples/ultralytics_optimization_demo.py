#!/usr/bin/env python3
"""
Example script demonstrating Ultralytics optimization features.
This script shows how to use different inference backends and compare their performance.
"""

import argparse
import logging
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Add autonomy module to path
sys.path.insert(0, str(Path(__file__).parent.parent / "autonomy" / "src"))

try:
    from computer_vision.detection_core import YOLOModelManager, DetectionPipelineManager
    from computer_vision.inference_engine import InferenceEngineManager
    from computer_vision.optimization_utils import export_model, benchmark_formats
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure you're running from the correct directory and dependencies are installed")
    sys.exit(1)

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)


def create_test_image(width=640, height=640):
    """Create a test image with some simple shapes for detection."""
    image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Add some colored rectangles that might be detected
    cv2.rectangle(image, (100, 100), (200, 200), (255, 0, 0), -1)  # Blue rectangle
    cv2.rectangle(image, (300, 300), (450, 400), (0, 255, 0), -1)  # Green rectangle
    cv2.circle(image, (500, 150), 50, (0, 0, 255), -1)  # Red circle
    
    # Add some noise
    noise = np.random.randint(0, 50, (height, width, 3), dtype=np.uint8)
    image = cv2.addWeighted(image, 0.8, noise, 0.2, 0)
    
    return image


def demo_basic_usage():
    """Demonstrate basic usage of optimized YOLO inference."""
    logger.info("=== Basic Usage Demo ===")
    
    # Create test image
    test_image = create_test_image()
    
    # Standard inference
    logger.info("Testing standard PyTorch inference...")
    manager = YOLOModelManager(use_enhanced_inference=False)
    start_time = time.time()
    results = manager.detect(test_image)
    pytorch_time = time.time() - start_time
    logger.info(f"PyTorch: {len(results)} detections in {pytorch_time:.3f}s")
    
    # Enhanced inference with PyTorch backend
    logger.info("Testing enhanced inference (PyTorch backend)...")
    enhanced_manager = YOLOModelManager(use_enhanced_inference=True, inference_backend='pytorch')
    start_time = time.time()
    enhanced_results = enhanced_manager.detect(test_image)
    enhanced_time = time.time() - start_time
    logger.info(f"Enhanced PyTorch: {len(enhanced_results)} detections in {enhanced_time:.3f}s")
    
    # Check available backends
    available_backends = enhanced_manager.get_available_backends()
    logger.info(f"Available backends: {available_backends}")
    
    return test_image, enhanced_manager


def demo_backend_switching(manager, test_image):
    """Demonstrate switching between different inference backends."""
    logger.info("\n=== Backend Switching Demo ===")
    
    available_backends = manager.get_available_backends()
    
    for backend in available_backends:
        try:
            logger.info(f"Testing {backend} backend...")
            
            # Switch backend
            success = manager.set_inference_backend(backend)
            if not success:
                logger.warning(f"Failed to switch to {backend} backend")
                continue
            
            # Run inference
            start_time = time.time()
            results = manager.detect(test_image)
            inference_time = time.time() - start_time
            
            # Get performance stats
            stats = manager.get_performance_stats()
            
            logger.info(f"{backend}: {len(results)} detections in {inference_time:.3f}s")
            if 'avg_inference_time' in stats:
                logger.info(f"  Average inference time: {stats['avg_inference_time']:.3f}s")
                logger.info(f"  Estimated FPS: {stats.get('fps', 0):.1f}")
            
        except Exception as e:
            logger.warning(f"Backend {backend} failed: {e}")


def demo_model_export(model_path="yolo11n.pt"):
    """Demonstrate model export to different formats."""
    logger.info("\n=== Model Export Demo ===")
    
    # Check if model exists, download if needed
    if not os.path.exists(model_path):
        logger.info(f"Downloading {model_path}...")
        from ultralytics import YOLO
        model = YOLO(model_path)  # This will download the model
    
    # Export to ONNX format
    try:
        logger.info("Exporting model to ONNX format...")
        exported = export_model(
            model_path,
            formats=['onnx'],
            output_dir='./demo_exports'
        )
        
        for format_name, path in exported.items():
            if os.path.exists(path):
                size_mb = os.path.getsize(path) / 1024 / 1024
                logger.info(f"✓ {format_name}: {path} ({size_mb:.1f} MB)")
            else:
                logger.warning(f"✗ {format_name}: Export failed or file not found")
                
    except Exception as e:
        logger.warning(f"Model export failed: {e}")


def demo_performance_benchmark(manager, test_image, num_runs=10):
    """Demonstrate performance benchmarking."""
    logger.info(f"\n=== Performance Benchmark Demo ({num_runs} runs) ===")
    
    available_backends = manager.get_available_backends()
    
    for backend in available_backends:
        try:
            logger.info(f"Benchmarking {backend} backend...")
            
            # Switch to backend
            success = manager.set_inference_backend(backend)
            if not success:
                continue
            
            # Run benchmark
            stats = manager.benchmark(test_image, num_runs=num_runs)
            
            logger.info(f"{backend} Results:")
            logger.info(f"  Average FPS: {stats.get('fps', 0):.2f}")
            logger.info(f"  Average time: {stats.get('avg_inference_time', 0):.4f}s")
            logger.info(f"  Min time: {stats.get('min_inference_time', 0):.4f}s")
            logger.info(f"  Max time: {stats.get('max_inference_time', 0):.4f}s")
            
        except Exception as e:
            logger.warning(f"Benchmark failed for {backend}: {e}")


def demo_detection_pipeline():
    """Demonstrate the full detection pipeline with optimization."""
    logger.info("\n=== Detection Pipeline Demo ===")
    
    test_image = create_test_image()
    
    # Test with enhanced inference
    try:
        pipeline = DetectionPipelineManager(
            use_enhanced_inference=True,
            inference_backend='pytorch'
        )
        
        # Run full detection pipeline
        results = pipeline.run_detections(image=test_image)
        
        logger.info("Detection Pipeline Results:")
        for detector_name, detections in results:
            logger.info(f"  {detector_name}: {len(detections)} detections")
            
    except Exception as e:
        logger.warning(f"Detection pipeline failed: {e}")


def main():
    """Main function with command line interface."""
    parser = argparse.ArgumentParser(description="Ultralytics Optimization Demo")
    parser.add_argument('--model', default='yolo11n.pt', help='YOLO model to use')
    parser.add_argument('--benchmark-runs', type=int, default=10, help='Number of benchmark runs')
    parser.add_argument('--export', action='store_true', help='Demo model export')
    parser.add_argument('--image', help='Path to test image (optional)')
    parser.add_argument('--save-image', help='Save test image to path')
    
    args = parser.parse_args()
    
    logger.info("🚀 Ultralytics Optimization Demo")
    logger.info("=" * 50)
    
    # Create or load test image
    if args.image and os.path.exists(args.image):
        test_image = cv2.imread(args.image)
        logger.info(f"Loaded test image: {args.image}")
    else:
        test_image = create_test_image()
        logger.info("Created synthetic test image")
    
    # Save test image if requested
    if args.save_image:
        cv2.imwrite(args.save_image, test_image)
        logger.info(f"Saved test image to: {args.save_image}")
    
    try:
        # Basic usage demo
        test_image, manager = demo_basic_usage()
        
        # Backend switching demo
        demo_backend_switching(manager, test_image)
        
        # Model export demo
        if args.export:
            demo_model_export(args.model)
        
        # Performance benchmark
        demo_performance_benchmark(manager, test_image, args.benchmark_runs)
        
        # Detection pipeline demo
        demo_detection_pipeline()
        
        logger.info("\n🎉 Demo completed successfully!")
        
    except KeyboardInterrupt:
        logger.info("\n⏹️  Demo interrupted by user")
    except Exception as e:
        logger.error(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())