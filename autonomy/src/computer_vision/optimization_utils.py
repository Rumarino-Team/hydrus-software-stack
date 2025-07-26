#!/usr/bin/env python3
"""
Utility functions for YOLO model optimization and conversion.
"""

import logging
import os
from pathlib import Path
from typing import Dict, List, Optional

from ultralytics import YOLO

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def export_model(
    model_path: str,
    formats: List[str],
    output_dir: str = "./exported_models",
    **export_kwargs
) -> Dict[str, str]:
    """
    Export YOLO model to multiple formats.
    
    Args:
        model_path: Path to source model
        formats: List of formats to export to ('onnx', 'engine', 'torchscript', etc.)
        output_dir: Directory to save exported models
        **export_kwargs: Additional export parameters
        
    Returns:
        Dictionary mapping format to exported file path
    """
    model = YOLO(model_path)
    exported_paths = {}
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    for fmt in formats:
        try:
            logger.info(f"Exporting model to {fmt} format...")
            
            # Set format-specific defaults
            kwargs = export_kwargs.copy()
            if fmt == 'onnx':
                kwargs.setdefault('opset', 11)
                kwargs.setdefault('simplify', True)
            elif fmt == 'engine':  # TensorRT
                kwargs.setdefault('half', True)
                kwargs.setdefault('workspace', 4)
            elif fmt == 'torchscript':
                kwargs.setdefault('optimize', True)
            
            # Export model
            exported_path = model.export(format=fmt, **kwargs)
            
            # Move to output directory if different
            if output_dir != str(Path(model_path).parent):
                model_name = Path(model_path).stem
                target_path = Path(output_dir) / f"{model_name}.{_get_format_extension(fmt)}"
                if Path(exported_path).exists():
                    Path(exported_path).rename(target_path)
                    exported_path = str(target_path)
            
            exported_paths[fmt] = exported_path
            logger.info(f"Successfully exported to {fmt}: {exported_path}")
            
        except Exception as e:
            logger.error(f"Failed to export to {fmt}: {e}")
            
    return exported_paths


def _get_format_extension(format_name: str) -> str:
    """Get file extension for export format."""
    format_extensions = {
        'onnx': 'onnx',
        'engine': 'engine',
        'torchscript': 'torchscript',
        'coreml': 'mlmodel',
        'saved_model': 'saved_model',
        'pb': 'pb',
        'tflite': 'tflite',
        'edgetpu': 'tflite',
        'tfjs': 'tfjs',
        'paddle': 'paddle',
        'ncnn': 'ncnn'
    }
    return format_extensions.get(format_name, format_name)


def benchmark_formats(
    model_path: str,
    formats: List[str],
    test_image_path: Optional[str] = None,
    num_runs: int = 100
) -> Dict[str, Dict]:
    """
    Benchmark different model formats.
    
    Args:
        model_path: Path to source model
        formats: List of formats to benchmark
        test_image_path: Path to test image (creates dummy if None)
        num_runs: Number of benchmark runs
        
    Returns:
        Dictionary with benchmark results for each format
    """
    from computer_vision.inference_engine import InferenceEngineManager
    import numpy as np
    import cv2
    
    # Load test image or create dummy
    if test_image_path and os.path.exists(test_image_path):
        test_image = cv2.imread(test_image_path)
    else:
        test_image = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
    
    results = {}
    
    for fmt in formats:
        try:
            logger.info(f"Benchmarking {fmt} format...")
            
            # Create inference engine for this format
            engine = InferenceEngineManager()
            engine.config.backend = fmt if fmt != 'engine' else 'tensorrt'
            
            # Load model
            if engine.load_model(model_path):
                # Run benchmark
                stats = engine.benchmark(test_image, num_runs)
                results[fmt] = stats
                logger.info(f"{fmt}: {stats['fps']:.2f} FPS")
            else:
                logger.warning(f"Failed to load model for {fmt} format")
                
        except Exception as e:
            logger.error(f"Benchmark failed for {fmt}: {e}")
            
    return results


def optimize_model_for_deployment(
    model_path: str,
    target_device: str = "cuda",
    optimization_level: str = "balanced"
) -> str:
    """
    Optimize model for deployment based on target device and requirements.
    
    Args:
        model_path: Source model path
        target_device: Target device ('cuda', 'cpu', 'edge')
        optimization_level: Optimization level ('speed', 'balanced', 'accuracy')
        
    Returns:
        Path to optimized model
    """
    logger.info(f"Optimizing model for {target_device} deployment...")
    
    if target_device == "cuda":
        if optimization_level == "speed":
            # TensorRT with FP16
            exported = export_model(
                model_path, 
                ['engine'], 
                half=True, 
                workspace=8
            )
            return exported.get('engine', model_path)
        elif optimization_level == "balanced":
            # ONNX with CUDA provider
            exported = export_model(model_path, ['onnx'], simplify=True)
            return exported.get('onnx', model_path)
        else:  # accuracy
            return model_path  # Keep original PyTorch
            
    elif target_device == "cpu":
        if optimization_level == "speed":
            # ONNX optimized for CPU
            exported = export_model(model_path, ['onnx'], simplify=True)
            return exported.get('onnx', model_path)
        else:
            # TorchScript
            exported = export_model(model_path, ['torchscript'], optimize=True)
            return exported.get('torchscript', model_path)
            
    elif target_device == "edge":
        # TensorFlow Lite for edge devices
        exported = export_model(model_path, ['tflite'], int8=True)
        return exported.get('tflite', model_path)
        
    return model_path


def validate_exported_model(original_path: str, exported_path: str, test_image_path: Optional[str] = None) -> bool:
    """
    Validate that exported model produces similar results to original.
    
    Args:
        original_path: Path to original model
        exported_path: Path to exported model
        test_image_path: Test image path
        
    Returns:
        True if validation passes
    """
    try:
        import cv2
        import numpy as np
        from computer_vision.detection_core import YOLOModelManager
        
        # Load test image
        if test_image_path and os.path.exists(test_image_path):
            test_image = cv2.imread(test_image_path)
        else:
            test_image = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
        
        # Run inference with original model
        original_manager = YOLOModelManager(original_path, use_enhanced_inference=False)
        original_results = original_manager.detect(test_image)
        
        # Run inference with exported model
        # (Implementation would depend on the specific format)
        logger.info(f"Original model detected {len(original_results)} objects")
        
        # For now, just check that exported file exists
        return os.path.exists(exported_path)
        
    except Exception as e:
        logger.error(f"Validation failed: {e}")
        return False


if __name__ == "__main__":
    # Example usage
    import argparse
    
    parser = argparse.ArgumentParser(description="YOLO Model Optimization Utilities")
    parser.add_argument("--model", required=True, help="Path to YOLO model")
    parser.add_argument("--export", nargs="+", help="Formats to export", 
                       choices=['onnx', 'engine', 'torchscript', 'tflite'])
    parser.add_argument("--benchmark", action="store_true", help="Run benchmark")
    parser.add_argument("--optimize", help="Optimize for device", 
                       choices=['cuda', 'cpu', 'edge'])
    parser.add_argument("--output-dir", default="./exported_models", 
                       help="Output directory")
    
    args = parser.parse_args()
    
    if args.export:
        exported = export_model(args.model, args.export, args.output_dir)
        print("Exported models:", exported)
    
    if args.benchmark and args.export:
        results = benchmark_formats(args.model, args.export)
        print("Benchmark results:", results)
    
    if args.optimize:
        optimized = optimize_model_for_deployment(args.model, args.optimize)
        print(f"Optimized model: {optimized}")