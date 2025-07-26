# Ultralytics Optimization Integration

This document describes the new Ultralytics optimization features that enable configurable inference backends for improved performance on different hardware platforms.

## Overview

The Hydrus Software Stack now supports multiple inference backends for YOLO models, allowing you to optimize performance based on your deployment target:

- **PyTorch** (default): Standard Ultralytics inference
- **ONNX Runtime**: Cross-platform optimized inference with CPU/GPU support
- **TensorRT**: NVIDIA GPU-accelerated inference (requires additional dependencies)
- **TorchScript**: PyTorch JIT compilation for production deployment

## Quick Start

### Basic Usage

```python
from computer_vision.detection_core import YOLOModelManager
import numpy as np

# Create test image
image = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)

# Standard inference (PyTorch)
manager = YOLOModelManager()
results = manager.detect(image)

# Enhanced inference with ONNX backend
manager = YOLOModelManager(use_enhanced_inference=True, inference_backend='onnx')
results = manager.detect(image)
```

### Backend Switching

```python
# List available backends
available = manager.get_available_backends()
print(f"Available backends: {available}")

# Switch to ONNX backend
success = manager.set_inference_backend('onnx')
if success:
    results = manager.detect(image)
    print(f"ONNX inference completed: {len(results)} detections")
```

### Performance Benchmarking

```python
# Run performance benchmark
stats = manager.benchmark(test_image, num_runs=100)
print(f"Average FPS: {stats['fps']:.2f}")
print(f"Average inference time: {stats['avg_inference_time']:.4f}s")
```

## Installation

### Core Dependencies

The optimization features require additional packages. Install them using:

```bash
# Basic ONNX support
pip install -e .[ultralytics-acceleration]

# Full optimization support
pip install -e .[ultralytics-acceleration,tensorrt]
```

### Manual Installation

```bash
# ONNX Runtime support
pip install onnx onnxruntime onnxruntime-gpu

# TensorRT support (NVIDIA GPUs only)
pip install tensorrt pycuda nvidia-tensorrt

# PyTorch (usually already installed)
pip install torch torchvision
```

## Configuration

### Configuration File

Create or modify `config/inference_config.yaml`:

```yaml
# Default inference backend
default_backend: 'pytorch'

# Backend-specific settings
backends:
  pytorch:
    device: 'auto'
    half: false
    
  onnx:
    providers: ['CUDAExecutionProvider', 'CPUExecutionProvider']
    device: 'auto'
    half: false
    
  tensorrt:
    device: 'cuda:0'
    half: true
    workspace_size: 4  # GB
    
  torchscript:
    device: 'auto'
    optimize: true

# Export settings
export:
  auto_export: true
  export_dir: './exported_models'
  keep_original: true
```

### Programmatic Configuration

```python
from computer_vision.inference_engine import InferenceEngineManager

# Create engine with custom config
engine = InferenceEngineManager(config_path='path/to/config.yaml')

# Or modify config directly
engine.config.backend = 'onnx'
engine.config.auto_export = True
```

## Model Export and Conversion

### Automated Export

```python
from computer_vision.optimization_utils import export_model

# Export to multiple formats
exported = export_model(
    'yolo11n.pt',
    formats=['onnx', 'torchscript'],
    output_dir='./optimized_models'
)
print(f"Exported models: {exported}")
```

### Command Line Export

```bash
cd autonomy/src
python computer_vision/optimization_utils.py \
    --model yolo11n.pt \
    --export onnx torchscript \
    --output-dir ./exported_models
```

### Optimization for Deployment

```python
from computer_vision.optimization_utils import optimize_model_for_deployment

# Optimize for CUDA deployment (speed priority)
optimized_path = optimize_model_for_deployment(
    'yolo11n.pt',
    target_device='cuda',
    optimization_level='speed'
)

# Optimize for CPU deployment
optimized_path = optimize_model_for_deployment(
    'yolo11n.pt', 
    target_device='cpu',
    optimization_level='balanced'
)
```

## Performance Comparison

### Benchmarking Different Backends

```python
from computer_vision.optimization_utils import benchmark_formats

# Compare performance across formats
results = benchmark_formats(
    'yolo11n.pt',
    formats=['pytorch', 'onnx', 'torchscript'],
    num_runs=100
)

for backend, stats in results.items():
    print(f"{backend}: {stats['fps']:.2f} FPS")
```

### Expected Performance

| Backend | CPU Performance | GPU Performance | Memory Usage | Notes |
|---------|----------------|-----------------|--------------|--------|
| PyTorch | Baseline | Baseline | High | Default, full features |
| ONNX | 1.2-1.5x | 1.1-1.3x | Medium | Good cross-platform |
| TensorRT | N/A | 2-4x | Low | NVIDIA GPUs only |
| TorchScript | 1.1-1.2x | 1.1-1.2x | Medium | Production ready |

*Performance varies by model size, input resolution, and hardware.*

## Advanced Usage

### Custom Inference Pipeline

```python
from computer_vision.inference_engine import InferenceEngineManager

# Create custom inference pipeline
engine = InferenceEngineManager()

# Load model with specific backend
engine.load_model('yolo11n.pt', backend='onnx')

# Warmup for consistent performance
engine.warmup()

# Run inference
results = engine.predict(image)

# Get detailed performance stats
stats = engine.get_performance_stats()
```

### Integration with Detection Pipeline

```python
from computer_vision.detection_core import DetectionPipelineManager

# Create pipeline with optimized inference
pipeline = DetectionPipelineManager(
    use_enhanced_inference=True,
    inference_backend='onnx'
)

# Run full detection pipeline
results = pipeline.run_detections(
    image=rgb_image,
    depth_image=depth_image,
    camera_intrinsics=(fx, fy, cx, cy)
)

# Results include both YOLO and color filter detections
for detector_name, detections in results:
    print(f"{detector_name}: {len(detections)} detections")
```

## Troubleshooting

### Common Issues

1. **ONNX export fails**
   ```bash
   pip install onnx onnxruntime onnxslim
   ```

2. **TensorRT not available**
   - Ensure NVIDIA GPU with CUDA support
   - Install TensorRT following NVIDIA documentation
   - Verify CUDA version compatibility

3. **Performance worse than expected**
   - Run warmup iterations
   - Check device configuration (CPU vs GPU)
   - Verify model is properly converted

4. **Memory issues**
   - Reduce batch size
   - Use FP16 precision where supported
   - Monitor GPU memory usage

### Debugging

```python
# Enable debug logging
import logging
logging.basicConfig(level=logging.DEBUG)

# Check backend dependencies
from computer_vision.inference_engine import InferenceEngineManager
available = InferenceEngineManager.list_available_backends()
print(f"Available backends: {available}")

# Check specific backend
backend_ok = InferenceEngineManager.check_backend_dependencies('onnx')
print(f"ONNX backend available: {backend_ok}")
```

## Integration with Existing Code

### Backward Compatibility

The optimization features are designed to be backward compatible:

```python
# Existing code continues to work
manager = YOLOModelManager()
results = manager.detect(image)

# Enable optimizations with minimal changes
manager = YOLOModelManager(use_enhanced_inference=True)
results = manager.detect(image)  # Same interface
```

### Migration Guide

1. **Update imports** (if using advanced features):
   ```python
   from computer_vision.inference_engine import InferenceEngineManager
   from computer_vision.optimization_utils import export_model
   ```

2. **Install dependencies**:
   ```bash
   pip install -e .[ultralytics-acceleration]
   ```

3. **Update configuration** (optional):
   - Create `config/inference_config.yaml`
   - Set preferred backend and optimization settings

4. **Test performance**:
   ```python
   stats = manager.benchmark(test_image)
   ```

## Contributing

To contribute to the optimization features:

1. Follow the existing code structure in `computer_vision/`
2. Add tests to `test_ultralytics_optimization.py`
3. Update this documentation for new features
4. Ensure backward compatibility

## References

- [Ultralytics Integrations Documentation](https://docs.ultralytics.com/integrations/)
- [ONNX Runtime Documentation](https://onnxruntime.ai/)
- [TensorRT Documentation](https://developer.nvidia.com/tensorrt)
- [PyTorch JIT Documentation](https://pytorch.org/docs/stable/jit.html)