#!/usr/bin/env python3
"""
Enhanced YOLO inference engine manager supporting multiple backends.
Supports PyTorch, ONNX Runtime, TensorRT, and PyTorch JIT (TorchScript).
"""

import logging
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Union

import numpy as np
import yaml
from ultralytics import YOLO

# Optional imports with graceful fallback
try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False

try:
    import tensorrt as trt
    import pycuda.driver as cuda
    import pycuda.autoinit
    TENSORRT_AVAILABLE = True
except ImportError:
    TENSORRT_AVAILABLE = False

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


@dataclass
class InferenceConfig:
    """Configuration for inference engines."""
    backend: str = 'pytorch'
    device: str = 'auto'
    half: bool = False
    providers: Optional[List[str]] = None
    workspace_size: int = 4
    max_batch_size: int = 1
    optimize: bool = True
    warmup_runs: int = 3
    auto_export: bool = False
    export_dir: str = './exported_models'


class InferenceEngineManager:
    """
    Advanced YOLO inference engine manager with support for multiple backends.
    """
    
    SUPPORTED_BACKENDS = ['pytorch', 'onnx', 'tensorrt', 'torchscript']
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize the inference engine manager.
        
        Args:
            config_path: Path to configuration YAML file
        """
        self.logger = logging.getLogger(__name__)
        self.config = self._load_config(config_path)
        self.model = None
        self.backend = None
        self.model_path = None
        self.session = None  # For ONNX Runtime
        self.engine = None   # For TensorRT
        self.inference_times = []
        
    def _load_config(self, config_path: Optional[str]) -> InferenceConfig:
        """Load configuration from YAML file."""
        if config_path is None:
            # Default config path
            config_path = Path(__file__).parent.parent.parent / "config" / "inference_config.yaml"
        
        try:
            with open(config_path, 'r') as f:
                config_dict = yaml.safe_load(f)
            
            # Extract main settings
            backend = config_dict.get('default_backend', 'pytorch')
            backend_config = config_dict.get('backends', {}).get(backend, {})
            export_config = config_dict.get('export', {})
            optimization_config = config_dict.get('optimization', {})
            
            return InferenceConfig(
                backend=backend,
                device=backend_config.get('device', 'auto'),
                half=backend_config.get('half', False),
                providers=backend_config.get('providers'),
                workspace_size=backend_config.get('workspace_size', 4),
                max_batch_size=backend_config.get('max_batch_size', 1),
                optimize=backend_config.get('optimize', True),
                warmup_runs=optimization_config.get('warmup_runs', 3),
                auto_export=export_config.get('auto_export', False),
                export_dir=export_config.get('export_dir', './exported_models')
            )
            
        except Exception as e:
            self.logger.warning(f"Failed to load config from {config_path}: {e}")
            self.logger.info("Using default configuration")
            return InferenceConfig()
    
    def load_model(self, model_path: str, backend: Optional[str] = None) -> bool:
        """
        Load a YOLO model with the specified backend.
        
        Args:
            model_path: Path to the model file
            backend: Inference backend to use (overrides config)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if backend:
                self.config.backend = backend
                
            if self.config.backend not in self.SUPPORTED_BACKENDS:
                raise ValueError(f"Unsupported backend: {self.config.backend}")
            
            self.model_path = model_path
            
            if self.config.backend == 'pytorch':
                return self._load_pytorch_model(model_path)
            elif self.config.backend == 'onnx':
                return self._load_onnx_model(model_path)
            elif self.config.backend == 'tensorrt':
                return self._load_tensorrt_model(model_path)
            elif self.config.backend == 'torchscript':
                return self._load_torchscript_model(model_path)
                
        except Exception as e:
            self.logger.error(f"Failed to load model {model_path} with backend {self.config.backend}: {e}")
            return False
    
    def _load_pytorch_model(self, model_path: str) -> bool:
        """Load PyTorch model."""
        self.model = YOLO(model_path)
        self.backend = 'pytorch'
        self.logger.info(f"Loaded PyTorch model: {model_path}")
        return True
    
    def _load_onnx_model(self, model_path: str) -> bool:
        """Load ONNX model."""
        if not ONNX_AVAILABLE:
            raise ImportError("ONNX Runtime not available. Install with: pip install onnxruntime-gpu")
            
        # Convert .pt to .onnx if needed
        onnx_path = self._ensure_onnx_model(model_path)
        
        providers = self.config.providers or ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(onnx_path, providers=providers)
        self.backend = 'onnx'
        
        # Get input/output info
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]
        
        self.logger.info(f"Loaded ONNX model: {onnx_path}")
        self.logger.info(f"Using provider: {self.session.get_providers()[0]}")
        return True
    
    def _load_tensorrt_model(self, model_path: str) -> bool:
        """Load TensorRT model."""
        if not TENSORRT_AVAILABLE:
            raise ImportError("TensorRT not available. Install TensorRT and pycuda")
            
        # Convert to TensorRT engine if needed
        engine_path = self._ensure_tensorrt_engine(model_path)
        
        # Load TensorRT engine
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(trt.Logger(trt.Logger.WARNING)).deserialize_cuda_engine(f.read())
            
        self.backend = 'tensorrt'
        self.logger.info(f"Loaded TensorRT engine: {engine_path}")
        return True
    
    def _load_torchscript_model(self, model_path: str) -> bool:
        """Load TorchScript model."""
        if not TORCH_AVAILABLE:
            raise ImportError("PyTorch not available")
            
        # Convert to TorchScript if needed
        torchscript_path = self._ensure_torchscript_model(model_path)
        
        self.model = torch.jit.load(torchscript_path)
        if self.config.device != 'cpu' and torch.cuda.is_available():
            self.model = self.model.cuda()
        self.model.eval()
        
        self.backend = 'torchscript'
        self.logger.info(f"Loaded TorchScript model: {torchscript_path}")
        return True
    
    def _ensure_onnx_model(self, model_path: str) -> str:
        """Ensure ONNX model exists, export if needed."""
        if model_path.endswith('.onnx'):
            return model_path
            
        # Generate ONNX path
        onnx_path = Path(self.config.export_dir) / f"{Path(model_path).stem}.onnx"
        onnx_path.parent.mkdir(parents=True, exist_ok=True)
        
        if onnx_path.exists() and not self.config.auto_export:
            return str(onnx_path)
            
        # Export to ONNX
        if self.config.auto_export:
            self.logger.info(f"Exporting {model_path} to ONNX format...")
            model = YOLO(model_path)
            model.export(format='onnx', half=self.config.half, simplify=True)
            # Move to export directory
            exported_path = Path(model_path).with_suffix('.onnx')
            if exported_path.exists():
                onnx_path.parent.mkdir(parents=True, exist_ok=True)
                exported_path.rename(onnx_path)
            return str(onnx_path)
        else:
            raise FileNotFoundError(f"ONNX model not found: {onnx_path}")
    
    def _ensure_tensorrt_engine(self, model_path: str) -> str:
        """Ensure TensorRT engine exists, build if needed."""
        engine_path = Path(self.config.export_dir) / f"{Path(model_path).stem}.engine"
        engine_path.parent.mkdir(parents=True, exist_ok=True)
        
        if engine_path.exists() and not self.config.auto_export:
            return str(engine_path)
            
        if self.config.auto_export:
            self.logger.info(f"Building TensorRT engine from {model_path}...")
            model = YOLO(model_path)
            model.export(format='engine', half=self.config.half, workspace=self.config.workspace_size)
            # Move to export directory if needed
            exported_path = Path(model_path).with_suffix('.engine')
            if exported_path.exists():
                engine_path.parent.mkdir(parents=True, exist_ok=True)
                exported_path.rename(engine_path)
            return str(engine_path)
        else:
            raise FileNotFoundError(f"TensorRT engine not found: {engine_path}")
    
    def _ensure_torchscript_model(self, model_path: str) -> str:
        """Ensure TorchScript model exists, export if needed."""
        if model_path.endswith('.torchscript') or model_path.endswith('.pt'):
            # Check if it's already a TorchScript model
            try:
                torch.jit.load(model_path)
                return model_path
            except:
                pass
                
        torchscript_path = Path(self.config.export_dir) / f"{Path(model_path).stem}.torchscript"
        torchscript_path.parent.mkdir(parents=True, exist_ok=True)
        
        if torchscript_path.exists() and not self.config.auto_export:
            return str(torchscript_path)
            
        if self.config.auto_export:
            self.logger.info(f"Exporting {model_path} to TorchScript...")
            model = YOLO(model_path)
            model.export(format='torchscript', optimize=self.config.optimize)
            # Move to export directory
            exported_path = Path(model_path).with_suffix('.torchscript')
            if exported_path.exists():
                torchscript_path.parent.mkdir(parents=True, exist_ok=True)
                exported_path.rename(torchscript_path)
            return str(torchscript_path)
        else:
            raise FileNotFoundError(f"TorchScript model not found: {torchscript_path}")
    
    def predict(self, image: np.ndarray) -> List:
        """
        Run inference on an image.
        
        Args:
            image: Input image as numpy array
            
        Returns:
            List of detection results
        """
        if self.backend is None:
            raise RuntimeError("No model loaded")
            
        start_time = time.time()
        
        try:
            if self.backend == 'pytorch':
                results = self.model(image)
            elif self.backend == 'onnx':
                results = self._predict_onnx(image)
            elif self.backend == 'tensorrt':
                results = self._predict_tensorrt(image)
            elif self.backend == 'torchscript':
                results = self._predict_torchscript(image)
            else:
                raise ValueError(f"Unsupported backend: {self.backend}")
                
            inference_time = time.time() - start_time
            self.inference_times.append(inference_time)
            
            if len(self.inference_times) > 100:  # Keep last 100 times
                self.inference_times = self.inference_times[-100:]
                
            return results
            
        except Exception as e:
            self.logger.error(f"Inference failed with {self.backend} backend: {e}")
            raise
    
    def _predict_onnx(self, image: np.ndarray) -> List:
        """ONNX Runtime inference."""
        # Preprocess image (similar to ultralytics preprocessing)
        input_tensor = self._preprocess_image_onnx(image)
        
        # Run inference
        outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
        
        # Post-process outputs (simplified, would need full ultralytics post-processing)
        return self._postprocess_onnx_outputs(outputs, image.shape)
    
    def _predict_tensorrt(self, image: np.ndarray) -> List:
        """TensorRT inference."""
        # Implementation would require CUDA memory management
        # This is a placeholder for the complete implementation
        raise NotImplementedError("TensorRT inference not fully implemented yet")
    
    def _predict_torchscript(self, image: np.ndarray) -> List:
        """TorchScript inference."""
        # Preprocess image
        input_tensor = self._preprocess_image_torch(image)
        
        with torch.no_grad():
            outputs = self.model(input_tensor)
            
        # Convert back to ultralytics format
        return self._postprocess_torch_outputs(outputs, image.shape)
    
    def _preprocess_image_onnx(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for ONNX model."""
        # Simplified preprocessing - full implementation would match ultralytics exactly
        import cv2
        resized = cv2.resize(image, (640, 640))
        normalized = resized.astype(np.float32) / 255.0
        transposed = np.transpose(normalized, (2, 0, 1))
        batched = np.expand_dims(transposed, axis=0)
        return batched
    
    def _preprocess_image_torch(self, image: np.ndarray) -> torch.Tensor:
        """Preprocess image for PyTorch model."""
        import cv2
        resized = cv2.resize(image, (640, 640))
        normalized = resized.astype(np.float32) / 255.0
        transposed = np.transpose(normalized, (2, 0, 1))
        tensor = torch.from_numpy(transposed).unsqueeze(0)
        if self.config.device != 'cpu' and torch.cuda.is_available():
            tensor = tensor.cuda()
        return tensor
    
    def _postprocess_onnx_outputs(self, outputs: List[np.ndarray], original_shape: tuple) -> List:
        """Post-process ONNX model outputs."""
        # Simplified post-processing - full implementation would match ultralytics
        # This is a placeholder that returns the raw outputs
        return outputs
    
    def _postprocess_torch_outputs(self, outputs: torch.Tensor, original_shape: tuple) -> List:
        """Post-process TorchScript model outputs."""
        # Simplified post-processing - full implementation would match ultralytics
        return [outputs.cpu().numpy()]
    
    def warmup(self, warmup_image: Optional[np.ndarray] = None) -> None:
        """
        Perform warmup runs for better performance measurement.
        
        Args:
            warmup_image: Image to use for warmup, creates dummy if None
        """
        if warmup_image is None:
            # Create dummy image
            warmup_image = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
            
        self.logger.info(f"Running {self.config.warmup_runs} warmup iterations...")
        
        for i in range(self.config.warmup_runs):
            try:
                self.predict(warmup_image)
            except Exception as e:
                self.logger.warning(f"Warmup iteration {i+1} failed: {e}")
                
        self.logger.info("Warmup completed")
    
    def get_performance_stats(self) -> Dict[str, float]:
        """Get performance statistics."""
        if not self.inference_times:
            return {}
            
        return {
            'backend': self.backend,
            'avg_inference_time': np.mean(self.inference_times),
            'min_inference_time': np.min(self.inference_times),
            'max_inference_time': np.max(self.inference_times),
            'total_inferences': len(self.inference_times),
            'fps': 1.0 / np.mean(self.inference_times) if self.inference_times else 0
        }
    
    def benchmark(self, test_image: np.ndarray, num_runs: int = 100) -> Dict[str, float]:
        """
        Run performance benchmark.
        
        Args:
            test_image: Image to use for benchmarking
            num_runs: Number of inference runs
            
        Returns:
            Performance statistics
        """
        self.logger.info(f"Running benchmark with {num_runs} iterations...")
        
        # Clear previous times
        self.inference_times = []
        
        # Warmup
        self.warmup(test_image)
        
        # Clear warmup times
        self.inference_times = []
        
        # Benchmark runs
        for i in range(num_runs):
            self.predict(test_image)
            
        stats = self.get_performance_stats()
        self.logger.info(f"Benchmark completed: {stats['fps']:.2f} FPS average")
        
        return stats
    
    @classmethod
    def list_available_backends(cls) -> List[str]:
        """List available inference backends."""
        available = ['pytorch']  # Always available if ultralytics is installed
        
        if ONNX_AVAILABLE:
            available.append('onnx')
        if TENSORRT_AVAILABLE:
            available.append('tensorrt')
        if TORCH_AVAILABLE:
            available.append('torchscript')
            
        return available
    
    @classmethod
    def check_backend_dependencies(cls, backend: str) -> bool:
        """Check if dependencies for a backend are available."""
        if backend == 'pytorch':
            return True
        elif backend == 'onnx':
            return ONNX_AVAILABLE
        elif backend == 'tensorrt':
            return TENSORRT_AVAILABLE
        elif backend == 'torchscript':
            return TORCH_AVAILABLE
        else:
            return False