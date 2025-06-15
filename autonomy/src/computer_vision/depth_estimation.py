#!/usr/bin/env python3
"""
Depth estimation module using Depth-Anything-ONNX
"""

import logging
import os
import sys
from typing import Optional, Tuple

import cv2
import numpy as np

# Add the third_party Depth-Anything-ONNX to Python path
HYDRUS_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
)
DEPTH_ANYTHING_PATH = os.path.join(HYDRUS_ROOT, "third_party", "Depth-Anything-ONNX")
sys.path.insert(0, DEPTH_ANYTHING_PATH)

# Optional dependency imports with graceful fallback
try:
    import onnxruntime as ort

    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False
    logging.warning(
        "ONNX Runtime not available. Install with: pip install -e .[depth-estimation]"
    )

try:
    from depth_anything_v2.dpt import DepthAnythingV2

    DEPTH_ANYTHING_AVAILABLE = True
except ImportError:
    DEPTH_ANYTHING_AVAILABLE = False
    logging.warning(
        "Depth-Anything-V2 not available. Install with: pip install -e .[depth-estimation]"
    )

try:
    import torch

    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class DepthEstimator:
    """
    Depth estimation using Depth-Anything V2 ONNX model
    """

    def __init__(self, model_path: Optional[str] = None, encoder: str = "vitb"):
        """
        Initialize the depth estimator

        Args:
            model_path: Path to ONNX model file. If None, will look for default path
            encoder: Model encoder type ('vits', 'vitb', 'vitl')

        Raises:
            ImportError: If required dependencies are not installed
        """
        if not ONNX_AVAILABLE:
            raise ImportError(
                "ONNX Runtime is required for depth estimation. "
                "Install with: pip install -e .[depth-estimation]"
            )

        self.encoder = encoder
        self.input_size = 518

        # Set default model path if not provided
        if model_path is None:
            weights_dir = os.path.join(DEPTH_ANYTHING_PATH, "weights")
            model_path = os.path.join(weights_dir, f"depth_anything_v2_{encoder}.onnx")

        self.model_path = model_path
        self.session = None
        self._load_model()

    def _load_model(self):
        """Load the ONNX model"""
        try:
            # Check if model file exists
            if not os.path.exists(self.model_path):
                logging.warning(f"Model file not found at {self.model_path}")
                logging.info("You may need to download or export the model first.")
                logging.info(
                    f"Run: cd {DEPTH_ANYTHING_PATH} && python dynamo.py export --encoder {self.encoder}"
                )
                return

            # Create ONNX Runtime session
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            self.session = ort.InferenceSession(self.model_path, providers=providers)

            # Get input/output info
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name

            logging.info(f"Loaded depth estimation model: {self.model_path}")
            logging.info(f"Using provider: {self.session.get_providers()[0]}")

        except Exception as e:
            logging.error(f"Failed to load ONNX model: {e}")
            self.session = None

    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """
        Preprocess image for depth estimation

        Args:
            image: Input image (BGR format)

        Returns:
            Preprocessed image tensor
        """
        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Resize to model input size
        resized = cv2.resize(rgb_image, (self.input_size, self.input_size))

        # Normalize to [0, 1] and convert to CHW format
        normalized = resized.astype(np.float32) / 255.0
        tensor = np.transpose(normalized, (2, 0, 1))  # HWC to CHW

        # Add batch dimension
        batch_tensor = np.expand_dims(tensor, axis=0)

        return batch_tensor

    def _postprocess_depth(
        self, depth_output: np.ndarray, original_shape: Tuple[int, int]
    ) -> np.ndarray:
        """
        Postprocess depth map output

        Args:
            depth_output: Raw depth output from model
            original_shape: (height, width) of original image

        Returns:
            Processed depth map
        """
        # Remove batch dimension and get depth map
        depth_map = depth_output.squeeze()

        # Resize to original image size
        h, w = original_shape
        depth_resized = cv2.resize(depth_map, (w, h))

        # Normalize depth values to [0, 255] for visualization
        depth_normalized = (
            (depth_resized - depth_resized.min())
            / (depth_resized.max() - depth_resized.min())
            * 255
        ).astype(np.uint8)

        return depth_normalized

    def estimate_depth(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate depth from input image

        Args:
            image: Input image (BGR format)

        Returns:
            Tuple of (raw_depth_map, visualization_depth_map)
            raw_depth_map: Float32 depth values
            visualization_depth_map: Uint8 normalized depth for visualization
        """
        if self.session is None:
            raise RuntimeError("Model not loaded. Cannot perform depth estimation.")

        # Store original shape
        original_shape = image.shape[:2]

        # Preprocess image
        input_tensor = self._preprocess_image(image)

        # Run inference
        try:
            depth_output = self.session.run(
                [self.output_name], {self.input_name: input_tensor}
            )[0]
        except Exception as e:
            logging.error(f"Inference failed: {e}")
            raise

        # Postprocess
        raw_depth = depth_output.squeeze()
        raw_depth_resized = cv2.resize(raw_depth, (image.shape[1], image.shape[0]))
        viz_depth = self._postprocess_depth(depth_output, original_shape)

        return raw_depth_resized, viz_depth

    def get_depth_at_point(self, depth_map: np.ndarray, x: int, y: int) -> float:
        """
        Get depth value at specific pixel coordinates

        Args:
            depth_map: Raw depth map from estimate_depth
            x, y: Pixel coordinates

        Returns:
            Depth value at (x, y)
        """
        if 0 <= y < depth_map.shape[0] and 0 <= x < depth_map.shape[1]:
            return float(depth_map[y, x])
        else:
            return 0.0

    def create_colored_depth_map(self, depth_map: np.ndarray) -> np.ndarray:
        """
        Create a colored depth map for better visualization

        Args:
            depth_map: Normalized depth map (uint8)

        Returns:
            Colored depth map (BGR format)
        """
        # Apply colormap
        colored_depth = cv2.applyColorMap(depth_map, cv2.COLORMAP_JET)
        return colored_depth

    @staticmethod
    def check_dependencies() -> bool:
        """
        Check if all required dependencies are available

        Returns:
            True if all dependencies are available, False otherwise
        """
        missing_deps = []

        if not ONNX_AVAILABLE:
            missing_deps.append("onnxruntime-gpu")
        if not TORCH_AVAILABLE:
            missing_deps.append("torch")
        if not DEPTH_ANYTHING_AVAILABLE:
            missing_deps.append("depth_anything_v2")

        if missing_deps:
            logging.error(f"Missing dependencies: {', '.join(missing_deps)}")
            logging.error("Install with: pip install -e .[depth-estimation]")
            return False

        return True


def main():
    """
    Example usage of the depth estimator
    """
    # Check dependencies first
    if not DepthEstimator.check_dependencies():
        logging.error("Cannot run depth estimation without required dependencies")
        sys.exit(1)

    # Initialize depth estimator
    estimator = DepthEstimator(encoder="vitb")

    # Example with webcam or test image
    cap = cv2.VideoCapture(0)  # Use webcam

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        try:
            # Estimate depth
            raw_depth, viz_depth = estimator.estimate_depth(frame)

            # Create colored depth map
            colored_depth = estimator.create_colored_depth_map(viz_depth)

            # Display results
            cv2.imshow("Original", frame)
            cv2.imshow("Depth Map", colored_depth)

            # Print depth at center point
            h, w = frame.shape[:2]
            center_depth = estimator.get_depth_at_point(raw_depth, w // 2, h // 2)
            print(f"Depth at center: {center_depth:.3f}")

        except Exception as e:
            logging.error(f"Error processing frame: {e}")

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
