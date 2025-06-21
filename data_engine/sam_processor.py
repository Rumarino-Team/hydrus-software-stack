#!/usr/bin/env python3
"""
SAM2 Integration Module
Handles SAM2 model loading, inference, and video tracking
"""

import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch

try:
    from ultralytics import SAM
except ImportError:
    print("Warning: ultralytics not installed. SAM functionality will be limited.")
    SAM = None


class SAM2Processor:
    """SAM2 model processor for image segmentation and video tracking"""

    def __init__(self, model_path: str = "sam2_b.pt", device: str = "auto"):
        """
        Initialize SAM2 processor

        Args:
            model_path: Path to SAM2 model file
            device: Device to run model on ('cpu', 'cuda', 'auto')
        """
        self.model_path = model_path
        self.device = self._get_device(device)
        self.model = None
        self.is_loaded = False

        # Video tracking state
        self.tracking_state = {}
        self.current_video_features = None

        self.load_model()

    def _get_device(self, device: str) -> str:
        """Determine the best device to use"""
        if device == "auto":
            return "cuda" if torch.cuda.is_available() else "cpu"
        return device

    def load_model(self) -> bool:
        """
        Load the SAM2 model

        Returns:
            True if model loaded successfully
        """
        try:
            if SAM is None:
                print("SAM not available - ultralytics not installed")
                return False

            self.model = SAM(self.model_path)
            self.is_loaded = True
            print(f"SAM2 model loaded successfully on {self.device}")
            return True

        except Exception as e:
            print(f"Failed to load SAM2 model: {e}")
            self.is_loaded = False
            return False

    def segment_with_points(
        self,
        image: np.ndarray,
        points: np.ndarray,
        labels: np.ndarray,
        multimask_output: bool = False,
    ) -> Dict[str, Any]:
        """
        Segment image using point prompts

        Args:
            image: Input image (RGB)
            points: Array of point coordinates [[x1, y1], [x2, y2], ...]
            labels: Array of point labels [1, 0, 1, ...] (1=positive, 0=negative)
            multimask_output: Whether to output multiple masks

        Returns:
            Dictionary containing masks and other outputs
        """
        if not self.is_loaded:
            return {"masks": None, "error": "Model not loaded"}

        try:
            # Run SAM2 prediction
            results = self.model(image, points=points, labels=labels)

            if results and len(results) > 0:
                result = results[0]

                output = {
                    "masks": None,
                    "scores": None,
                    "logits": None,
                    "success": True,
                }

                if hasattr(result, "masks") and result.masks is not None:
                    # Extract mask data
                    masks = result.masks.data.cpu().numpy()
                    output["masks"] = masks

                    # Extract scores if available
                    if hasattr(result.masks, "conf"):
                        output["scores"] = result.masks.conf.cpu().numpy()

                return output
            else:
                return {
                    "masks": None,
                    "error": "No results from model",
                    "success": False,
                }

        except Exception as e:
            return {"masks": None, "error": str(e), "success": False}

    def segment_with_box(self, image: np.ndarray, box: List[int]) -> Dict[str, Any]:
        """
        Segment image using bounding box prompt

        Args:
            image: Input image (RGB)
            box: Bounding box [x1, y1, x2, y2]

        Returns:
            Dictionary containing masks and other outputs
        """
        if not self.is_loaded:
            return {"masks": None, "error": "Model not loaded"}

        try:
            results = self.model(image, bboxes=[box])

            if results and len(results) > 0:
                result = results[0]

                output = {"masks": None, "scores": None, "success": True}

                if hasattr(result, "masks") and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                    output["masks"] = masks

                    if hasattr(result.masks, "conf"):
                        output["scores"] = result.masks.conf.cpu().numpy()

                return output
            else:
                return {
                    "masks": None,
                    "error": "No results from model",
                    "success": False,
                }

        except Exception as e:
            return {"masks": None, "error": str(e), "success": False}

    def initialize_video_tracking(self, first_frame: np.ndarray) -> bool:
        """
        Initialize video tracking with the first frame

        Args:
            first_frame: First frame of the video

        Returns:
            True if initialization successful
        """
        try:
            # This would typically encode the first frame for tracking
            # For now, we'll store the frame and prepare for tracking
            self.tracking_state = {
                "initialized": True,
                "frame_count": 0,
                "objects": {},  # object_id -> tracking_info
            }

            # In a full SAM2 implementation, this would:
            # 1. Encode the frame with the image encoder
            # 2. Store image features for efficient tracking
            # 3. Initialize memory banks for object tracking

            return True

        except Exception as e:
            print(f"Failed to initialize video tracking: {e}")
            return False

    def add_tracking_object(
        self, frame: np.ndarray, points: np.ndarray, labels: np.ndarray, object_id: int
    ) -> bool:
        """
        Add a new object to track in the video

        Args:
            frame: Current frame
            points: Point prompts
            labels: Point labels
            object_id: Unique ID for the object

        Returns:
            True if object added successfully
        """
        try:
            # Segment the object in the current frame
            result = self.segment_with_points(frame, points, labels)

            if result["success"] and result["masks"] is not None:
                # Store object information for tracking
                self.tracking_state["objects"][object_id] = {
                    "mask": result["masks"][0],  # Use the best mask
                    "last_frame": self.tracking_state["frame_count"],
                    "points": points,
                    "labels": labels,
                }
                return True

            return False

        except Exception as e:
            print(f"Failed to add tracking object: {e}")
            return False

    def propagate_masks(
        self, frames: List[np.ndarray], start_frame_idx: int, direction: str = "forward"
    ) -> Dict[int, Dict[int, np.ndarray]]:
        """
        Propagate masks across multiple frames

        Args:
            frames: List of frames to process
            start_frame_idx: Index of the starting frame
            direction: 'forward' or 'backward'

        Returns:
            Dictionary mapping frame_idx -> {object_id -> mask}
        """
        results = {}

        if not self.tracking_state.get("initialized", False):
            return results

        try:
            # For each frame, propagate masks for all tracked objects
            frame_indices = range(len(frames))
            if direction == "backward":
                frame_indices = reversed(frame_indices)

            for frame_idx in frame_indices:
                if frame_idx == start_frame_idx:
                    continue

                frame = frames[frame_idx]
                frame_results = {}

                # Propagate each tracked object
                for object_id, obj_info in self.tracking_state["objects"].items():
                    # In a full SAM2 implementation, this would:
                    # 1. Use temporal features from previous frames
                    # 2. Apply object tracking algorithms
                    # 3. Refine masks based on motion and appearance

                    # For now, we'll use a simplified approach
                    propagated_mask = self._simple_mask_propagation(
                        frame,
                        obj_info["mask"],
                        obj_info.get("points"),
                        obj_info.get("labels"),
                    )

                    if propagated_mask is not None:
                        frame_results[object_id] = propagated_mask

                if frame_results:
                    results[frame_idx] = frame_results

            return results

        except Exception as e:
            print(f"Failed to propagate masks: {e}")
            return {}

    def _simple_mask_propagation(
        self,
        frame: np.ndarray,
        reference_mask: np.ndarray,
        points: Optional[np.ndarray] = None,
        labels: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """
        Simple mask propagation using template matching or re-segmentation

        Args:
            frame: Current frame
            reference_mask: Mask from previous frame
            points: Original point prompts (optional)
            labels: Original point labels (optional)

        Returns:
            Propagated mask or None if failed
        """
        try:
            # Method 1: If we have original points, re-segment
            if points is not None and labels is not None:
                result = self.segment_with_points(frame, points, labels)
                if result["success"] and result["masks"] is not None:
                    return result["masks"][0]

            # Method 2: Use template matching or other computer vision techniques
            # This is a placeholder for more sophisticated tracking
            # In practice, SAM2 would use temporal attention and memory mechanisms

            return None

        except Exception as e:
            print(f"Simple mask propagation failed: {e}")
            return None

    def save_features_cache(
        self, cache_path: str, frame_idx: int, features: torch.Tensor
    ):
        """
        Save encoded features to cache for faster processing

        Args:
            cache_path: Path to cache directory
            frame_idx: Frame index
            features: Encoded features tensor
        """
        try:
            cache_dir = Path(cache_path)
            cache_dir.mkdir(parents=True, exist_ok=True)

            features_file = cache_dir / f"features_frame_{frame_idx:06d}.pt"
            torch.save(features, features_file)

        except Exception as e:
            print(f"Failed to save features cache: {e}")

    def load_features_cache(
        self, cache_path: str, frame_idx: int
    ) -> Optional[torch.Tensor]:
        """
        Load encoded features from cache

        Args:
            cache_path: Path to cache directory
            frame_idx: Frame index

        Returns:
            Cached features tensor or None if not found
        """
        try:
            cache_dir = Path(cache_path)
            features_file = cache_dir / f"features_frame_{frame_idx:06d}.pt"

            if features_file.exists():
                return torch.load(features_file, map_location=self.device)

            return None

        except Exception as e:
            print(f"Failed to load features cache: {e}")
            return None

    def clear_tracking_state(self):
        """Clear all tracking state"""
        self.tracking_state = {}
        self.current_video_features = None

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the loaded model

        Returns:
            Dictionary with model information
        """
        info = {
            "model_path": self.model_path,
            "device": self.device,
            "is_loaded": self.is_loaded,
            "tracking_initialized": self.tracking_state.get("initialized", False),
            "tracked_objects_count": len(self.tracking_state.get("objects", {})),
        }

        if self.model and hasattr(self.model, "model"):
            try:
                # Try to get model parameters info
                total_params = sum(p.numel() for p in self.model.model.parameters())
                info["total_parameters"] = total_params
            except:
                pass

        return info


def create_sam_processor(
    model_name: str = "sam2_b.pt", device: str = "auto"
) -> SAM2Processor:
    """
    Factory function to create SAM2 processor

    Args:
        model_name: Name of the SAM2 model
        device: Device to run on

    Returns:
        SAM2Processor instance
    """
    return SAM2Processor(model_name, device)


# Utility functions for SAM2 integration


def visualize_masks(
    image: np.ndarray,
    masks: List[np.ndarray],
    colors: Optional[List[Tuple[int, int, int]]] = None,
    alpha: float = 0.5,
) -> np.ndarray:
    """
    Visualize multiple masks on an image

    Args:
        image: Base image
        masks: List of binary masks
        colors: Colors for each mask
        alpha: Transparency

    Returns:
        Image with mask overlays
    """
    if not masks:
        return image

    result = image.copy()

    # Generate colors if not provided
    if colors is None:
        colors = []
        for i in range(len(masks)):
            # Generate distinct colors
            hue = (i * 137.5) % 360  # Golden angle approximation
            color = tuple(
                int(c)
                for c in cv2.cvtColor(np.uint8([[[hue, 255, 255]]]), cv2.COLOR_HSV2RGB)[
                    0
                ][0]
            )
            colors.append(color)

    # Apply each mask
    for mask, color in zip(masks, colors):
        if len(mask.shape) > 2:
            mask = mask[0]  # Take first mask if multiple

        mask_colored = np.zeros_like(image)
        mask_colored[mask > 0] = color

        result = cv2.addWeighted(result, 1 - alpha, mask_colored, alpha, 0)

    return result


def masks_to_rle(masks: List[np.ndarray]) -> List[Dict]:
    """
    Convert masks to Run-Length Encoding format

    Args:
        masks: List of binary masks

    Returns:
        List of RLE dictionaries
    """
    rle_list = []

    for mask in masks:
        if len(mask.shape) > 2:
            mask = mask[0]

        # Simple RLE encoding
        flat_mask = mask.flatten()
        rle = []
        current_val = flat_mask[0]
        count = 1

        for val in flat_mask[1:]:
            if val == current_val:
                count += 1
            else:
                rle.extend([count, current_val])
                current_val = val
                count = 1

        rle.extend([count, current_val])

        rle_dict = {"counts": rle, "size": list(mask.shape)}
        rle_list.append(rle_dict)

    return rle_list
