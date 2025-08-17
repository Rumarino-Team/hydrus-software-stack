"""
SAM2 Video Worker for background video tracking operations
"""

import os
import sys
from pathlib import Path

import numpy as np
import torch
from PySide6.QtCore import QThread, Signal

# Add SAM2 to path
sys.path.append("/app/third_party/sam2")
sys.path.append("/app/third_party/sam2/sam2")

try:
    from sam2.build_sam import build_sam2_video_predictor
    from sam2.sam2_video_predictor import SAM2VideoPredictor

    SAM2_AVAILABLE = True
except ImportError as e:
    print(f"SAM2 not available: {e}")
    SAM2VideoPredictor = None
    SAM2_AVAILABLE = False


class SAM2VideoWorker(QThread):
    """Worker for SAM2 video tracking operations"""

    # Signals
    video_initialized = Signal(bool, str)  # success, message
    object_added = Signal(int, dict)  # obj_id, object_info
    propagation_progress = Signal(int, int, str)  # current_frame, total_frames, message
    propagation_complete = Signal(dict)  # frame_idx -> {obj_id: mask}
    error_occurred = Signal(str)
    model_loaded = Signal(bool, str)  # success, message

    def __init__(self):
        super().__init__()
        self.predictor = None
        self.inference_state = None
        self.video_path = None
        self.current_task = None
        self.task_data = None
        self.is_running = False

    def load_model(self, model_name="sam2_hiera_base_plus.pt"):
        """Load SAM2 video predictor model"""
        self.current_task = "load_model"
        self.task_data = {"model_name": model_name}
        self.start()

    def init_video(self, video_path):
        """Initialize video for tracking"""
        self.video_path = video_path
        self.current_task = "init_video"
        self.start()

    def add_object(self, frame_idx, points, labels, obj_id):
        """Add new object for tracking"""
        self.current_task = "add_object"
        self.task_data = {
            "frame_idx": frame_idx,
            "points": points,
            "labels": labels,
            "obj_id": obj_id,
        }
        self.start()

    def propagate_video(self, start_frame=None, reverse=False, max_frames=None):
        """Propagate tracking through video"""
        self.current_task = "propagate"
        self.task_data = {
            "start_frame": start_frame,
            "reverse": reverse,
            "max_frames": max_frames,
        }
        self.start()

    def stop_current_task(self):
        """Stop the current running task"""
        self.is_running = False

    def run(self) -> None:
        """Execute the current task"""
        self.is_running = True
        try:
            if self.current_task == "load_model":
                self._load_model()
            elif self.current_task == "init_video":
                self._init_video()
            elif self.current_task == "add_object":
                self._add_object()
            elif self.current_task == "propagate":
                self._propagate()
        except Exception as e:
            self.error_occurred.emit(f"Error in {self.current_task}: {str(e)}")
        finally:
            self.is_running = False

    def _load_model(self):
        """Load SAM2 video predictor model"""
        if not SAM2_AVAILABLE:
            self.model_loaded.emit(False, "SAM2 not available")
            return

        try:
            model_name = self.task_data.get("model_name", "sam2_hiera_base_plus.pt")

            # Try different possible locations for the model
            possible_paths = [
                f"/app/sam2_models/{model_name}",
                f"/app/models/{model_name}",
                f"/app/third_party/sam2/checkpoints/{model_name}",
                model_name,  # Let SAM2 handle download
            ]

            model_path = None
            for path in possible_paths:
                if os.path.exists(path):
                    model_path = path
                    break

            if model_path is None:
                model_path = model_name  # Let SAM2 handle it

            # Determine config based on model name
            if "tiny" in model_name:
                config = "sam2_hiera_t.yaml"
            elif "small" in model_name:
                config = "sam2_hiera_s.yaml"
            elif "base_plus" in model_name:
                config = "sam2_hiera_b+.yaml"
            elif "large" in model_name:
                config = "sam2_hiera_l.yaml"
            else:
                config = "sam2_hiera_b+.yaml"  # default

            # Try to find config file
            config_path = f"/app/third_party/sam2/sam2/configs/{config}"
            if not os.path.exists(config_path):
                # Fallback to default config
                config_path = config

            self.predictor = build_sam2_video_predictor(config_path, model_path)
            self.model_loaded.emit(True, f"SAM2 model {model_name} loaded successfully")

        except Exception as e:
            self.model_loaded.emit(False, f"Failed to load SAM2 model: {e}")

    def _init_video(self):
        """Initialize video in SAM2"""
        if not self.predictor or not self.video_path:
            self.video_initialized.emit(False, "Model or video path not set")
            return

        try:
            self.inference_state = self.predictor.init_state(
                video_path=self.video_path,
                offload_video_to_cpu=True,  # Save GPU memory
                offload_state_to_cpu=False,
            )

            num_frames = self.inference_state.get("num_frames", 0)
            self.video_initialized.emit(
                True, f"Video initialized with {num_frames} frames"
            )

        except Exception as e:
            self.video_initialized.emit(False, f"Failed to initialize video: {e}")

    def _add_object(self):
        """Add object to tracking"""
        if not self.predictor or not self.inference_state:
            self.error_occurred.emit("SAM2 not initialized")
            return

        data = self.task_data
        try:
            # Convert numpy arrays to proper format
            points = np.array(data["points"], dtype=np.float32)
            labels = np.array(data["labels"], dtype=np.int32)

            frame_idx, obj_ids, video_res_masks = self.predictor.add_new_points_or_box(
                inference_state=self.inference_state,
                frame_idx=data["frame_idx"],
                obj_id=data["obj_id"],
                points=points,
                labels=labels,
                normalize_coords=True,
            )

            # Extract mask for the new object
            obj_idx = obj_ids.index(data["obj_id"])
            mask = video_res_masks[obj_idx].cpu().numpy()

            print(
                f"SAM2 mask shape: {mask.shape}, dtype: {mask.dtype}, min: {mask.min()}, max: {mask.max()}"
            )

            # Ensure mask is in the correct format (2D boolean/binary)
            if mask.ndim > 2:
                mask = mask.squeeze()
            if mask.ndim != 2:
                raise ValueError(f"Mask has invalid dimensions: {mask.shape}")

            # Convert to binary mask if needed
            if mask.dtype == np.bool_:
                # Already boolean, convert to uint8 for consistency
                mask = mask.astype(np.uint8) * 255
            elif mask.max() <= 1.0:
                # Floating point mask, threshold and convert
                mask = (mask > 0.5).astype(np.uint8) * 255
            else:
                # Already in uint8 range
                mask = mask.astype(np.uint8)

            object_info = {
                "obj_id": data["obj_id"],
                "frame_idx": frame_idx,
                "mask": mask,
            }
            self.object_added.emit(data["obj_id"], object_info)

        except Exception as e:
            self.error_occurred.emit(f"Failed to add object: {e}")

    def _propagate(self):
        """Propagate tracking through video"""
        if not self.predictor or not self.inference_state:
            self.error_occurred.emit("SAM2 not initialized")
            return

        data = self.task_data
        all_results = {}

        try:
            total_frames = self.inference_state["num_frames"]
            current_frame = 0

            for (
                frame_idx,
                obj_ids,
                video_res_masks,
            ) in self.predictor.propagate_in_video(
                self.inference_state,
                start_frame_idx=data.get("start_frame"),
                max_frame_num_to_track=data.get("max_frames"),
                reverse=data.get("reverse", False),
            ):
                if not self.is_running:
                    break

                # Convert masks to numpy and organize by object
                frame_results = {}
                for i, obj_id in enumerate(obj_ids):
                    mask = video_res_masks[i].cpu().numpy()

                    # Ensure mask is in the correct format (2D boolean/binary)
                    if mask.ndim > 2:
                        mask = mask.squeeze()
                    if mask.ndim != 2:
                        print(
                            f"Warning: Mask for obj {obj_id} has invalid dimensions: {mask.shape}"
                        )
                        continue

                    # Convert to binary mask if needed
                    if mask.dtype == np.bool_:
                        # Already boolean, convert to uint8 for consistency
                        mask = mask.astype(np.uint8) * 255
                    elif mask.max() <= 1.0:
                        # Floating point mask, threshold and convert
                        mask = (mask > 0.5).astype(np.uint8) * 255
                    else:
                        # Already in uint8 range
                        mask = mask.astype(np.uint8)

                    frame_results[obj_id] = mask

                all_results[frame_idx] = frame_results
                current_frame += 1

                # Emit progress
                progress_msg = f"Processing frame {frame_idx}/{total_frames}"
                self.propagation_progress.emit(
                    current_frame, total_frames, progress_msg
                )

            if self.is_running:
                self.propagation_complete.emit(all_results)

        except Exception as e:
            self.error_occurred.emit(f"Failed to propagate: {e}")

    def clear_object(self, obj_id):
        """Clear/remove an object from tracking"""
        if not self.predictor or not self.inference_state:
            return

        try:
            self.predictor.remove_object(self.inference_state, obj_id)
        except Exception as e:
            self.error_occurred.emit(f"Failed to remove object {obj_id}: {e}")

    def reset_tracking(self):
        """Reset all tracking"""
        if not self.predictor or not self.inference_state:
            return

        try:
            self.predictor.reset_state(self.inference_state)
        except Exception as e:
            self.error_occurred.emit(f"Failed to reset tracking: {e}")
