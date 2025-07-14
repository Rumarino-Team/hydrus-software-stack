"""
Main window for the SAM2 Data Engine application
"""

import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch
from PySide6.QtCore import Qt, QThreadPool
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QHBoxLayout,
    QMainWindow,
    QMessageBox,
    QSplitter,
    QWidget,
)

try:
    from ultralytics import SAM
except ImportError:
    print("Warning: ultralytics not installed. Limited functionality.")
    SAM = None

# Import local modules
from .frame_viewer import FrameViewer
from .left_panel import LeftControlPanel
from .right_panel import RightPanel
from .sam2_video_worker import SAM2VideoWorker
from .workers import VideoProcessor


class DataEngineMainWindow(QMainWindow):
    """Main window for the Data Engine application"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("SAM2 Data Engine - YOLO Dataset Generator")
        self.setGeometry(100, 100, 1400, 900)

        # Initialize variables
        self.video_path = None
        self.output_dir = None
        self.frames = []
        self.current_frame_idx = 0
        self.sam_model = None
        self.frame_cache = {}
        self.masks_cache = {}
        self.annotations = {}  # frame_idx: [annotations]

        # Thread pool for SAM processing
        self.thread_pool = QThreadPool()

        # SAM2 Video Worker for advanced tracking
        self.sam2_worker = SAM2VideoWorker()
        self.video_tracking_enabled = False
        self.tracked_objects = {}  # obj_id -> object_info
        self.next_object_id = 0
        self.propagation_results = {}  # frame_idx -> {obj_id: mask}

        self.setup_ui()
        self.connect_signals()
        self.connect_sam2_signals()
        self.load_sam_model()

    def setup_ui(self):
        """Setup the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QHBoxLayout()

        # Create panels
        self.left_panel = LeftControlPanel()
        self.frame_viewer = FrameViewer()
        self.right_panel = RightPanel()

        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.left_panel)
        splitter.addWidget(self.frame_viewer)
        splitter.addWidget(self.right_panel)
        splitter.setSizes([300, 700, 300])

        main_layout.addWidget(splitter)
        central_widget.setLayout(main_layout)

    def connect_signals(self):
        """Connect signals between components"""
        # Left panel signals
        self.left_panel.load_video_btn.clicked.connect(self.load_video)
        self.left_panel.output_dir_btn.clicked.connect(self.set_output_directory)

        # Frame navigation
        self.left_panel.frame_slider.valueChanged.connect(self.on_frame_changed)
        self.left_panel.frame_spinbox.valueChanged.connect(
            self.on_frame_spinbox_changed
        )
        self.left_panel.prev_btn.clicked.connect(self.prev_frame)
        self.left_panel.next_btn.clicked.connect(self.next_frame)

        # SAM controls (single frame)
        self.left_panel.positive_point_btn.clicked.connect(
            lambda: self.set_point_mode(1)
        )
        self.left_panel.negative_point_btn.clicked.connect(
            lambda: self.set_point_mode(0)
        )
        self.left_panel.clear_points_btn.clicked.connect(self.clear_points)
        self.left_panel.segment_btn.clicked.connect(self.segment_current_frame)

        # SAM2 Video Tracking controls
        self.left_panel.init_video_btn.clicked.connect(self.init_video_tracking)
        self.left_panel.add_object_btn.clicked.connect(self.add_tracked_object)
        self.left_panel.remove_object_btn.clicked.connect(self.remove_tracked_object)
        self.left_panel.propagate_forward_btn.clicked.connect(
            self.propagate_forward_sam2
        )
        self.left_panel.propagate_backward_btn.clicked.connect(
            self.propagate_backward_sam2
        )
        self.left_panel.clear_tracking_btn.clicked.connect(self.clear_all_tracking)
        self.left_panel.tracked_objects_list.itemSelectionChanged.connect(
            self.on_object_selection_changed
        )

        # Export
        self.left_panel.export_yolo_btn.clicked.connect(self.export_yolo_dataset)

        # Frame viewer signals
        self.frame_viewer.point_clicked.connect(self.on_point_clicked)

        # Right panel signals
        self.right_panel.remove_annotation_btn.clicked.connect(self.remove_annotation)

    def connect_sam2_signals(self):
        """Connect SAM2 worker signals"""
        self.sam2_worker.model_loaded.connect(self.on_sam2_model_loaded)
        self.sam2_worker.video_initialized.connect(self.on_video_initialized)
        self.sam2_worker.object_added.connect(self.on_object_added)
        self.sam2_worker.propagation_progress.connect(self.on_propagation_progress)
        self.sam2_worker.propagation_complete.connect(self.on_propagation_complete)
        self.sam2_worker.error_occurred.connect(self.on_sam2_error)

    def load_sam_model(self):
        """Load both SAM2 video model and fallback SAM model"""
        try:
            # Load SAM2 video model for tracking
            self.sam2_worker.load_model("sam2_hiera_base_plus.pt")

            # Load fallback SAM model for single-frame segmentation
            if SAM is not None:
                self.sam_model = SAM("sam2_b.pt")  # Use SAM2 base model
                print("Fallback SAM model loaded successfully")
        except Exception as e:
            print(f"Warning: Failed to load models: {e}")
            QMessageBox.warning(
                self,
                "Model Loading Warning",
                f"Some models failed to load: {e}\nSome features may be limited.",
            )

    def load_video(self):
        """Load a video file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Video File",
            "",
            "Video Files (*.mp4 *.avi *.mov *.mkv *.wmv);;All Files (*)",
        )

        if file_path:
            self.video_path = file_path
            self.left_panel.video_path_label.setText(f"Video: {Path(file_path).name}")
            self.load_video_frames()

            # Enable SAM2 video tracking controls
            self.left_panel.init_video_btn.setEnabled(True)
            self.left_panel.tracking_status_label.setText(
                "Video loaded. Click 'Initialize Video Tracking' to begin."
            )

    def set_output_directory(self):
        """Set the output directory for the dataset"""
        dir_path = QFileDialog.getExistingDirectory(self, "Select Output Directory")

        if dir_path:
            self.output_dir = dir_path
            self.left_panel.output_dir_label.setText(f"Output: {Path(dir_path).name}")

    def load_video_frames(self):
        """Load frames from the video"""
        if not self.video_path:
            return

        try:
            cap = cv2.VideoCapture(self.video_path)
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

            # Setup frame navigation
            self.left_panel.frame_slider.setMaximum(total_frames - 1)
            self.left_panel.frame_spinbox.setMaximum(total_frames - 1)

            # Cache first frame for immediate display
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()
            if ret:
                self.frame_cache[0] = frame
                self.frame_viewer.set_frame(frame)

            cap.release()

            # Start background frame caching
            if self.output_dir:
                self.cache_frames_async()

            self.update_stats()

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load video: {e}")

    def cache_frames_async(self):
        """Cache frames asynchronously"""
        cache_dir = Path(self.output_dir) / "cache" / "frames"
        cache_dir.mkdir(parents=True, exist_ok=True)

        # Process video frames in background
        self.video_processor = VideoProcessor(self.video_path, str(cache_dir))
        self.video_processor.frame_processed.connect(self.on_frame_cached)
        self.video_processor.progress_updated.connect(
            self.left_panel.progress_bar.setValue
        )
        self.video_processor.start()

    def on_frame_cached(self, frame_idx: int, frame: np.ndarray):
        """Handle cached frame"""
        self.frame_cache[frame_idx] = frame

    def on_frame_changed(self, frame_idx: int):
        """Handle frame slider change"""
        self.current_frame_idx = frame_idx
        self.left_panel.frame_spinbox.setValue(frame_idx)
        self.load_current_frame()
        self.update_annotations_display()

    def on_frame_spinbox_changed(self, frame_idx: int):
        """Handle frame spinbox change"""
        self.left_panel.frame_slider.setValue(frame_idx)

    def load_current_frame(self):
        """Load and display the current frame"""
        if self.current_frame_idx in self.frame_cache:
            frame = self.frame_cache[self.current_frame_idx]
            self.frame_viewer.set_frame(frame)

            # Clear existing masks and load masks for this frame
            self.frame_viewer.clear_masks()

            # Show manual annotations
            if self.current_frame_idx in self.annotations:
                for annotation in self.annotations[self.current_frame_idx]:
                    mask = annotation["mask"]
                    class_id = annotation["class_id"]
                    color = self.get_class_color(class_id)
                    self.frame_viewer.add_mask(mask, color=color, alpha=0.4)

            # Show SAM2 tracking results
            if self.current_frame_idx in self.propagation_results:
                for obj_id, mask in self.propagation_results[
                    self.current_frame_idx
                ].items():
                    if obj_id in self.tracked_objects:
                        class_id = self.tracked_objects[obj_id]["class_id"]
                        color = self.get_class_color(class_id)
                        self.frame_viewer.add_mask(mask, color=color, alpha=0.3)

            # Show individual tracked object masks
            for obj_id, obj_info in self.tracked_objects.items():
                if self.current_frame_idx in obj_info["masks"]:
                    mask = obj_info["masks"][self.current_frame_idx]
                    class_id = obj_info["class_id"]
                    color = self.get_class_color(class_id)
                    self.frame_viewer.add_mask(mask, color=color, alpha=0.3)
        else:
            # Load frame from video if not cached
            self.load_frame_from_video(self.current_frame_idx)

    def load_frame_from_video(self, frame_idx: int):
        """Load a specific frame from video"""
        if not self.video_path:
            return

        try:
            cap = cv2.VideoCapture(self.video_path)
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
            ret, frame = cap.read()
            cap.release()

            if ret:
                self.frame_cache[frame_idx] = frame
                self.frame_viewer.set_frame(frame)
        except Exception as e:
            print(f"Error loading frame {frame_idx}: {e}")

    def export_yolo_dataset(self):
        """Export annotations in YOLO format"""
        if not self.output_dir:
            QMessageBox.warning(self, "Error", "Please set output directory first")
            return

        if not self.annotations and not self.tracked_objects:
            QMessageBox.warning(self, "Error", "No annotations to export")
            return

        try:
            self.create_yolo_dataset()
            QMessageBox.information(
                self, "Success", "YOLO dataset exported successfully"
            )
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export dataset: {e}")

    def create_yolo_dataset(self):
        """Create YOLO format dataset"""
        if not self.output_dir:
            raise ValueError("Output directory not set")

        dataset_dir = Path(self.output_dir) / "yolo_dataset"
        images_dir = dataset_dir / "images"
        labels_dir = dataset_dir / "labels"

        # Create directories
        images_dir.mkdir(parents=True, exist_ok=True)
        labels_dir.mkdir(parents=True, exist_ok=True)

        # Export frames and create YOLO annotations
        all_frames = set(self.annotations.keys())
        all_frames.update(
            self.tracked_objects.get(obj_id, {}).get("masks", {}).keys()
            for obj_id in self.tracked_objects
        )

        for frame_idx in all_frames:
            if frame_idx in self.frame_cache:
                frame = self.frame_cache[frame_idx]

                # Save image
                image_filename = f"frame_{frame_idx:06d}.jpg"
                image_path = images_dir / image_filename
                cv2.imwrite(str(image_path), frame)

                # Create YOLO label file
                label_filename = f"frame_{frame_idx:06d}.txt"
                label_path = labels_dir / label_filename

                with open(label_path, "w") as f:
                    # Write manual annotations
                    if frame_idx in self.annotations:
                        for annotation in self.annotations[frame_idx]:
                            mask = annotation["mask"]
                            class_id = annotation["class_id"]
                            bbox = self.mask_to_yolo_bbox(mask, frame.shape[:2])
                            f.write(f"{class_id} {' '.join(map(str, bbox))}\n")

                    # Write tracked object annotations
                    for obj_id, obj_info in self.tracked_objects.items():
                        if frame_idx in obj_info["masks"]:
                            mask = obj_info["masks"][frame_idx]
                            class_id = obj_info["class_id"]
                            bbox = self.mask_to_yolo_bbox(mask, frame.shape[:2])
                            f.write(f"{class_id} {' '.join(map(str, bbox))}\n")

        # Create classes.txt file
        classes_path = dataset_dir / "classes.txt"
        with open(classes_path, "w") as f:
            for class_id, class_name in self.right_panel.class_manager.classes.items():
                f.write(f"{class_name}\n")

        print(f"YOLO dataset exported to {dataset_dir}")

    def mask_to_yolo_bbox(self, mask: np.ndarray, image_shape: tuple) -> list:
        """Convert mask to YOLO format bounding box (normalized)"""
        # Find bounding box of mask
        rows = np.any(mask, axis=1)
        cols = np.any(mask, axis=0)

        if not rows.any() or not cols.any():
            return [0.5, 0.5, 0.0, 0.0]  # Default small box in center

        rmin, rmax = np.where(rows)[0][[0, -1]]
        cmin, cmax = np.where(cols)[0][[0, -1]]

        # Convert to YOLO format (center_x, center_y, width, height) - normalized
        h, w = image_shape
        center_x = (cmin + cmax) / 2 / w
        center_y = (rmin + rmax) / 2 / h
        width = (cmax - cmin) / w
        height = (rmax - rmin) / h

        return [center_x, center_y, width, height]

    def prev_frame(self):
        """Go to previous frame"""
        if self.current_frame_idx > 0:
            self.left_panel.frame_slider.setValue(self.current_frame_idx - 1)

    def next_frame(self):
        """Go to next frame"""
        max_frame = self.left_panel.frame_slider.maximum()
        if self.current_frame_idx < max_frame:
            self.left_panel.frame_slider.setValue(self.current_frame_idx + 1)

    def set_point_mode(self, mode: int):
        """Set point mode (0=negative, 1=positive)"""
        self.frame_viewer.point_mode = mode

        # Update button states
        self.left_panel.positive_point_btn.setChecked(mode == 1)
        self.left_panel.negative_point_btn.setChecked(mode == 0)

    def clear_points(self):
        """Clear all points on current frame"""
        self.frame_viewer.clear_points()

    def segment_current_frame(self):
        """Segment current frame using SAM"""
        if not self.frame_viewer.points:
            QMessageBox.warning(
                self, "Error", "Please click points on the object first"
            )
            return

        if self.sam_model is None:
            QMessageBox.warning(self, "Error", "SAM model not loaded")
            return

        current_class_id = self.right_panel.class_manager.get_current_class_id()
        if current_class_id is None:
            QMessageBox.warning(self, "Error", "Please select a class")
            return

        try:
            # Get current frame
            frame = self.frame_cache.get(self.current_frame_idx)
            if frame is None:
                QMessageBox.warning(self, "Error", "Frame not loaded")
                return

            # Prepare points for SAM
            points = np.array([[x, y] for x, y, _ in self.frame_viewer.points])
            labels = np.array([label for _, _, label in self.frame_viewer.points])

            # Run SAM segmentation
            results = self.sam_model(frame, points=points, labels=labels)

            if results and len(results) > 0:
                masks = results[0].masks
                if masks is not None:
                    # Get the best mask
                    mask = masks.data[0].cpu().numpy().astype(bool)

                    # Store annotation
                    self.store_annotation(
                        self.current_frame_idx, mask, current_class_id
                    )

                    # Update display
                    color = self.get_class_color(current_class_id)
                    self.frame_viewer.add_mask(mask, color=color, alpha=0.4)
                    self.update_annotations_display()

                    # Clear points
                    self.frame_viewer.clear_points()

                    print(f"Segmentation completed for frame {self.current_frame_idx}")
                else:
                    QMessageBox.warning(self, "Error", "No mask generated")
            else:
                QMessageBox.warning(self, "Error", "Segmentation failed")

        except Exception as e:
            QMessageBox.critical(self, "Segmentation Error", f"Failed to segment: {e}")

    def store_annotation(self, frame_idx: int, mask: np.ndarray, class_id: int):
        """Store annotation for a frame"""
        if frame_idx not in self.annotations:
            self.annotations[frame_idx] = []

        annotation = {
            "mask": mask,
            "class_id": class_id,
            "class_name": self.right_panel.class_manager.classes.get(
                class_id, "unknown"
            ),
        }

        self.annotations[frame_idx].append(annotation)

    def update_annotations_display(self):
        """Update the annotations list for current frame"""
        self.right_panel.annotations_list.clear()

        if self.current_frame_idx in self.annotations:
            for i, annotation in enumerate(self.annotations[self.current_frame_idx]):
                class_name = annotation["class_name"]
                item_text = f"Object {i}: {class_name}"
                self.right_panel.annotations_list.addItem(item_text)

    def remove_annotation(self):
        """Remove selected annotation"""
        current_item = self.right_panel.annotations_list.currentItem()
        if current_item and self.current_frame_idx in self.annotations:
            row = self.right_panel.annotations_list.row(current_item)
            if 0 <= row < len(self.annotations[self.current_frame_idx]):
                del self.annotations[self.current_frame_idx][row]
                self.update_annotations_display()
                self.load_current_frame()  # Refresh display

    def get_class_color(self, class_id: int) -> tuple:
        """Get color for a class ID"""
        colors = [
            (255, 0, 0),  # Red
            (0, 255, 0),  # Green
            (0, 0, 255),  # Blue
            (255, 255, 0),  # Yellow
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Cyan
            (255, 128, 0),  # Orange
            (128, 0, 255),  # Purple
        ]
        return colors[class_id % len(colors)]

    def update_stats(self):
        """Update statistics display"""
        total_frames = (
            self.left_panel.frame_slider.maximum() + 1 if self.video_path else 0
        )
        annotated_frames = len(self.annotations)
        tracked_frames = len(
            set().union(
                *(
                    obj_info["masks"].keys()
                    for obj_info in self.tracked_objects.values()
                )
            )
        )

        stats_text = f"Total: {total_frames} frames | Annotated: {annotated_frames} | Tracked: {tracked_frames}"
        # You can display this in a status bar or label if needed
        print(stats_text)

    def init_video_tracking(self):
        """Initialize video for SAM2 tracking"""
        if not self.video_path:
            QMessageBox.warning(self, "Error", "Please load a video first")
            return

        self.left_panel.tracking_status_label.setText("Initializing video tracking...")
        self.left_panel.tracking_progress.setVisible(True)
        self.sam2_worker.init_video(self.video_path)

    def add_tracked_object(self):
        """Add a new object to track using current points"""
        if not self.video_tracking_enabled:
            QMessageBox.warning(
                self,
                "Error",
                "Video tracking not initialized. Click 'Initialize Video Tracking' first.",
            )
            return

        if not self.frame_viewer.points:
            QMessageBox.warning(
                self, "Error", "Please click points on the object to track"
            )
            return

        current_class_id = self.right_panel.class_manager.get_current_class_id()
        if current_class_id is None:
            QMessageBox.warning(self, "Error", "Please select a class")
            return

        # Prepare points for SAM2
        points = [[x, y] for x, y, _ in self.frame_viewer.points]
        labels = [label for _, _, label in self.frame_viewer.points]

        # Create new object ID
        obj_id = self.next_object_id
        self.next_object_id += 1

        # Store object info
        self.tracked_objects[obj_id] = {
            "class_id": current_class_id,
            "class_name": self.right_panel.class_manager.classes.get(
                current_class_id, "unknown"
            ),
            "masks": {},
        }

        # Add object to SAM2 tracking
        self.sam2_worker.add_object(
            frame_idx=self.current_frame_idx,
            points=points,
            labels=labels,
            obj_id=obj_id,
        )

    def remove_tracked_object(self):
        """Remove selected object from tracking"""
        if not hasattr(self.left_panel, "tracked_objects_list"):
            return

        current_item = self.left_panel.tracked_objects_list.currentItem()
        if current_item:
            # Extract object ID from item text
            item_text = current_item.text()
            try:
                obj_id = int(item_text.split(":")[0].split()[-1])

                # Remove from SAM2
                self.sam2_worker.clear_object(obj_id)

                # Remove from local storage
                if obj_id in self.tracked_objects:
                    del self.tracked_objects[obj_id]

                # Remove from propagation results
                for frame_results in self.propagation_results.values():
                    if obj_id in frame_results:
                        del frame_results[obj_id]

                # Remove from list
                row = self.left_panel.tracked_objects_list.row(current_item)
                self.left_panel.tracked_objects_list.takeItem(row)

                # Refresh display
                self.load_current_frame()

                QMessageBox.information(self, "Success", f"Object {obj_id} removed")

            except (ValueError, IndexError):
                QMessageBox.warning(self, "Error", "Could not parse object ID")

    def propagate_forward_sam2(self):
        """Propagate tracking forward using SAM2"""
        if not self.video_tracking_enabled or not self.tracked_objects:
            QMessageBox.warning(
                self, "Error", "No tracking initialized or objects to track"
            )
            return

        self.left_panel.tracking_progress.setVisible(True)
        self.sam2_worker.propagate_video(
            start_frame=self.current_frame_idx, reverse=False
        )

    def propagate_backward_sam2(self):
        """Propagate tracking backward using SAM2"""
        if not self.video_tracking_enabled or not self.tracked_objects:
            QMessageBox.warning(
                self, "Error", "No tracking initialized or objects to track"
            )
            return

        self.left_panel.tracking_progress.setVisible(True)
        self.sam2_worker.propagate_video(
            start_frame=self.current_frame_idx, reverse=True
        )

    def clear_all_tracking(self):
        """Clear all SAM2 tracking"""
        if self.video_tracking_enabled:
            self.sam2_worker.reset_tracking()
            self.tracked_objects.clear()
            self.propagation_results.clear()
            self.next_object_id = 0

            # Clear tracked objects list
            if hasattr(self.left_panel, "tracked_objects_list"):
                self.left_panel.tracked_objects_list.clear()

            # Refresh display
            self.load_current_frame()

            self.left_panel.tracking_status_label.setText("All tracking cleared")
            QMessageBox.information(self, "Success", "All tracking cleared")

    def on_object_selection_changed(self):
        """Handle tracked object selection change"""
        # Enable/disable remove button based on selection
        has_selection = self.left_panel.tracked_objects_list.currentItem() is not None
        self.left_panel.remove_object_btn.setEnabled(has_selection)

    def on_point_clicked(self, x: int, y: int, label: int):
        """Handle point clicked in frame viewer"""
        # Add point to frame viewer
        self.frame_viewer.add_point(x, y, label)

    def on_sam2_model_loaded(self, success: bool, message: str):
        """Handle SAM2 model loading completion"""
        if success:
            self.left_panel.tracking_status_label.setText(
                "SAM2 model loaded successfully"
            )
            print("SAM2 model loaded successfully")
        else:
            self.left_panel.tracking_status_label.setText(
                f"SAM2 model loading failed: {message}"
            )
            QMessageBox.critical(
                self, "Model Loading Error", f"Failed to load SAM2 model: {message}"
            )

    def on_video_initialized(self, success: bool, message: str):
        """Handle video initialization completion"""
        self.left_panel.tracking_progress.setVisible(False)

        if success:
            self.video_tracking_enabled = True
            self.left_panel.tracking_status_label.setText("Video tracking initialized")

            # Enable tracking controls
            self.left_panel.add_object_btn.setEnabled(True)
            self.left_panel.clear_tracking_btn.setEnabled(True)

            print("Video tracking initialized successfully")
        else:
            self.left_panel.tracking_status_label.setText(
                f"Video initialization failed: {message}"
            )
            QMessageBox.critical(
                self,
                "Video Initialization Error",
                f"Failed to initialize video: {message}",
            )

    def on_object_added(self, obj_id: int, object_info: dict):
        """Handle object addition completion"""
        try:
            frame_idx = object_info["frame_idx"]
            mask = object_info["mask"]

            # Store the mask for this object
            if obj_id in self.tracked_objects:
                self.tracked_objects[obj_id]["masks"][frame_idx] = mask

                # Add to tracked objects list
                class_name = self.tracked_objects[obj_id]["class_name"]
                item_text = f"Object {obj_id}: {class_name}"
                self.left_panel.tracked_objects_list.addItem(item_text)

                # Enable propagation controls
                self.left_panel.propagate_forward_btn.setEnabled(True)
                self.left_panel.propagate_backward_btn.setEnabled(True)

                # Clear points and refresh display
                self.frame_viewer.clear_points()
                self.load_current_frame()

                self.left_panel.tracking_status_label.setText(
                    f"Object {obj_id} added successfully"
                )
                print(f"Object {obj_id} added successfully to frame {frame_idx}")
            else:
                print(f"Warning: Object {obj_id} is missing from tracked_objects")

        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to process added object: {e}")
            print(f"Error in on_object_added: {e}")

    def on_propagation_progress(self, frame_idx: int, progress: int):
        """Handle propagation progress updates"""
        self.left_panel.tracking_progress.setValue(progress)
        self.left_panel.tracking_status_label.setText(
            f"Propagating... Frame {frame_idx} ({progress}%)"
        )

    def on_propagation_complete(self, results):
        """Handle propagation completion"""
        self.left_panel.tracking_progress.setVisible(False)

        # Store propagation results
        self.propagation_results.update(results)

        # Also store in tracked objects
        for frame_idx, frame_results in results.items():
            for obj_id, mask in frame_results.items():
                if obj_id in self.tracked_objects:
                    self.tracked_objects[obj_id]["masks"][frame_idx] = mask

        # Refresh current frame display
        self.load_current_frame()

        num_frames = len(results)
        num_objects = len(
            set(
                obj_id
                for frame_results in results.values()
                for obj_id in frame_results.keys()
            )
        )

        self.left_panel.tracking_status_label.setText(
            f"Propagation complete: {num_frames} frames, {num_objects} objects"
        )
        print(f"Propagation complete: {num_frames} frames processed")

    def on_sam2_error(self, error_message: str):
        """Handle SAM2 worker errors"""
        self.left_panel.tracking_progress.setVisible(False)
        self.left_panel.tracking_status_label.setText(f"Error: {error_message}")
        QMessageBox.critical(
            self, "SAM2 Error", f"SAM2 operation failed: {error_message}"
        )
        print(f"SAM2 Error: {error_message}")
