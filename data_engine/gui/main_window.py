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
try:
    from config import get_config_manager
    from sam_processor import SAM2Processor
    from utils import create_directory_structure, export_statistics
except ImportError as e:
    print(f"Warning: Could not import local modules: {e}")
    SAM2Processor = None
    get_config_manager = None
    create_directory_structure = None
    export_statistics = None

from .center_panel import CenterPanel
from .frame_viewer import FrameViewer
from .left_panel import LeftControlPanel
from .right_panel import RightPanel
from .workers import SAMProcessor, VideoProcessor


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

        self.setup_ui()
        self.connect_signals()
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

        # SAM controls
        self.left_panel.positive_point_btn.clicked.connect(
            lambda: self.set_point_mode(1)
        )
        self.left_panel.negative_point_btn.clicked.connect(
            lambda: self.set_point_mode(0)
        )
        self.left_panel.clear_points_btn.clicked.connect(self.clear_points)
        self.left_panel.segment_btn.clicked.connect(self.segment_current_frame)
        self.left_panel.propagate_forward_btn.clicked.connect(self.propagate_forward)
        self.left_panel.propagate_backward_btn.clicked.connect(self.propagate_backward)

        # Export
        self.left_panel.export_yolo_btn.clicked.connect(self.export_yolo_dataset)

        # Frame viewer signals
        self.frame_viewer.point_clicked.connect(self.on_point_clicked)

        # Right panel signals
        self.right_panel.remove_annotation_btn.clicked.connect(self.remove_annotation)

    def load_sam_model(self):
        """Load the SAM2 model"""
        try:
            self.sam_model = SAM("sam2_b.pt")  # Use SAM2 base model
            print("SAM2 model loaded successfully")
        except Exception as e:
            QMessageBox.warning(
                self, "Model Loading Error", f"Failed to load SAM2 model: {e}"
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
        """Set point selection mode"""
        self.frame_viewer.set_click_mode(mode)

        # Update button states
        self.left_panel.positive_point_btn.setChecked(mode == 1)
        self.left_panel.negative_point_btn.setChecked(mode == 0)

    def clear_points(self):
        """Clear all points on current frame"""
        self.frame_viewer.clear_points()

    def on_point_clicked(self, x: int, y: int, label: int):
        """Handle point click on frame"""
        print(f"Point clicked: ({x}, {y}), label: {label}")

    def segment_current_frame(self):
        """Segment object in current frame using SAM2"""
        if not self.sam_model or self.current_frame_idx not in self.frame_cache:
            QMessageBox.warning(self, "Error", "No model loaded or frame not available")
            return

        if not self.frame_viewer.points:
            QMessageBox.warning(self, "Error", "Please add at least one point")
            return

        current_class_id = self.right_panel.class_manager.get_current_class_id()
        if current_class_id is None:
            QMessageBox.warning(self, "Error", "Please select a class")
            return

        # Prepare points and labels for SAM
        points = np.array([[x, y] for x, y, _ in self.frame_viewer.points])
        labels = np.array([label for _, _, label in self.frame_viewer.points])

        frame = self.frame_cache[self.current_frame_idx]

        # Run SAM segmentation
        try:
            results = self.sam_model(frame, points=points, labels=labels)

            if results and len(results) > 0:
                result = results[0]
                if hasattr(result, "masks") and result.masks is not None:
                    # Get the best mask
                    mask = result.masks.data[0].cpu().numpy()

                    # Store annotation
                    self.store_annotation(
                        self.current_frame_idx, mask, current_class_id
                    )
                    self.update_annotations_display()

                    # Cache mask
                    self.cache_mask(self.current_frame_idx, mask, current_class_id)

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

    def cache_mask(self, frame_idx: int, mask: np.ndarray, class_id: int):
        """Cache mask to disk"""
        if not self.output_dir:
            return

        cache_dir = Path(self.output_dir) / "cache" / "masks"
        cache_dir.mkdir(parents=True, exist_ok=True)

        mask_path = cache_dir / f"frame_{frame_idx:06d}_class_{class_id}.pt"
        torch.save(torch.from_numpy(mask), mask_path)

    def propagate_forward(self):
        """Propagate segmentation forward"""
        # TODO: Implement SAM2 video tracking for forward propagation
        QMessageBox.information(self, "Info", "Forward propagation not yet implemented")

    def propagate_backward(self):
        """Propagate segmentation backward"""
        # TODO: Implement SAM2 video tracking for backward propagation
        QMessageBox.information(
            self, "Info", "Backward propagation not yet implemented"
        )

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

    def export_yolo_dataset(self):
        """Export annotations in YOLO format"""
        if not self.output_dir:
            QMessageBox.warning(self, "Error", "Please set output directory first")
            return

        if not self.annotations:
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
        dataset_dir = Path(self.output_dir) / "yolo_dataset"
        images_dir = dataset_dir / "images"
        labels_dir = dataset_dir / "labels"

        images_dir.mkdir(parents=True, exist_ok=True)
        labels_dir.mkdir(parents=True, exist_ok=True)

        # Create classes.txt
        classes_file = dataset_dir / "classes.txt"
        with open(classes_file, "w") as f:
            classes = self.right_panel.class_manager.get_classes()
            for class_id in sorted(classes.keys()):
                f.write(f"{classes[class_id]}\n")

        # Export annotated frames
        for frame_idx, annotations in self.annotations.items():
            if frame_idx not in self.frame_cache:
                continue

            # Save image
            frame = self.frame_cache[frame_idx]
            image_path = images_dir / f"frame_{frame_idx:06d}.jpg"
            cv2.imwrite(str(image_path), frame)

            # Save labels
            label_path = labels_dir / f"frame_{frame_idx:06d}.txt"
            with open(label_path, "w") as f:
                for annotation in annotations:
                    # Convert mask to YOLO segmentation format
                    mask = annotation["mask"]
                    class_id = annotation["class_id"]

                    # Find contours from mask
                    mask_uint8 = (mask * 255).astype(np.uint8)
                    contours, _ = cv2.findContours(
                        mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                    )

                    if contours:
                        # Use the largest contour
                        largest_contour = max(contours, key=cv2.contourArea)

                        # Normalize coordinates
                        h, w = frame.shape[:2]
                        normalized_contour = []

                        for point in largest_contour:
                            x, y = point[0]
                            normalized_contour.extend([x / w, y / h])

                        # Write YOLO segmentation line
                        if len(normalized_contour) >= 6:  # At least 3 points
                            line = f"{class_id} " + " ".join(
                                [f"{coord:.6f}" for coord in normalized_contour]
                            )
                            f.write(line + "\n")

    def update_stats(self):
        """Update statistics display"""
        total_frames = len(self.frame_cache)
        annotated_frames = len(self.annotations)
        total_annotations = sum(len(anns) for anns in self.annotations.values())

        stats_text = f"""
Total Frames: {total_frames}
Annotated Frames: {annotated_frames}
Total Annotations: {total_annotations}
Current Frame: {self.current_frame_idx}
        """

        self.right_panel.stats_label.setText(stats_text.strip())
