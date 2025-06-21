#!/usr/bin/env python3
"""
Data Engine GUI Application
An automatic data engine that uses visual prompts to interact with AI models
for generating YOLO training datasets using SAM2 segmentation.
"""

import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch
from PIL import Image
from PySide6.QtCore import QRunnable, Qt, QThread, QThreadPool, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSlider,
    QSpinBox,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
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


class VideoProcessor(QThread):
    """Thread for processing video frames and SAM2 operations"""

    frame_processed = Signal(int, np.ndarray)
    progress_updated = Signal(int)
    finished = Signal()
    error_occurred = Signal(str)

    def __init__(self, video_path: str, output_dir: str):
        super().__init__()
        self.video_path = video_path
        self.output_dir = output_dir
        self.cache_dir = Path(output_dir) / "cache"
        self.cache_dir.mkdir(exist_ok=True)

    def run(self):
        try:
            cap = cv2.VideoCapture(self.video_path)
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

            frame_idx = 0
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                # Cache frame
                frame_path = self.cache_dir / f"frame_{frame_idx:06d}.jpg"
                cv2.imwrite(str(frame_path), frame)

                self.frame_processed.emit(frame_idx, frame)
                self.progress_updated.emit(int((frame_idx / total_frames) * 100))
                frame_idx += 1

            cap.release()
            self.finished.emit()

        except Exception as e:
            self.error_occurred.emit(str(e))


class SAMProcessor(QRunnable):
    """Runnable for SAM2 segmentation processing"""

    def __init__(self, model, frame, points, labels, frame_idx, callback):
        super().__init__()
        self.model = model
        self.frame = frame
        self.points = points
        self.labels = labels
        self.frame_idx = frame_idx
        self.callback = callback

    def run(self):
        try:
            # Process with SAM2
            results = self.model(self.frame, points=self.points, labels=self.labels)
            self.callback(self.frame_idx, results)
        except Exception as e:
            print(f"SAM processing error: {e}")


class FrameViewer(QLabel):
    """Custom QLabel for displaying frames with click interaction"""

    point_clicked = Signal(
        int, int, int
    )  # x, y, label (1 for positive, 0 for negative)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(640, 480)
        self.setStyleSheet("border: 2px solid gray;")
        self.setScaledContents(True)
        self.current_frame = None
        self.scale_factor = 1.0
        self.click_mode = 1  # 1 for positive, 0 for negative
        self.points = []  # List of (x, y, label) tuples

    def set_frame(self, frame: np.ndarray):
        """Set the current frame to display"""
        self.current_frame = frame.copy()
        self.update_display()

    def set_click_mode(self, mode: int):
        """Set click mode: 1 for positive points, 0 for negative points"""
        self.click_mode = mode

    def add_point(self, x: int, y: int, label: int):
        """Add a point to the current frame"""
        self.points.append((x, y, label))
        self.update_display()

    def clear_points(self):
        """Clear all points"""
        self.points.clear()
        self.update_display()

    def update_display(self):
        """Update the display with current frame and points"""
        if self.current_frame is None:
            return

        display_frame = self.current_frame.copy()

        # Draw points
        for x, y, label in self.points:
            color = (
                (0, 255, 0) if label == 1 else (0, 0, 255)
            )  # Green for positive, Red for negative
            cv2.circle(display_frame, (x, y), 5, color, -1)
            cv2.circle(display_frame, (x, y), 7, (255, 255, 255), 2)

        # Convert to QPixmap
        height, width, channel = display_frame.shape
        bytes_per_line = 3 * width
        q_image = QImage(
            display_frame.data, width, height, bytes_per_line, QImage.Format_RGB888
        ).rgbSwapped()
        pixmap = QPixmap.fromImage(q_image)
        self.setPixmap(pixmap)

    def mousePressEvent(self, event):
        """Handle mouse click events"""
        if self.current_frame is None:
            return

        # Get click position relative to the actual image
        widget_size = self.size()
        pixmap_size = self.pixmap().size() if self.pixmap() else widget_size

        # Calculate scale factors
        scale_x = self.current_frame.shape[1] / pixmap_size.width()
        scale_y = self.current_frame.shape[0] / pixmap_size.height()

        # Convert widget coordinates to image coordinates
        x = int(event.position().x() * scale_x)
        y = int(event.position().y() * scale_y)

        # Ensure coordinates are within bounds
        x = max(0, min(x, self.current_frame.shape[1] - 1))
        y = max(0, min(y, self.current_frame.shape[0] - 1))

        self.add_point(x, y, self.click_mode)
        self.point_clicked.emit(x, y, self.click_mode)


class ObjectClassManager(QWidget):
    """Widget for managing object classes"""

    def __init__(self):
        super().__init__()
        self.classes = {}  # id: name mapping
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout()

        # Add class controls
        add_layout = QHBoxLayout()
        self.class_name_input = QLineEdit()
        self.class_name_input.setPlaceholderText("Enter class name")
        add_button = QPushButton("Add Class")
        add_button.clicked.connect(self.add_class)

        add_layout.addWidget(QLabel("Class Name:"))
        add_layout.addWidget(self.class_name_input)
        add_layout.addWidget(add_button)

        # Class list
        self.class_list = QListWidget()

        # Remove class button
        remove_button = QPushButton("Remove Selected")
        remove_button.clicked.connect(self.remove_selected_class)

        layout.addLayout(add_layout)
        layout.addWidget(QLabel("Classes:"))
        layout.addWidget(self.class_list)
        layout.addWidget(remove_button)

        self.setLayout(layout)

    def add_class(self):
        """Add a new class"""
        name = self.class_name_input.text().strip()
        if name and name not in self.classes.values():
            class_id = len(self.classes)
            self.classes[class_id] = name
            self.class_list.addItem(f"{class_id}: {name}")
            self.class_name_input.clear()

    def remove_selected_class(self):
        """Remove selected class"""
        current_item = self.class_list.currentItem()
        if current_item:
            # Parse class ID from item text
            text = current_item.text()
            class_id = int(text.split(":")[0])

            # Remove from classes dict
            if class_id in self.classes:
                del self.classes[class_id]

            # Remove from list
            self.class_list.takeItem(self.class_list.row(current_item))

    def get_current_class_id(self) -> Optional[int]:
        """Get currently selected class ID"""
        current_item = self.class_list.currentItem()
        if current_item:
            text = current_item.text()
            return int(text.split(":")[0])
        return None

    def get_classes(self) -> Dict[int, str]:
        """Get all classes"""
        return self.classes.copy()


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
        self.load_sam_model()

    def setup_ui(self):
        """Setup the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QHBoxLayout()

        # Left panel - Controls
        left_panel = self.create_left_panel()

        # Center panel - Frame viewer
        center_panel = self.create_center_panel()

        # Right panel - Object classes and annotations
        right_panel = self.create_right_panel()

        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(center_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([300, 700, 300])

        main_layout.addWidget(splitter)
        central_widget.setLayout(main_layout)

    def create_left_panel(self) -> QWidget:
        """Create the left control panel"""
        panel = QWidget()
        layout = QVBoxLayout()

        # Video loading group
        video_group = QGroupBox("Video Loading")
        video_layout = QVBoxLayout()

        self.load_video_btn = QPushButton("Load Video")
        self.load_video_btn.clicked.connect(self.load_video)

        self.video_path_label = QLabel("No video loaded")
        self.video_path_label.setWordWrap(True)

        self.output_dir_btn = QPushButton("Set Output Directory")
        self.output_dir_btn.clicked.connect(self.set_output_directory)

        self.output_dir_label = QLabel("No output directory set")
        self.output_dir_label.setWordWrap(True)

        video_layout.addWidget(self.load_video_btn)
        video_layout.addWidget(self.video_path_label)
        video_layout.addWidget(self.output_dir_btn)
        video_layout.addWidget(self.output_dir_label)
        video_group.setLayout(video_layout)

        # Frame navigation group
        nav_group = QGroupBox("Frame Navigation")
        nav_layout = QVBoxLayout()

        # Frame slider
        self.frame_slider = QSlider(Qt.Horizontal)
        self.frame_slider.valueChanged.connect(self.on_frame_changed)

        # Frame controls
        frame_controls = QHBoxLayout()
        self.prev_btn = QPushButton("◀ Prev")
        self.prev_btn.clicked.connect(self.prev_frame)

        self.frame_spinbox = QSpinBox()
        self.frame_spinbox.valueChanged.connect(self.on_frame_spinbox_changed)

        self.next_btn = QPushButton("Next ▶")
        self.next_btn.clicked.connect(self.next_frame)

        frame_controls.addWidget(self.prev_btn)
        frame_controls.addWidget(self.frame_spinbox)
        frame_controls.addWidget(self.next_btn)

        nav_layout.addWidget(QLabel("Frame:"))
        nav_layout.addWidget(self.frame_slider)
        nav_layout.addLayout(frame_controls)
        nav_group.setLayout(nav_layout)

        # SAM controls group
        sam_group = QGroupBox("SAM2 Controls")
        sam_layout = QVBoxLayout()

        # Point selection mode
        point_mode_layout = QHBoxLayout()
        self.positive_point_btn = QPushButton("Positive Point")
        self.positive_point_btn.setCheckable(True)
        self.positive_point_btn.setChecked(True)
        self.positive_point_btn.clicked.connect(lambda: self.set_point_mode(1))

        self.negative_point_btn = QPushButton("Negative Point")
        self.negative_point_btn.setCheckable(True)
        self.negative_point_btn.clicked.connect(lambda: self.set_point_mode(0))

        point_mode_layout.addWidget(self.positive_point_btn)
        point_mode_layout.addWidget(self.negative_point_btn)

        # SAM action buttons
        self.clear_points_btn = QPushButton("Clear Points")
        self.clear_points_btn.clicked.connect(self.clear_points)

        self.segment_btn = QPushButton("Segment Object")
        self.segment_btn.clicked.connect(self.segment_current_frame)

        # Propagation controls
        prop_layout = QHBoxLayout()
        self.propagate_forward_btn = QPushButton("Propagate Forward")
        self.propagate_forward_btn.clicked.connect(self.propagate_forward)

        self.propagate_backward_btn = QPushButton("Propagate Backward")
        self.propagate_backward_btn.clicked.connect(self.propagate_backward)

        prop_layout.addWidget(self.propagate_forward_btn)
        prop_layout.addWidget(self.propagate_backward_btn)

        sam_layout.addLayout(point_mode_layout)
        sam_layout.addWidget(self.clear_points_btn)
        sam_layout.addWidget(self.segment_btn)
        sam_layout.addLayout(prop_layout)
        sam_group.setLayout(sam_layout)

        # Export group
        export_group = QGroupBox("Export")
        export_layout = QVBoxLayout()

        self.export_yolo_btn = QPushButton("Export YOLO Dataset")
        self.export_yolo_btn.clicked.connect(self.export_yolo_dataset)

        self.progress_bar = QProgressBar()

        export_layout.addWidget(self.export_yolo_btn)
        export_layout.addWidget(self.progress_bar)
        export_group.setLayout(export_layout)

        # Add all groups to layout
        layout.addWidget(video_group)
        layout.addWidget(nav_group)
        layout.addWidget(sam_group)
        layout.addWidget(export_group)
        layout.addStretch()

        panel.setLayout(layout)
        return panel

    def create_center_panel(self) -> QWidget:
        """Create the center frame viewer panel"""
        panel = QWidget()
        layout = QVBoxLayout()

        # Frame viewer
        self.frame_viewer = FrameViewer()
        self.frame_viewer.point_clicked.connect(self.on_point_clicked)

        layout.addWidget(self.frame_viewer)
        panel.setLayout(layout)
        return panel

    def create_right_panel(self) -> QWidget:
        """Create the right panel for classes and annotations"""
        panel = QWidget()
        layout = QVBoxLayout()

        # Object class manager
        self.class_manager = ObjectClassManager()

        # Current annotations
        annotations_group = QGroupBox("Current Frame Annotations")
        annotations_layout = QVBoxLayout()

        self.annotations_list = QListWidget()

        remove_annotation_btn = QPushButton("Remove Selected Annotation")
        remove_annotation_btn.clicked.connect(self.remove_annotation)

        annotations_layout.addWidget(self.annotations_list)
        annotations_layout.addWidget(remove_annotation_btn)
        annotations_group.setLayout(annotations_layout)

        # Statistics
        stats_group = QGroupBox("Statistics")
        stats_layout = QVBoxLayout()

        self.stats_label = QTextEdit()
        self.stats_label.setMaximumHeight(150)
        self.stats_label.setReadOnly(True)

        stats_layout.addWidget(self.stats_label)
        stats_group.setLayout(stats_layout)

        layout.addWidget(self.class_manager)
        layout.addWidget(annotations_group)
        layout.addWidget(stats_group)

        panel.setLayout(layout)
        return panel

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
            self.video_path_label.setText(f"Video: {Path(file_path).name}")
            self.load_video_frames()

    def set_output_directory(self):
        """Set the output directory for the dataset"""
        dir_path = QFileDialog.getExistingDirectory(self, "Select Output Directory")

        if dir_path:
            self.output_dir = dir_path
            self.output_dir_label.setText(f"Output: {Path(dir_path).name}")

    def load_video_frames(self):
        """Load frames from the video"""
        if not self.video_path:
            return

        try:
            cap = cv2.VideoCapture(self.video_path)
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

            # Setup frame navigation
            self.frame_slider.setMaximum(total_frames - 1)
            self.frame_spinbox.setMaximum(total_frames - 1)

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
        self.video_processor.progress_updated.connect(self.progress_bar.setValue)
        self.video_processor.start()

    def on_frame_cached(self, frame_idx: int, frame: np.ndarray):
        """Handle cached frame"""
        self.frame_cache[frame_idx] = frame

    def on_frame_changed(self, frame_idx: int):
        """Handle frame slider change"""
        self.current_frame_idx = frame_idx
        self.frame_spinbox.setValue(frame_idx)
        self.load_current_frame()
        self.update_annotations_display()

    def on_frame_spinbox_changed(self, frame_idx: int):
        """Handle frame spinbox change"""
        self.frame_slider.setValue(frame_idx)

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
            self.frame_slider.setValue(self.current_frame_idx - 1)

    def next_frame(self):
        """Go to next frame"""
        max_frame = self.frame_slider.maximum()
        if self.current_frame_idx < max_frame:
            self.frame_slider.setValue(self.current_frame_idx + 1)

    def set_point_mode(self, mode: int):
        """Set point selection mode"""
        self.frame_viewer.set_click_mode(mode)

        # Update button states
        self.positive_point_btn.setChecked(mode == 1)
        self.negative_point_btn.setChecked(mode == 0)

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

        current_class_id = self.class_manager.get_current_class_id()
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
            "class_name": self.class_manager.classes.get(class_id, "unknown"),
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
        self.annotations_list.clear()

        if self.current_frame_idx in self.annotations:
            for i, annotation in enumerate(self.annotations[self.current_frame_idx]):
                class_name = annotation["class_name"]
                item_text = f"Object {i}: {class_name}"
                self.annotations_list.addItem(item_text)

    def remove_annotation(self):
        """Remove selected annotation"""
        current_item = self.annotations_list.currentItem()
        if current_item and self.current_frame_idx in self.annotations:
            row = self.annotations_list.row(current_item)
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
            classes = self.class_manager.get_classes()
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

        self.stats_label.setText(stats_text.strip())


def main():
    app = QApplication(sys.argv)

    window = DataEngineMainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
