"""
Left control panel for video loading, navigation, and SAM controls
"""

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QProgressBar,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


class LeftControlPanel(QWidget):
    """Left panel containing video loading, navigation, and SAM controls"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()

    def setup_ui(self):
        """Setup the left panel UI"""
        layout = QVBoxLayout()

        # Video loading group
        video_group = self.create_video_group()

        # Frame navigation group
        nav_group = self.create_navigation_group()

        # SAM controls group
        sam_group = self.create_sam_group()

        # SAM2 Video Tracking group
        sam2_group = self.create_sam2_group()

        # Export group
        export_group = self.create_export_group()

        # Add all groups to layout
        layout.addWidget(video_group)
        layout.addWidget(nav_group)
        layout.addWidget(sam_group)
        layout.addWidget(sam2_group)
        layout.addWidget(export_group)
        layout.addStretch()

        self.setLayout(layout)

    def create_video_group(self) -> QGroupBox:
        """Create video loading group"""
        video_group = QGroupBox("Video Loading")
        video_layout = QVBoxLayout()

        self.load_video_btn = QPushButton("Load Video")
        self.video_path_label = QLabel("No video loaded")
        self.video_path_label.setWordWrap(True)

        self.output_dir_btn = QPushButton("Set Output Directory")
        self.output_dir_label = QLabel("No output directory set")
        self.output_dir_label.setWordWrap(True)

        video_layout.addWidget(self.load_video_btn)
        video_layout.addWidget(self.video_path_label)
        video_layout.addWidget(self.output_dir_btn)
        video_layout.addWidget(self.output_dir_label)
        video_group.setLayout(video_layout)

        return video_group

    def create_navigation_group(self) -> QGroupBox:
        """Create frame navigation group"""
        nav_group = QGroupBox("Frame Navigation")
        nav_layout = QVBoxLayout()

        # Frame slider
        self.frame_slider = QSlider(Qt.Horizontal)

        # Frame controls
        frame_controls = QHBoxLayout()
        self.prev_btn = QPushButton("◀ Prev")
        self.frame_spinbox = QSpinBox()
        self.next_btn = QPushButton("Next ▶")

        frame_controls.addWidget(self.prev_btn)
        frame_controls.addWidget(self.frame_spinbox)
        frame_controls.addWidget(self.next_btn)

        nav_layout.addWidget(QLabel("Frame:"))
        nav_layout.addWidget(self.frame_slider)
        nav_layout.addLayout(frame_controls)
        nav_group.setLayout(nav_layout)

        return nav_group

    def create_sam_group(self) -> QGroupBox:
        """Create SAM2 point controls group"""
        sam_group = QGroupBox("SAM2 Point Controls")
        sam_layout = QVBoxLayout()

        # Point selection mode
        point_mode_layout = QHBoxLayout()
        self.positive_point_btn = QPushButton("Positive Point")
        self.positive_point_btn.setCheckable(True)
        self.positive_point_btn.setChecked(True)

        self.negative_point_btn = QPushButton("Negative Point")
        self.negative_point_btn.setCheckable(True)

        point_mode_layout.addWidget(self.positive_point_btn)
        point_mode_layout.addWidget(self.negative_point_btn)

        # SAM action buttons
        self.clear_points_btn = QPushButton("Clear Points")
        self.segment_btn = QPushButton("Segment Object")

        sam_layout.addLayout(point_mode_layout)
        sam_layout.addWidget(self.clear_points_btn)
        sam_layout.addWidget(self.segment_btn)
        sam_group.setLayout(sam_layout)

        return sam_group

    def create_sam2_group(self) -> QGroupBox:
        """Create SAM2 Video Tracking group"""
        sam2_group = QGroupBox("SAM2 Video Tracking")
        sam2_layout = QVBoxLayout()

        # Video initialization
        self.init_video_btn = QPushButton("Initialize Video Tracking")
        self.init_video_btn.setEnabled(False)

        # Object management
        object_controls = QHBoxLayout()
        self.add_object_btn = QPushButton("Add Object")
        self.add_object_btn.setEnabled(False)
        self.remove_object_btn = QPushButton("Remove Object")
        self.remove_object_btn.setEnabled(False)

        object_controls.addWidget(self.add_object_btn)
        object_controls.addWidget(self.remove_object_btn)

        # Tracked objects list
        self.tracked_objects_list = QListWidget()
        self.tracked_objects_list.setMaximumHeight(100)

        # Propagation controls
        prop_controls = QHBoxLayout()
        self.propagate_forward_btn = QPushButton("Propagate Forward")
        self.propagate_forward_btn.setEnabled(False)
        self.propagate_backward_btn = QPushButton("Propagate Backward")
        self.propagate_backward_btn.setEnabled(False)

        prop_controls.addWidget(self.propagate_forward_btn)
        prop_controls.addWidget(self.propagate_backward_btn)

        # Clear tracking
        self.clear_tracking_btn = QPushButton("Clear All Tracking")
        self.clear_tracking_btn.setEnabled(False)

        # Auto-propagation option
        self.auto_propagate_checkbox = QCheckBox("Auto-propagate on frame change")

        # Progress bar for tracking operations
        self.tracking_progress = QProgressBar()
        self.tracking_progress.setVisible(False)

        # Status label
        self.tracking_status_label = QLabel("Video tracking not initialized")
        self.tracking_status_label.setWordWrap(True)

        # Add all controls to layout
        sam2_layout.addWidget(self.init_video_btn)
        sam2_layout.addWidget(QLabel("Tracked Objects:"))
        sam2_layout.addWidget(self.tracked_objects_list)
        sam2_layout.addLayout(object_controls)
        sam2_layout.addLayout(prop_controls)
        sam2_layout.addWidget(self.clear_tracking_btn)
        sam2_layout.addWidget(self.auto_propagate_checkbox)
        sam2_layout.addWidget(self.tracking_progress)
        sam2_layout.addWidget(self.tracking_status_label)

        sam2_group.setLayout(sam2_layout)
        return sam2_group

    def create_export_group(self) -> QGroupBox:
        """Create export group"""
        export_group = QGroupBox("Export")
        export_layout = QVBoxLayout()

        self.export_yolo_btn = QPushButton("Export YOLO Dataset")
        self.progress_bar = QProgressBar()

        export_layout.addWidget(self.export_yolo_btn)
        export_layout.addWidget(self.progress_bar)
        export_group.setLayout(export_layout)

        return export_group
