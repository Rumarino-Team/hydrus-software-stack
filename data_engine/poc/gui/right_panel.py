"""
Right panel for object class management and annotations display
"""

from PySide6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QListWidget,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .object_class_manager import ObjectClassManager


class RightPanel(QWidget):
    """Right panel for classes and annotations"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()

    def setup_ui(self):
        """Setup the right panel UI"""
        layout = QVBoxLayout()

        # Object class manager
        self.class_manager = ObjectClassManager()

        # Current annotations
        annotations_group = self.create_annotations_group()

        # Statistics
        stats_group = self.create_stats_group()

        layout.addWidget(self.class_manager)
        layout.addWidget(annotations_group)
        layout.addWidget(stats_group)

        self.setLayout(layout)

    def create_annotations_group(self) -> QGroupBox:
        """Create annotations display group"""
        annotations_group = QGroupBox("Current Frame Annotations")
        annotations_layout = QVBoxLayout()

        self.annotations_list = QListWidget()

        self.remove_annotation_btn = QPushButton("Remove Selected Annotation")

        annotations_layout.addWidget(self.annotations_list)
        annotations_layout.addWidget(self.remove_annotation_btn)
        annotations_group.setLayout(annotations_layout)

        return annotations_group

    def create_stats_group(self) -> QGroupBox:
        """Create statistics display group"""
        stats_group = QGroupBox("Statistics")
        stats_layout = QVBoxLayout()

        self.stats_label = QTextEdit()
        self.stats_label.setMaximumHeight(150)
        self.stats_label.setReadOnly(True)

        stats_layout.addWidget(self.stats_label)
        stats_group.setLayout(stats_layout)

        return stats_group
