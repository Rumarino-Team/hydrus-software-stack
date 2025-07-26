"""
Center panel containing the frame viewer
"""

from PySide6.QtWidgets import QVBoxLayout, QWidget

from .frame_viewer import FrameViewer


class CenterPanel(QWidget):
    """Center panel for frame viewing"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()

    def setup_ui(self):
        """Setup the center panel UI"""
        layout = QVBoxLayout()

        # Frame viewer
        self.frame_viewer = FrameViewer()

        layout.addWidget(self.frame_viewer)
        self.setLayout(layout)
