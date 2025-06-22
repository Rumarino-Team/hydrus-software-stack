"""
Frame viewer widget for displaying video frames with interactive point selection
"""

import cv2
import numpy as np
from PySide6.QtCore import Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel


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
