"""
Frame viewer widget for displaying video frames with interactive point selection
"""

import cv2
import numpy as np
from PySide6.QtCore import Qt, Signal
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
        self.setScaledContents(False)  # Don't stretch - maintain aspect ratio
        self.setAlignment(Qt.AlignCenter)  # Center the image
        self.current_frame = None
        self.scale_factor = 1.0
        self.click_mode = 1  # 1 for positive, 0 for negative
        self.points = []  # List of (x, y, label) tuples
        self.masks = []  # List of masks to overlay

    def set_frame(self, frame: np.ndarray):
        """Set the current frame to display"""
        self.current_frame = frame.copy()
        self.update_display()

    def add_mask(
        self, mask: np.ndarray, color: tuple = (0, 255, 0), alpha: float = 0.3
    ):
        """Add a segmentation mask to display"""
        self.masks.append({"mask": mask, "color": color, "alpha": alpha})
        self.update_display()

    def clear_masks(self):
        """Clear all segmentation masks"""
        self.masks.clear()
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
        """Update the display with current frame, masks, and points"""
        if self.current_frame is None:
            return

        display_frame = self.current_frame.copy()

        # Draw segmentation masks
        for mask_info in self.masks:
            mask = mask_info["mask"]
            color = mask_info["color"]
            alpha = mask_info["alpha"]

            # Validate mask
            try:
                if mask is None:
                    print("Warning: None mask, skipping")
                    continue

                # Convert to numpy array if needed
                if not isinstance(mask, np.ndarray):
                    mask = np.array(mask)

                if mask.size == 0:
                    print("Warning: Empty mask, skipping")
                    continue

                # Ensure mask is 2D
                if mask.ndim > 2:
                    mask = mask.squeeze()
                if mask.ndim != 2:
                    print(
                        f"Warning: Mask has invalid dimensions {mask.shape}, skipping"
                    )
                    continue

                # Ensure mask has valid dimensions
                if mask.shape[0] == 0 or mask.shape[1] == 0:
                    print(f"Warning: Mask has zero dimensions {mask.shape}, skipping")
                    continue

                # Ensure we have valid frame dimensions
                if display_frame.shape[0] <= 0 or display_frame.shape[1] <= 0:
                    continue

                # Resize mask to match frame if needed
                if mask.shape[:2] != display_frame.shape[:2]:
                    # Ensure mask is uint8 for resize
                    if mask.dtype == np.bool_:
                        mask_for_resize = mask.astype(np.uint8) * 255
                    elif mask.dtype in [np.float32, np.float64]:
                        mask_for_resize = (mask * 255).astype(np.uint8)
                    else:
                        mask_for_resize = mask.astype(np.uint8)

                    # Check target dimensions are valid
                    target_height, target_width = display_frame.shape[:2]
                    if target_height <= 0 or target_width <= 0:
                        print(
                            f"Warning: Invalid target dimensions {target_height}x{target_width}, skipping"
                        )
                        continue

                    mask = cv2.resize(
                        mask_for_resize,
                        (target_width, target_height),
                        interpolation=cv2.INTER_NEAREST,
                    )

                # Ensure final mask is binary
                if mask.max() > 1:
                    mask_bool = mask > 127
                else:
                    mask_bool = mask > 0.5

            except Exception as e:
                print(f"Warning: Error processing mask: {e}")
                continue

            # Create colored overlay
            overlay = display_frame.copy()
            try:
                overlay[mask_bool] = color
            except Exception as e:
                print(f"Warning: Failed to apply mask overlay: {e}")
                continue

            # Blend with original frame
            try:
                display_frame = cv2.addWeighted(
                    display_frame, 1 - alpha, overlay, alpha, 0
                )
            except Exception as e:
                print(f"Warning: Failed to blend overlay: {e}")
                continue

            # Add mask contours
            try:
                # Convert mask to uint8 for contour detection
                if mask.max() > 1:
                    mask_uint8 = mask.astype(np.uint8)
                else:
                    mask_uint8 = (mask_bool * 255).astype(np.uint8)

                contours, _ = cv2.findContours(
                    mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                cv2.drawContours(display_frame, contours, -1, color, 2)
            except Exception as e:
                print(f"Warning: Failed to draw contours: {e}")
                continue

        # Draw points
        for x, y, label in self.points:
            color = (
                (0, 255, 0) if label == 1 else (0, 0, 255)
            )  # Green for positive, Red for negative
            cv2.circle(display_frame, (x, y), 5, color, -1)
            cv2.circle(display_frame, (x, y), 7, (255, 255, 255), 2)

        # Convert to QPixmap with proper aspect ratio
        height, width, channel = display_frame.shape
        bytes_per_line = 3 * width
        q_image = QImage(
            display_frame.data, width, height, bytes_per_line, QImage.Format_RGB888
        ).rgbSwapped()

        # Scale pixmap to fit widget while maintaining aspect ratio
        pixmap = QPixmap.fromImage(q_image)
        widget_size = self.size()
        scaled_pixmap = pixmap.scaled(
            widget_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
        )

        self.setPixmap(scaled_pixmap)

    def mousePressEvent(self, event):
        """Handle mouse click events"""
        if self.current_frame is None or self.pixmap() is None:
            return

        # Get click position relative to the actual image
        widget_size = self.size()
        pixmap_size = self.pixmap().size()

        # Calculate the actual displayed image position and size
        # (since we use Qt.KeepAspectRatio and center alignment)
        scale_x = pixmap_size.width() / self.current_frame.shape[1]
        scale_y = pixmap_size.height() / self.current_frame.shape[0]
        scale = min(scale_x, scale_y)

        # Calculate the displayed image size
        display_width = int(self.current_frame.shape[1] * scale)
        display_height = int(self.current_frame.shape[0] * scale)

        # Calculate offset (centering)
        offset_x = (widget_size.width() - display_width) // 2
        offset_y = (widget_size.height() - display_height) // 2

        # Convert widget coordinates to image coordinates
        click_x = event.position().x() - offset_x
        click_y = event.position().y() - offset_y

        # Check if click is within the image bounds
        if 0 <= click_x <= display_width and 0 <= click_y <= display_height:
            # Convert to original image coordinates
            x = int(click_x / scale)
            y = int(click_y / scale)

            # Ensure coordinates are within bounds
            x = max(0, min(x, self.current_frame.shape[1] - 1))
            y = max(0, min(y, self.current_frame.shape[0] - 1))

            self.add_point(x, y, self.click_mode)
            self.point_clicked.emit(x, y, self.click_mode)
