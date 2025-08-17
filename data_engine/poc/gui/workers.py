"""
Background workers for video processing and SAM operations
"""

from pathlib import Path

import cv2
import numpy as np
import torch
from PySide6.QtCore import QRunnable, QThread, Signal


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
