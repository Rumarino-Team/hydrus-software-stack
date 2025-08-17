# GUI Module

This directory contains the modularized GUI components for the SAM2 Data Engine application.

## Structure

- `__init__.py` - Module initialization and exports
- `main_window.py` - Main application window and core logic
- `frame_viewer.py` - Interactive frame viewer widget with point selection
- `object_class_manager.py` - Widget for managing object classes
- `left_panel.py` - Left control panel with video loading, navigation, and SAM controls
- `right_panel.py` - Right panel with class management and annotations display
- `center_panel.py` - Center panel wrapper for the frame viewer
- `workers.py` - Background worker threads for video processing and SAM operations

## Components Overview

### DataEngineMainWindow
The main application window that orchestrates all other components. Handles:
- Video loading and frame caching
- SAM model initialization
- Annotation management
- YOLO dataset export

### FrameViewer
Custom widget for displaying video frames with interactive point selection:
- Click-based point annotation (positive/negative)
- Visual feedback for selected points
- Frame scaling and coordinate mapping

### ObjectClassManager
Widget for managing object classes in the dataset:
- Add/remove classes
- Class selection for annotation
- Dynamic class ID assignment

### Control Panels
- **LeftControlPanel**: Video controls, navigation, SAM settings, export
- **RightPanel**: Class management, annotations list, statistics
- **CenterPanel**: Simple wrapper for the frame viewer

### Workers
- **VideoProcessor**: Background thread for frame extraction and caching
- **SAMProcessor**: Background worker for SAM2 segmentation processing

## Usage

```python
from gui import DataEngineMainWindow
from PySide6.QtWidgets import QApplication
import sys

app = QApplication(sys.argv)
window = DataEngineMainWindow()
window.show()
sys.exit(app.exec())
```

## Dependencies

- PySide6 (Qt GUI framework)
- OpenCV (cv2) for image processing
- NumPy for array operations
- PyTorch for tensor operations
- Native SAM2 for segmentation
- pathlib for path handling

## Architecture

The GUI follows a modular design pattern where:
1. Each major UI component is in its own file
2. The main window coordinates between components
3. Worker threads handle heavy processing
4. Signal/slot connections provide component communication
5. Clean separation between UI and business logic
