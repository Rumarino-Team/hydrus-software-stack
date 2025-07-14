#!/usr/bin/env python3
"""
Data Engine GUI Application
An automatic data engine that uses visual prompts to interact with AI models
for generating YOLO training datasets using SAM2 segmentation.
"""

import sys

# Import modularized GUI components
from gui import DataEngineMainWindow
from PySide6.QtWidgets import QApplication


def main():
    app = QApplication(sys.argv)

    window = DataEngineMainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
