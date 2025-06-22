"""
Object class manager widget for managing object classes in the dataset
"""

from typing import Dict, Optional

from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


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
