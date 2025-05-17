# mission_planner/types.py
"""
Definition of types used in the mission planner system.
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Callable, List
from geometry_msgs.msg import Point

class TaskType(Enum):
    """Enumeration of possible task types in a mission"""
    MOVE_TO_CENTER = auto()
    MOVE_AROUND = auto()
    HOLD_POSITION = auto()
    SEARCH = auto()
    SURFACE = auto()

@dataclass
class MissionObject:
    """Class to represent an object in the mission"""
    position: Point
    object_name: str
    distance: float
