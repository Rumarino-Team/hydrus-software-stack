# mission_planner/types.py
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable
from geometry_msgs.msg import Point

class TaskType(Enum):
    MOVE_TO_CENTER = 1    # Move directly to the center of an object
    MOVE_AROUND = 2       # Circle around an object
    MOVE_THROUGH = 3      # Move through an object (e.g., a gate)
    HOLD_POSITION = 4     # Maintain current position
    SURFACE = 5           # Move to the surface
    SEARCH = 6            # Execute a search pattern (new type for branching)

@dataclass
class MissionObject:
    position: Point
    object_name: str
    distance: float

@dataclass
class MissionInstructions:
    task: TaskType
    object_cls: Optional[str]
    conditions: Callable
    status: Optional[str] = "NONCALLED"  # Not used in the tree-based approach



@dataclass
class TaskParameters:
    radius: float = 2.0           # For MOVE_AROUND – radius of circular motion
    circle_points: int = 8        # For MOVE_AROUND – number of points in circle
    approach_distance: float = 1.0 # Distance to maintain from target
    hold_time: float = 5.0        # Time to hold position in seconds
