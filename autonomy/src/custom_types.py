from dataclasses import dataclass
from typing import List, Optional


@dataclass
class Point3D:
    x: float
    y: float
    z: float


@dataclass
class Rotation3D:
    x: float
    y: float
    z: float
    w: float


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    cls: int
    conf: float
    depth: float = 0
    point: Optional[Point3D] = None


@dataclass
class Detections:
    detections: List[Detection]
    detector_name: str
