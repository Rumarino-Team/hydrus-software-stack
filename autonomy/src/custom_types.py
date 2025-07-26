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
class BoundingBox3D:
    """
    Represents a 3D bounding box with center point and dimensions.
    """

    center: Point3D
    width: float  # X dimension
    height: float  # Y dimension
    depth: float  # Z dimension

    def get_corners(self) -> List[Point3D]:
        """
        Get the 8 corner points of the 3D bounding box.
        Returns corners in the order: [front-bottom-left, front-bottom-right,
        front-top-left, front-top-right, back-bottom-left, back-bottom-right,
        back-top-left, back-top-right]
        """
        cx, cy, cz = self.center.x, self.center.y, self.center.z
        w, h, d = self.width / 2, self.height / 2, self.depth / 2

        return [
            Point3D(cx - w, cy - h, cz - d),  # front-bottom-left
            Point3D(cx + w, cy - h, cz - d),  # front-bottom-right
            Point3D(cx - w, cy + h, cz - d),  # front-top-left
            Point3D(cx + w, cy + h, cz - d),  # front-top-right
            Point3D(cx - w, cy - h, cz + d),  # back-bottom-left
            Point3D(cx + w, cy - h, cz + d),  # back-bottom-right
            Point3D(cx - w, cy + h, cz + d),  # back-top-left
            Point3D(cx + w, cy + h, cz + d),  # back-top-right
        ]


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
    bbox_3d: Optional[BoundingBox3D] = None


@dataclass
class Detections:
    detections: List[Detection]
    detector_name: str
