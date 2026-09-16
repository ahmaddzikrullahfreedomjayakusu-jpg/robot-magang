import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from geometry_msgs.msg import Quaternion


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance_point_to_line(px: float, py: float, a: Tuple[float, float], b: Tuple[float, float]) -> float:
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    denom = math.hypot(dx, dy)
    if denom < 1e-6:
        return math.hypot(px - ax, py - ay)
    return abs(dy * px - dx * py + bx * ay - by * ax) / denom


def project_along_line(px: float, py: float, a: Tuple[float, float], b: Tuple[float, float]) -> float:
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom < 1e-6:
        return 0.0
    return ((px - ax) * dx + (py - ay) * dy) / denom


def min_range(ranges: Iterable[float], default: float = 10.0) -> float:
    valid = [r for r in ranges if math.isfinite(r) and r > 0.02]
    return min(valid) if valid else default


def grid_index(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> Optional[Tuple[int, int, int]]:
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    if 0 <= col < width and 0 <= row < height:
        return row, col, row * width + col
    return None
