"""Pure geometry used by the RViz scene and its unit tests."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class OperationZone:
    center: tuple
    radius: float
    unit: tuple
    normal: tuple
    circle: tuple
    rectangle: tuple


def operation_zone(point_a, point_b, cage_width, circle_samples=100):
    """Build the target operation zone whose diameter is GPS_A--GPS_B."""
    ax, ay = point_a
    bx, by = point_b
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        raise ValueError('GPS_A and GPS_B must be distinct')
    if cage_width <= 0.0:
        raise ValueError('cage_width must be positive')
    if circle_samples < 8:
        raise ValueError('circle_samples must be at least 8')

    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux
    center = ((ax + bx) / 2.0, (ay + by) / 2.0)
    radius = length / 2.0
    circle = tuple(
        (center[0] + radius * math.cos(2.0 * math.pi * i / circle_samples),
         center[1] + radius * math.sin(2.0 * math.pi * i / circle_samples))
        for i in range(circle_samples + 1)
    )

    half_length = length / 2.0
    half_width = cage_width / 2.0
    rectangle = tuple(
        (center[0] + sx * half_length * ux + sy * half_width * nx,
         center[1] + sx * half_length * uy + sy * half_width * ny)
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))
    )
    return OperationZone(center, radius, (ux, uy), (nx, ny), circle, rectangle)
