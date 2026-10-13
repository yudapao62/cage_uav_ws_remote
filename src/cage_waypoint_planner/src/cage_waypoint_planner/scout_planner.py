"""Side-effect-free Scout survey route planner.

The route is generated from nominal survey geometry.  It deliberately has no
dependency on simulation ground truth or on the persistent map being built.
"""
import math
from dataclasses import dataclass

from cage_geo.geodesy import LocalENU

from .planner import PlannedWaypoint


@dataclass(frozen=True)
class ScoutRouteConfig:
    rows: int = 6
    cols: int = 10
    origin_x: float = 70.0
    origin_y: float = -6.0
    cell_size_x: float = 12.0
    cell_size_y: float = 12.0
    yaw_deg: float = 0.0
    survey_relative_alt: float = 25.0
    enter_distance: float = 8.0
    acceptance_radius: float = 1.0
    speed: float = 8.0
    max_leg_distance: float = 5000.0


def _rotate(x_value, y_value, yaw):
    return (math.cos(yaw) * x_value - math.sin(yaw) * y_value,
            math.sin(yaw) * x_value + math.cos(yaw) * y_value)


def serpentine_survey_points(config):
    """Return two nominal row endpoints per row in serpentine order."""
    _validate_config(config)
    yaw = math.radians(config.yaw_deg)
    result = []
    for row_zero in range(config.rows):
        row_offset = (row_zero + 0.5) * config.cell_size_y
        columns = (0.5, config.cols - 0.5)
        if row_zero % 2:
            columns = tuple(reversed(columns))
        for column in columns:
            dx, dy = _rotate(column * config.cell_size_x, row_offset, yaw)
            result.append((config.origin_x + dx, config.origin_y + dy))
    return result


def plan_scout_mission(home, map_reference, config):
    """Build TAKEOFF/TRANSIT plus 12 survey endpoints for a 6-row cage.

    ``home`` and ``map_reference`` are WGS84 ``(lat, lon, alt)`` tuples.  The
    nominal cage coordinates are ENU metres in ``map_reference``.  Survey
    progress counts SCOUT_ENTER, SCOUT_PASS and SCOUT_EXIT only, so the default
    route reports exactly 12/12 even though TAKEOFF and TRANSIT are also part of
    the uploaded FCU mission.
    """
    _validate_config(config)
    points = serpentine_survey_points(config)
    if len(points) < 2:
        raise ValueError('Scout route requires at least two survey endpoints')
    reference = LocalENU(*map_reference)
    first, second = points[0], points[1]
    leg_x, leg_y = second[0] - first[0], second[1] - first[1]
    leg_length = math.hypot(leg_x, leg_y)
    if leg_length <= 1e-9:
        raise ValueError('Scout first survey leg is degenerate')
    approach = (first[0] - config.enter_distance * leg_x / leg_length,
                first[1] - config.enter_distance * leg_y / leg_length)
    mission_points = [
        ((None, None), 'TAKEOFF'),
        (approach, 'TRANSIT'),
    ]
    for index, point in enumerate(points):
        if index == 0:
            kind = 'SCOUT_ENTER'
        elif index == len(points) - 1:
            kind = 'SCOUT_EXIT'
        else:
            kind = 'SCOUT_PASS'
        mission_points.append((point, kind))

    result = []
    previous_xy = (0.0, 0.0)
    for seq, (point, kind) in enumerate(mission_points):
        if kind == 'TAKEOFF':
            latitude, longitude = float(home[0]), float(home[1])
            current_xy = (0.0, 0.0)
        else:
            current_xy = point
            latitude, longitude, _ = reference.to_wgs84(
                current_xy[0], current_xy[1], 0.0)
        if seq + 1 < len(mission_points):
            next_point = mission_points[seq + 1][0]
            if next_point[0] is None:
                yaw = 0.0
            else:
                dx = next_point[0] - current_xy[0]
                dy = next_point[1] - current_xy[1]
                yaw = math.degrees(math.atan2(dx, dy)) % 360.0
        else:
            dx = current_xy[0] - previous_xy[0]
            dy = current_xy[1] - previous_xy[1]
            yaw = math.degrees(math.atan2(dx, dy)) % 360.0
        if math.hypot(current_xy[0] - previous_xy[0],
                      current_xy[1] - previous_xy[1]) > config.max_leg_distance:
            raise ValueError('Scout mission leg exceeds max_leg_distance')
        result.append(PlannedWaypoint(
            seq=seq,
            lat=latitude,
            lon=longitude,
            alt=config.survey_relative_alt,
            yaw=yaw,
            acceptance_radius=config.acceptance_radius,
            speed=config.speed,
            hold_time=0.2 if kind.startswith('SCOUT_') else 0.0,
            waypoint_type=kind,
            cell_id='',
        ))
        previous_xy = current_xy
    return result


def _validate_config(config):
    if int(config.rows) < 1 or int(config.cols) < 2:
        raise ValueError('Scout survey needs at least 1 row and 2 columns')
    positive = (config.cell_size_x, config.cell_size_y,
                config.survey_relative_alt, config.enter_distance,
                config.acceptance_radius, config.speed,
                config.max_leg_distance)
    if not all(math.isfinite(float(value)) and float(value) > 0.0
               for value in positive):
        raise ValueError('Scout route dimensions and flight limits must be positive')
    for value in (config.origin_x, config.origin_y, config.yaw_deg):
        if not math.isfinite(float(value)):
            raise ValueError('Scout nominal survey geometry must be finite')
