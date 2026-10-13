"""Side-effect-free AUTO Mission planner. This module never calls MAVROS."""
import math
from dataclasses import dataclass
from cage_geo.geodesy import LocalENU, cage_frame
from cage_grid.model import CircularCageGeometry


@dataclass(frozen=True)
class PlannerConfig:
    enter_distance: float = 20.0
    exit_distance: float = 20.0
    takeoff_relative_alt: float = 20.0
    cruise_relative_alt: float = 30.0
    feed_relative_alt: float = 15.0
    acceptance_radius: float = 3.0
    speed: float = 5.0
    max_leg_distance: float = 5000.0


@dataclass(frozen=True)
class PlannedWaypoint:
    seq: int
    lat: float
    lon: float
    alt: float
    yaw: float
    acceptance_radius: float
    speed: float
    hold_time: float
    waypoint_type: str
    cell_id: str = ''


class MissionPlanner:
    def __init__(self, config=None): self.config = config or PlannerConfig()

    def plan(self, home, endpoint_a, endpoint_b):
        cfg = self.config
        self._validate_config(cfg)
        local = LocalENU(*home)
        a = local.to_enu(*endpoint_a)
        b = local.to_enu(*endpoint_b)
        frame = cage_frame(a, b)
        enter = (a[0]-cfg.enter_distance*frame.unit_east, a[1]-cfg.enter_distance*frame.unit_north)
        exit_ = (b[0]+cfg.exit_distance*frame.unit_east, b[1]+cfg.exit_distance*frame.unit_north)
        if max(math.hypot(*a[:2]), math.hypot(*b[:2]), math.hypot(*enter), math.hypot(*exit_)) > cfg.max_leg_distance:
            raise ValueError('planned mission exceeds max_leg_distance')
        yaw = math.degrees(math.atan2(frame.unit_east, frame.unit_north)) % 360.0
        points = [
            ((0.0, 0.0), cfg.takeoff_relative_alt, 'TAKEOFF'),
            (enter, cfg.cruise_relative_alt, 'TRANSIT'),
            (enter, cfg.feed_relative_alt, 'CAGE_ENTER'),
            (a[:2], cfg.feed_relative_alt, 'FEED_START'),
            (b[:2], cfg.feed_relative_alt, 'FEED_END'),
            (exit_, cfg.feed_relative_alt, 'CAGE_EXIT'),
        ]
        result = []
        for seq, (point, alt, kind) in enumerate(points):
            lat, lon, _ = local.to_wgs84(point[0], point[1], 0.0)
            result.append(PlannedWaypoint(seq, lat, lon, alt, yaw,
                                          cfg.acceptance_radius, cfg.speed, 0.0, kind, ''))
        return result

    def plan_cells(self, home, targets, transit_relative_alt=None):
        """Plan one complete grid mission from CellTarget-like objects."""
        if not targets:
            raise ValueError('at least one CellTarget is required')
        cfg = self.config
        self._validate_config(cfg)
        local = LocalENU(*home)
        first, last = targets[0], targets[-1]
        first_enu = local.to_enu(first.lat, first.lon, 0.0)
        last_enu = local.to_enu(last.lat, last.lon, 0.0)
        cruise = float(transit_relative_alt or cfg.cruise_relative_alt)
        yaw = math.degrees(math.atan2(last_enu[0]-first_enu[0], last_enu[1]-first_enu[1])) % 360.0
        points = [((0.0, 0.0), cfg.takeoff_relative_alt, 'TAKEOFF', ''),
                  ((first_enu[0], first_enu[1]), cruise, 'TRANSIT', ''),
                  ((first_enu[0], first_enu[1]), float(first.relative_alt or cfg.feed_relative_alt), 'GRID_ENTER', '')]
        for target in targets:
            enu = local.to_enu(target.lat, target.lon, 0.0)
            points.append(((enu[0], enu[1]), float(target.relative_alt or cfg.feed_relative_alt),
                           'CELL_TARGET', target.cell_id))
        points.append(((last_enu[0], last_enu[1]), float(last.relative_alt or cfg.feed_relative_alt), 'GRID_EXIT', ''))
        self._validate_legs([point for point, _alt, _kind, _cell_id in points], cfg.max_leg_distance)
        result = []
        for seq, (point, alt, kind, cell_id) in enumerate(points):
            lat, lon, _ = local.to_wgs84(point[0], point[1], 0.0)
            result.append(PlannedWaypoint(seq, lat, lon, alt, yaw,
                                          cfg.acceptance_radius, cfg.speed, 0.0, kind, cell_id))
        return result

    def plan_circular(self, home, endpoint_a, endpoint_b, feeding_dwell_sec,
                      transit_relative_alt=None, target_acceptance_radius=None,
                      feed_relative_alt=None):
        """Plan one center feeding target; A/B are diameter endpoints."""
        cfg = self.config
        self._validate_config(cfg)
        dwell = float(feeding_dwell_sec)
        acceptance = float(target_acceptance_radius or cfg.acceptance_radius)
        cruise = float(transit_relative_alt or cfg.cruise_relative_alt)
        feed_alt = float(feed_relative_alt or cfg.feed_relative_alt)
        if not math.isfinite(dwell) or dwell <= 0.0:
            raise ValueError('circular feeding_dwell_sec must be positive')
        if not math.isfinite(acceptance) or acceptance <= 0.0:
            raise ValueError('circular target acceptance radius must be positive')
        geometry = CircularCageGeometry.from_wgs84(endpoint_a, endpoint_b, home)
        center = geometry.center
        if math.hypot(center[0], center[1]) > cfg.max_leg_distance:
            raise ValueError('circular feeding target exceeds max_leg_distance')
        yaw = math.degrees(math.atan2(geometry.anchor_b[0]-geometry.anchor_a[0],
                                     geometry.anchor_b[1]-geometry.anchor_a[1])) % 360.0
        points = [
            ((0.0, 0.0), cfg.takeoff_relative_alt, cfg.acceptance_radius, 0.0, 'TAKEOFF'),
            ((center[0], center[1]), cruise, cfg.acceptance_radius, 0.0, 'TRANSIT'),
            ((center[0], center[1]), feed_alt, acceptance, dwell, 'FEED_POINT'),
        ]
        self._validate_legs([point for point, _alt, _accept, _hold, _kind in points], cfg.max_leg_distance)
        result = []
        for seq, (point, alt, radius, hold, kind) in enumerate(points):
            if not math.isfinite(alt) or alt <= 0.0:
                raise ValueError('circular waypoint altitude must be positive')
            lat, lon, _ = LocalENU(*home).to_wgs84(point[0], point[1], 0.0)
            result.append(PlannedWaypoint(seq, lat, lon, alt, yaw, radius,
                                          cfg.speed, hold, kind, ''))
        return result

    @staticmethod
    def _validate_legs(points, max_leg_distance):
        previous = (0.0, 0.0)
        for point in points:
            if not all(math.isfinite(v) for v in point):
                raise ValueError('planned waypoint contains non-finite coordinates')
            if math.hypot(point[0], point[1]) > max_leg_distance:
                raise ValueError('planned mission exceeds max_leg_distance')
            if math.hypot(point[0]-previous[0], point[1]-previous[1]) > max_leg_distance:
                raise ValueError('planned mission leg exceeds max_leg_distance')
            previous = point

    @staticmethod
    def _validate_config(cfg):
        positive = [cfg.enter_distance, cfg.exit_distance, cfg.takeoff_relative_alt,
                    cfg.cruise_relative_alt, cfg.feed_relative_alt,
                    cfg.acceptance_radius, cfg.speed, cfg.max_leg_distance]
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError('planner distances, heights, radius and speed must be positive')
