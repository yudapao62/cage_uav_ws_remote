"""Shared cage geometry and task-order model used by ROS, RViz and Web."""
import math
from dataclasses import dataclass
from cage_geo.geodesy import LocalENU


VALID_GEOMETRY_MODES = ('rectangular', 'circular')


def normalize_geometry_mode(value):
    """Return the configured geometry mode, failing closed on typos."""
    mode = str(value).strip().lower()
    if mode not in VALID_GEOMETRY_MODES:
        raise ValueError('invalid /cage/geometry_mode {!r}; expected rectangular or circular'.format(value))
    return mode


@dataclass(frozen=True)
class Cell:
    cage_id: int
    row: int
    col: int
    cell_id: str
    x: float
    y: float
    z: float
    lat: float
    lon: float
    relative_alt: float = 15.0


@dataclass(frozen=True)
class CircularCageGeometry:
    """A/B diameter endpoints and their midpoint in one ENU frame."""
    anchor_a: tuple
    anchor_b: tuple
    center: tuple
    center_wgs84: tuple
    diameter: float
    radius: float
    reference: tuple

    @classmethod
    def from_wgs84(cls, anchor_a, anchor_b, reference):
        values = tuple(anchor_a) + tuple(anchor_b) + tuple(reference)
        if len(anchor_a) != 3 or len(anchor_b) != 3 or len(reference) != 3:
            raise ValueError('anchors and ENU reference must be (lat, lon, alt)')
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError('circular cage coordinates must be finite')
        for point in (anchor_a, anchor_b, reference):
            if not (-90.0 <= float(point[0]) <= 90.0 and -180.0 <= float(point[1]) <= 180.0):
                raise ValueError('circular cage latitude/longitude is outside WGS84 range')
        local = LocalENU(*reference)
        a = local.to_enu(*anchor_a)
        b = local.to_enu(*anchor_b)
        center = tuple(0.5 * (a[i] + b[i]) for i in range(3))
        diameter = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
        if diameter <= 0.0:
            raise ValueError('invalid circular cage diameter')
        return cls(tuple(a), tuple(b), center, local.to_wgs84(*center),
                   diameter, diameter / 2.0, tuple(reference))


class CageGridModel:
    def __init__(self, cage_id=0, rows=6, cols=10, origin_x=70.0, origin_y=-6.0,
                 origin_z=0.0, cell_size_x=12.0, cell_size_y=12.0, yaw=0.0,
                 reference=None):
        self.cage_id = int(cage_id); self.rows = int(rows); self.cols = int(cols)
        self.origin_x = float(origin_x); self.origin_y = float(origin_y); self.origin_z = float(origin_z)
        self.cell_size_x = float(cell_size_x); self.cell_size_y = float(cell_size_y); self.yaw = float(yaw)
        self.reference = reference or (-35.363261, 149.165230, 584.0)
        self.local = LocalENU(*self.reference)
        if self.rows < 1 or self.cols < 1 or self.cell_size_x <= 0 or self.cell_size_y <= 0:
            raise ValueError('invalid cage grid dimensions')

    @classmethod
    def from_beacon(cls, geometry, home, rows, cols, cell_size_x, cell_size_y,
                    longitudinal_m=0.0, lateral_m=0.0, yaw_offset_deg=0.0):
        """Place a grid and its GPS cells in one UAV-home ENU frame."""
        reference = (home.latitude, home.longitude, home.altitude)
        local = LocalENU(*reference)
        ax, ay, _ = local.to_enu(geometry.endpoint_a_lat, geometry.endpoint_a_lon, geometry.endpoint_a_alt)
        bx, by, _ = local.to_enu(geometry.endpoint_b_lat, geometry.endpoint_b_lon, geometry.endpoint_b_alt)
        baseline = math.hypot(bx - ax, by - ay)
        if baseline <= 0:
            raise ValueError('invalid beacon baseline')
        yaw = math.atan2(by - ay, bx - ax) + math.radians(float(yaw_offset_deg))
        along, across = float(longitudinal_m), float(lateral_m)
        ox = ax + math.cos(yaw)*along - math.sin(yaw)*across
        oy = ay + math.sin(yaw)*along + math.cos(yaw)*across
        return cls(geometry.cage_id, rows, cols, ox, oy, 0.0,
                   cell_size_x, cell_size_y, yaw, reference)

    @staticmethod
    def cell_id(row, col):
        return 'R{:02d}C{:02d}'.format(int(row), int(col))

    def validate_cell(self, row, col):
        if not (1 <= int(row) <= self.rows and 1 <= int(col) <= self.cols):
            raise ValueError('cell outside active grid: row={} col={}'.format(row, col))

    def parse_cell_id(self, value):
        text = str(value).upper().strip()
        if len(text) != 6 or text[0] != 'R' or text[3] != 'C':
            raise ValueError('invalid cell id: {}'.format(value))
        row, col = int(text[1:3]), int(text[4:6]); self.validate_cell(row, col)
        return row, col

    def cell_center(self, row, col, relative_alt=15.0):
        self.validate_cell(row, col)
        lx = (int(col) - 0.5) * self.cell_size_x
        ly = (int(row) - 0.5) * self.cell_size_y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x = self.origin_x + c * lx - s * ly
        y = self.origin_y + s * lx + c * ly
        lat, lon, _ = self.local.to_wgs84(x, y, self.origin_z)
        return Cell(self.cage_id, int(row), int(col), self.cell_id(row, col),
                    x, y, self.origin_z, lat, lon, float(relative_alt))

    def all_cells(self, relative_alt=15.0):
        return [self.cell_center(r, c, relative_alt) for r in range(1, self.rows + 1)
                for c in range(1, self.cols + 1)]

    def targets(self, mode, rows=None, cols=None, selected_cell_ids=None,
                start_xy=None, relative_alt=15.0):
        mode = str(mode).upper()
        if mode == 'FULL_GRID':
            cells = []
            for r in range(1, self.rows + 1):
                order = range(1, self.cols + 1) if r % 2 else range(self.cols, 0, -1)
                cells.extend(self.cell_center(r, c, relative_alt) for c in order)
            return cells
        if mode == 'ROW_OR_COLUMN':
            rows = list(rows or []); cols = list(cols or [])
            if len(rows) == 1 and not cols:
                r = int(rows[0]); self.validate_cell(r, 1)
                cells = [self.cell_center(r, c, relative_alt) for c in range(1, self.cols + 1)]
            elif len(cols) == 1 and not rows:
                c = int(cols[0]); self.validate_cell(1, c)
                cells = [self.cell_center(r, c, relative_alt) for r in range(1, self.rows + 1)]
            else:
                raise ValueError('ROW_OR_COLUMN requires exactly one row or one column')
            return self._nearest_order(cells, start_xy)
        if mode == 'SELECTED_CELLS':
            cells = [self.cell_center(*self.parse_cell_id(v), relative_alt)
                     for v in (selected_cell_ids or [])]
            if not cells: raise ValueError('SELECTED_CELLS requires selected_cell_ids')
            return self._nearest_order(cells, start_xy)
        raise ValueError('unsupported feeding mode: {}'.format(mode))

    @staticmethod
    def _nearest_order(cells, start_xy):
        if not start_xy: return cells
        remaining = list(cells); out = []; x, y = float(start_xy[0]), float(start_xy[1])
        while remaining:
            i = min(range(len(remaining)), key=lambda k: math.hypot(remaining[k].x-x, remaining[k].y-y))
            cell = remaining.pop(i); out.append(cell); x, y = cell.x, cell.y
        return out
