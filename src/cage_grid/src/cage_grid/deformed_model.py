"""Deterministic shared-vertex geometry for non-uniform cage lattices.

The row/column topology is regular, but every cell polygon is derived from a
single shared vertex grid.  This keeps neighboring cells watertight while
allowing low-frequency bending, spacing variation, shear and small residual
offsets.  The module is ROS-free so generators and tests can use it directly.
"""
from __future__ import annotations

import hashlib
import json
import math
import random
from dataclasses import dataclass


@dataclass(frozen=True)
class DeformationConfig:
    enabled: bool = True
    random_seed: int = 42
    row_spacing_cv: float = 0.04
    col_spacing_cv: float = 0.04
    smooth_warp_amplitude_m: float = 0.8
    warp_correlation_cells: float = 3.0
    bend_amplitude_m: float = 0.6
    shear_deg: float = 2.0
    local_residual_std_m: float = 0.08
    min_edge_length_ratio: float = 0.70
    max_edge_length_ratio: float = 1.30
    min_cell_area_ratio: float = 0.55
    min_internal_angle_deg: float = 35.0

    @classmethod
    def from_mapping(cls, mapping):
        raw = dict(mapping or {})
        presets = raw.pop('presets', {}) or {}
        preset_name = raw.pop('preset', None)
        validation = dict(raw.pop('validation', {}) or {})
        merged = {}
        if preset_name:
            if preset_name not in presets:
                raise ValueError('unknown deformation preset: {}'.format(preset_name))
            merged.update(presets[preset_name] or {})
        merged.update(raw)
        merged.update(validation)
        allowed = {field.name for field in cls.__dataclass_fields__.values()}
        unknown = sorted(set(merged) - allowed)
        if unknown:
            raise ValueError('unknown deformation keys: {}'.format(', '.join(unknown)))
        return cls(**merged)

    def validate(self):
        finite = (
            self.row_spacing_cv,
            self.col_spacing_cv,
            self.smooth_warp_amplitude_m,
            self.warp_correlation_cells,
            self.bend_amplitude_m,
            self.shear_deg,
            self.local_residual_std_m,
            self.min_edge_length_ratio,
            self.max_edge_length_ratio,
            self.min_cell_area_ratio,
            self.min_internal_angle_deg,
        )
        if not all(math.isfinite(float(value)) for value in finite):
            raise ValueError('deformation values must be finite')
        if self.row_spacing_cv < 0.0 or self.col_spacing_cv < 0.0:
            raise ValueError('spacing coefficients of variation must be non-negative')
        if self.smooth_warp_amplitude_m < 0.0 or self.bend_amplitude_m < 0.0:
            raise ValueError('warp and bend amplitudes must be non-negative')
        if self.local_residual_std_m < 0.0:
            raise ValueError('local residual standard deviation must be non-negative')
        if self.warp_correlation_cells <= 0.0:
            raise ValueError('warp correlation length must be positive')
        if not 0.0 < self.min_edge_length_ratio <= self.max_edge_length_ratio:
            raise ValueError('invalid edge length ratio bounds')
        if self.min_cell_area_ratio <= 0.0:
            raise ValueError('minimum area ratio must be positive')
        if not 0.0 < self.min_internal_angle_deg < 90.0:
            raise ValueError('minimum internal angle must be between 0 and 90 degrees')


@dataclass(frozen=True)
class GridVertex:
    row: int
    col: int
    east: float
    north: float
    up: float = 0.0

    @property
    def xy(self):
        return self.east, self.north


@dataclass(frozen=True)
class DeformedCell:
    row: int
    col: int
    cell_id: str
    vertex_indices: tuple
    polygon_enu: tuple
    center_enu: tuple
    neighbors: tuple
    exists: bool = True


def _smooth_values(count, coefficient, rng, correlation):
    if count <= 0:
        return []
    if coefficient == 0.0:
        return [1.0] * count
    raw = [rng.gauss(0.0, 1.0) for _ in range(count)]
    radius = max(1, int(math.ceil(correlation)))
    smooth = []
    for index in range(count):
        weighted = total = 0.0
        for other in range(max(0, index - radius), min(count, index + radius + 1)):
            distance = (other - index) / max(correlation, 1e-9)
            weight = math.exp(-0.5 * distance * distance)
            weighted += weight * raw[other]
            total += weight
        smooth.append(weighted / total)
    mean = sum(smooth) / count
    centered = [value - mean for value in smooth]
    rms = math.sqrt(sum(value * value for value in centered) / count)
    normalized = [value / rms for value in centered] if rms > 1e-12 else [0.0] * count
    # Keep every spacing comfortably positive even for intentionally strong presets.
    return [max(0.50, 1.0 + coefficient * value) for value in normalized]


def _cumulative(values, nominal):
    result = [0.0]
    for value in values:
        result.append(result[-1] + float(nominal) * value)
    return result


def _distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def polygon_area(points):
    return 0.5 * sum(
        points[index][0] * points[(index + 1) % len(points)][1]
        - points[(index + 1) % len(points)][0] * points[index][1]
        for index in range(len(points))
    )


def polygon_centroid(points):
    area = polygon_area(points)
    if abs(area) < 1e-12:
        raise ValueError('cannot compute centroid of zero-area polygon')
    cx = cy = 0.0
    for index, point in enumerate(points):
        nxt = points[(index + 1) % len(points)]
        cross = point[0] * nxt[1] - nxt[0] * point[1]
        cx += (point[0] + nxt[0]) * cross
        cy += (point[1] + nxt[1]) * cross
    scale = 1.0 / (6.0 * area)
    return cx * scale, cy * scale, 0.0


def _orientation(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _proper_intersection(a, b, c, d):
    return _orientation(a, b, c) * _orientation(a, b, d) < 0.0 and \
        _orientation(c, d, a) * _orientation(c, d, b) < 0.0


def _minimum_internal_angle(points):
    result = 180.0
    for index, current in enumerate(points):
        previous = points[index - 1]
        nxt = points[(index + 1) % len(points)]
        a = (previous[0] - current[0], previous[1] - current[1])
        b = (nxt[0] - current[0], nxt[1] - current[1])
        denominator = max(math.hypot(*a) * math.hypot(*b), 1e-12)
        cosine = max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / denominator))
        result = min(result, math.degrees(math.acos(cosine)))
    return result


class DeformedGrid:
    """One deterministic deformed row/column lattice in ENU coordinates."""

    def __init__(self, rows, cols, cell_size_x, cell_size_y, origin_east=0.0,
                 origin_north=0.0, origin_up=0.0, yaw_deg=0.0, cage_id=0,
                 config=None, structural_holes=None):
        self.rows = int(rows)
        self.cols = int(cols)
        self.cell_size_x = float(cell_size_x)
        self.cell_size_y = float(cell_size_y)
        self.origin_east = float(origin_east)
        self.origin_north = float(origin_north)
        self.origin_up = float(origin_up)
        self.yaw_deg = float(yaw_deg)
        self.cage_id = int(cage_id)
        self.config = config or DeformationConfig()
        self.config.validate()
        self.structural_holes = self._normalize_holes(structural_holes or [])
        if self.rows < 1 or self.cols < 1:
            raise ValueError('rows and columns must be positive')
        if self.cell_size_x <= 0.0 or self.cell_size_y <= 0.0:
            raise ValueError('cell dimensions must be positive')
        if not all(math.isfinite(value) for value in (
                self.origin_east, self.origin_north, self.origin_up, self.yaw_deg)):
            raise ValueError('grid origin and yaw must be finite')
        self.vertices = self._generate_vertices()
        self.cells = self._build_cells()
        self.validate()

    @staticmethod
    def cell_id(row, col):
        return 'R{:02d}C{:02d}'.format(int(row), int(col))

    def _normalize_holes(self, holes):
        normalized = set()
        for item in holes:
            if isinstance(item, str):
                text = item.strip().upper()
                if len(text) != 6 or text[0] != 'R' or text[3] != 'C':
                    raise ValueError('invalid structural hole: {}'.format(item))
                row, col = int(text[1:3]), int(text[4:6])
            elif isinstance(item, dict):
                row, col = int(item['row']), int(item['col'])
            else:
                row, col = map(int, item)
            if not (1 <= row <= self.rows and 1 <= col <= self.cols):
                raise ValueError('structural hole outside lattice: ({}, {})'.format(row, col))
            normalized.add((row, col))
        return normalized

    def _generate_vertices(self):
        cfg = self.config
        rng = random.Random(int(cfg.random_seed))
        if cfg.enabled:
            col_scales = _smooth_values(self.cols, cfg.col_spacing_cv, rng,
                                        cfg.warp_correlation_cells)
            row_scales = _smooth_values(self.rows, cfg.row_spacing_cv, rng,
                                        cfg.warp_correlation_cells)
        else:
            col_scales = [1.0] * self.cols
            row_scales = [1.0] * self.rows
        xs = _cumulative(col_scales, self.cell_size_x)
        ys = _cumulative(row_scales, self.cell_size_y)
        total_x = max(xs[-1], self.cell_size_x)
        total_y = max(ys[-1], self.cell_size_y)

        control_count = max(4, int(math.ceil(
            self.rows * self.cols / max(2.0 * cfg.warp_correlation_cells ** 2, 1.0))))
        controls = []
        for _ in range(control_count):
            controls.append((
                rng.uniform(0.0, self.cols),
                rng.uniform(0.0, self.rows),
                rng.gauss(0.0, cfg.smooth_warp_amplitude_m),
                rng.gauss(0.0, cfg.smooth_warp_amplitude_m),
            ))

        def smooth_warp(row, col):
            if not cfg.enabled or cfg.smooth_warp_amplitude_m == 0.0:
                return 0.0, 0.0
            sum_x = sum_y = total = 0.0
            sigma = max(cfg.warp_correlation_cells, 1e-6)
            for center_col, center_row, dx, dy in controls:
                distance2 = (col - center_col) ** 2 + (row - center_row) ** 2
                weight = math.exp(-0.5 * distance2 / (sigma * sigma))
                sum_x += weight * dx
                sum_y += weight * dy
                total += weight
            return (sum_x / total, sum_y / total) if total > 1e-12 else (0.0, 0.0)

        local = {}
        shear = math.tan(math.radians(cfg.shear_deg if cfg.enabled else 0.0))
        bend_phase = rng.uniform(-math.pi, math.pi)
        for row in range(self.rows + 1):
            for col in range(self.cols + 1):
                x, y = xs[col], ys[row]
                u, v = x / total_x, y / total_y
                dx, dy = smooth_warp(row, col)
                if cfg.enabled:
                    x += shear * y
                    x += 0.35 * cfg.bend_amplitude_m * math.sin(math.pi * v + bend_phase)
                    y += cfg.bend_amplitude_m * math.sin(math.pi * u)
                    x += dx + rng.gauss(0.0, cfg.local_residual_std_m)
                    y += dy + rng.gauss(0.0, cfg.local_residual_std_m)
                local[(row, col)] = (x, y)

        # The configured anchor is exactly vertex[0][0], independent of seed.
        anchor_x, anchor_y = local[(0, 0)]
        yaw = math.radians(self.yaw_deg)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        vertices = {}
        for (row, col), (x, y) in local.items():
            x -= anchor_x
            y -= anchor_y
            east = self.origin_east + cosine * x - sine * y
            north = self.origin_north + sine * x + cosine * y
            vertices[(row, col)] = GridVertex(row, col, east, north, self.origin_up)
        return vertices

    def _build_cells(self):
        cells = {}
        for row in range(1, self.rows + 1):
            for col in range(1, self.cols + 1):
                indices = ((row - 1, col - 1), (row - 1, col),
                           (row, col), (row, col - 1))
                polygon = tuple(self.vertices[index].xy for index in indices)
                exists = (row, col) not in self.structural_holes
                neighbors = []
                for other_row, other_col in ((row - 1, col), (row + 1, col),
                                             (row, col - 1), (row, col + 1)):
                    if (1 <= other_row <= self.rows and 1 <= other_col <= self.cols
                            and (other_row, other_col) not in self.structural_holes):
                        neighbors.append(self.cell_id(other_row, other_col))
                cells[(row, col)] = DeformedCell(
                    row, col, self.cell_id(row, col), indices, polygon,
                    polygon_centroid(polygon), tuple(neighbors), exists)
        return cells

    def validate(self):
        cfg = self.config
        yaw = math.radians(self.yaw_deg)
        col_axis = (math.cos(yaw), math.sin(yaw))
        row_axis = (-math.sin(yaw), math.cos(yaw))
        for row in range(self.rows + 1):
            for col in range(self.cols):
                a, b = self.vertices[(row, col)], self.vertices[(row, col + 1)]
                projected = (b.east - a.east) * col_axis[0] + (b.north - a.north) * col_axis[1]
                if projected <= 0.0:
                    raise ValueError('column order reversed at vertex ({}, {})'.format(row, col))
        for row in range(self.rows):
            for col in range(self.cols + 1):
                a, b = self.vertices[(row, col)], self.vertices[(row + 1, col)]
                projected = (b.east - a.east) * row_axis[0] + (b.north - a.north) * row_axis[1]
                if projected <= 0.0:
                    raise ValueError('row order reversed at vertex ({}, {})'.format(row, col))

        for cell in self.cells.values():
            points = cell.polygon_enu
            area = polygon_area(points)
            if area <= 0.0:
                raise ValueError('{} has non-positive area'.format(cell.cell_id))
            if _proper_intersection(points[0], points[1], points[2], points[3]) or \
                    _proper_intersection(points[1], points[2], points[3], points[0]):
                raise ValueError('{} is self-intersecting'.format(cell.cell_id))
            lengths = [_distance(points[i], points[(i + 1) % 4]) for i in range(4)]
            ratios = (
                lengths[0] / self.cell_size_x,
                lengths[1] / self.cell_size_y,
                lengths[2] / self.cell_size_x,
                lengths[3] / self.cell_size_y,
            )
            if min(ratios) < cfg.min_edge_length_ratio or max(ratios) > cfg.max_edge_length_ratio:
                raise ValueError('{} edge ratio outside validation bounds: {}'.format(
                    cell.cell_id, tuple(round(value, 3) for value in ratios)))
            if area / (self.cell_size_x * self.cell_size_y) < cfg.min_cell_area_ratio:
                raise ValueError('{} area ratio below validation bound'.format(cell.cell_id))
            angle = _minimum_internal_angle(points)
            if angle < cfg.min_internal_angle_deg:
                raise ValueError('{} internal angle {:.2f} below validation bound'.format(
                    cell.cell_id, angle))
        return True

    def manifest(self):
        vertices = [
            {
                'index': [row, col],
                'enu': [vertex.east, vertex.north, vertex.up],
            }
            for (row, col), vertex in sorted(self.vertices.items())
        ]
        cells = [
            {
                'row': cell.row,
                'col': cell.col,
                'cell_id': cell.cell_id,
                'vertex_indices': [list(index) for index in cell.vertex_indices],
                'polygon_enu': [[point[0], point[1], self.origin_up]
                                for point in cell.polygon_enu],
                'center_enu': list(cell.center_enu),
                'neighbors': list(cell.neighbors),
                'exists': cell.exists,
            }
            for cell in sorted(self.cells.values(), key=lambda value: (value.row, value.col))
        ]
        payload = {
            'schema_version': 1,
            'frame_id': 'map',
            'cage_id': self.cage_id,
            'seed': int(self.config.random_seed),
            'anchor_definition': 'vertex_0_0',
            'anchor_enu': [self.origin_east, self.origin_north, self.origin_up],
            'rows': self.rows,
            'cols': self.cols,
            'cell_size_nominal': [self.cell_size_x, self.cell_size_y],
            'yaw_deg': self.yaw_deg,
            'vertices': vertices,
            'cells': cells,
        }
        canonical = json.dumps(payload, sort_keys=True, separators=(',', ':'))
        payload['geometry_sha256'] = hashlib.sha256(canonical.encode('utf-8')).hexdigest()
        return payload

    def write_manifest(self, path):
        payload = self.manifest()
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write('\n')
        return payload

