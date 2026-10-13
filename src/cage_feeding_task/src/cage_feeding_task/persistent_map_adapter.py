"""Immutable task-target view of one confirmed Scout map version."""
import math

from cage_geo.geodesy import LocalENU
from cage_grid.model import Cell


class PersistentMapTargetModel:
    def __init__(self, cage_id, map_version, cells, reference, map_uuid='',
                 map_sha256='', manifest_sha256='', frozen=True, coverage=1.0):
        self.cage_id = int(cage_id)
        self.map_version = int(map_version)
        self.reference = tuple(reference)
        self.map_uuid = str(map_uuid)
        self.map_sha256 = str(map_sha256)
        self.manifest_sha256 = str(manifest_sha256)
        self.frozen = bool(frozen)
        self.coverage = float(coverage)
        self.local = LocalENU(*self.reference)
        self._cells = {(int(cell['row']), int(cell['col'])): dict(cell)
                       for cell in cells if cell.get('confirmed', True)}
        if not self._cells:
            raise ValueError('persistent map has no confirmed cells')
        self.rows = max(index[0] for index in self._cells)
        self.cols = max(index[1] for index in self._cells)

    @classmethod
    def from_message(cls, message, reference=None):
        if reference is None:
            reference = (0.0, 0.0, 0.0)
        message_reference = (
            float(getattr(message, 'reference_lat', float('nan'))),
            float(getattr(message, 'reference_lon', float('nan'))),
            float(getattr(message, 'reference_alt', float('nan'))))
        if not (-90.0 <= message_reference[0] <= 90.0 and
                -180.0 <= message_reference[1] <= 180.0 and
                all(math.isfinite(value) for value in message_reference)):
            raise ValueError('persistent map WGS84 reference is invalid')
        if not bool(getattr(message, 'frozen', False)):
            raise ValueError('persistent map is not frozen')
        if not str(getattr(message, 'map_uuid', '')):
            raise ValueError('persistent map UUID is missing')
        if not str(getattr(message, 'map_sha256', '')) or not str(
                getattr(message, 'manifest_sha256', '')):
            raise ValueError('persistent map content hashes are missing')
        cells = []
        for cell in message.cells:
            if cell.identity_state != cell.IDENTITY_CONFIRMED or not cell.exists:
                continue
            cells.append({
                'row': cell.row,
                'col': cell.col,
                'cell_id': cell.cell_id,
                'x': cell.estimated_position.x,
                'y': cell.estimated_position.y,
                'z': cell.estimated_position.z,
                'confirmed': True,
            })
        return cls(
            message.cage_id, message.map_version, cells, message_reference,
            map_uuid=message.map_uuid,
            map_sha256=message.map_sha256,
            manifest_sha256=message.manifest_sha256,
            frozen=message.frozen,
            coverage=float(getattr(message, 'coverage', 0.0)))

    @staticmethod
    def cell_id(row, col):
        return 'R{:02d}C{:02d}'.format(int(row), int(col))

    def validate_cell(self, row, col):
        if (int(row), int(col)) not in self._cells:
            raise ValueError('cell is not confirmed in frozen map: row={} col={}'.format(
                row, col))

    def parse_cell_id(self, value):
        text = str(value).upper().strip()
        if len(text) != 6 or text[0] != 'R' or text[3] != 'C':
            raise ValueError('invalid cell id: {}'.format(value))
        row, col = int(text[1:3]), int(text[4:6])
        self.validate_cell(row, col)
        return row, col

    def cell_center(self, row, col, relative_alt=15.0):
        self.validate_cell(row, col)
        value = self._cells[(int(row), int(col))]
        x, y, z = float(value['x']), float(value['y']), float(value.get('z', 0.0))
        lat, lon, _ = self.local.to_wgs84(x, y, z)
        return Cell(self.cage_id, int(row), int(col),
                    value.get('cell_id') or self.cell_id(row, col),
                    x, y, z, lat, lon, float(relative_alt))

    def targets(self, mode, rows=None, cols=None, selected_cell_ids=None,
                start_xy=None, relative_alt=15.0):
        del start_xy  # Identity order is stable; geometry never reorders IDs.
        mode = str(mode).upper()
        if mode == 'FULL_GRID':
            requested = []
            for row in range(1, self.rows + 1):
                columns = (range(1, self.cols + 1) if row % 2
                           else range(self.cols, 0, -1))
                requested.extend((row, col) for col in columns)
        elif mode == 'ROW_OR_COLUMN':
            rows, cols = list(rows or []), list(cols or [])
            if len(rows) == 1 and not cols:
                requested = [(int(rows[0]), col)
                             for col in range(1, self.cols + 1)]
            elif len(cols) == 1 and not rows:
                requested = [(row, int(cols[0]))
                             for row in range(1, self.rows + 1)]
            else:
                raise ValueError('ROW_OR_COLUMN requires exactly one row or one column')
        elif mode == 'SELECTED_CELLS':
            requested = [self.parse_cell_id(value)
                         for value in (selected_cell_ids or [])]
            if not requested:
                raise ValueError('SELECTED_CELLS requires selected_cell_ids')
        else:
            raise ValueError('unsupported feeding mode: {}'.format(mode))
        return [self.cell_center(row, col, relative_alt)
                for row, col in requested]
