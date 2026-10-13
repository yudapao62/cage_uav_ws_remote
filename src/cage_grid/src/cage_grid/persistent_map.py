"""Persistent row/column identity state independent of ROS transport."""
import copy
import json
import math
import os
import tempfile
import time
import uuid
from dataclasses import dataclass


IDENTITY_TENTATIVE = 0
IDENTITY_CONFIRMED = 1
IDENTITY_REJECTED = 2
VISIBILITY_VISIBLE = 0
VISIBILITY_OUT_OF_VIEW = 1
VISIBILITY_EXPECTED_BUT_UNOBSERVED = 2


@dataclass
class PersistentCellState:
    row: int
    col: int
    cell_id: str
    nominal_position: tuple
    estimated_position: tuple
    covariance_xy: tuple
    identity_state: int = IDENTITY_TENTATIVE
    visibility_state: int = VISIBILITY_VISIBLE
    identity_confidence: float = 0.0
    first_seen: float = 0.0
    last_seen: float = 0.0
    observation_count: int = 0
    confirmation_count: int = 0


class PersistentLatticeMap:
    SCHEMA_VERSION = 2
    SUPPORTED_SCHEMA_VERSIONS = (1, 2)

    def __init__(self, rows, cols, nominal_cells, min_confirmations=3, map_uuid=None,
                 task_id='', cage_id=None, created_at=None, nominal_polygons=None):
        self.rows = int(rows)
        self.cols = int(cols)
        self.nominal_cells = dict(nominal_cells)
        self.nominal_polygons = {
            index: tuple(tuple(float(component) for component in point)
                         for point in polygon)
            for index, polygon in dict(nominal_polygons or {}).items()
        }
        self.min_confirmations = int(min_confirmations)
        if self.rows < 1 or self.cols < 1 or self.min_confirmations < 1:
            raise ValueError('invalid persistent-map dimensions or confirmation count')
        expected = {(row, col) for row in range(1, self.rows + 1)
                    for col in range(1, self.cols + 1)}
        if set(self.nominal_cells) != expected:
            raise ValueError('nominal cell map must cover the complete lattice')
        self.cells = {}
        # Missing cells that were already inside an expected FoV rectangle
        # need repeated evidence before they may be born. This distinguishes
        # a temporary internal miss from a cell entering a finite FoV for the
        # first time during progressive boundary growth.
        self.deferred_birth_evidence = {}
        self.map_version = 0
        self.map_uuid = str(map_uuid or uuid.uuid4())
        self.task_id = str(task_id or '')
        self.cage_id = None if cage_id is None else int(cage_id)
        self.created_at = float(time.time() if created_at is None else created_at)
        self.finalized_at = None
        self.frozen = False
        self.events = []

    def fresh_session(self, task_id, cage_id=None, created_at=None):
        """Return an empty task-scoped map with a new UUID.

        A reset creates a new object instead of clearing this object in place.
        That makes any deep snapshot held by a concurrent finalizer independent
        from the next mission's mutable state.
        """
        return type(self)(
            self.rows, self.cols, self.nominal_cells,
            min_confirmations=self.min_confirmations,
            task_id=task_id,
            cage_id=self.cage_id if cage_id is None else cage_id,
            created_at=created_at,
            nominal_polygons=self.nominal_polygons,
        )

    @staticmethod
    def cell_id(row, col):
        return 'R{:02d}C{:02d}'.format(int(row), int(col))

    def candidate_cells(self, nominal_sigma_m=3.0):
        variance = float(nominal_sigma_m) ** 2
        result = {}
        for index, nominal in self.nominal_cells.items():
            if index in self.cells:
                state = self.cells[index]
                result[index] = {
                    'position': state.estimated_position,
                    'covariance_xy': state.covariance_xy,
                    'established': True,
                }
            else:
                # An unobserved cell inside/next to the known lattice keeps
                # its permanent index.  Predict its geometry by transporting
                # the nominal neighbor delta from every established direct
                # neighbor.  This prevents a temporary hole under strong,
                # smooth deformation from attracting the whole observation
                # to an adjacent nominal cell.
                predictions = []
                neighbor_variances = []
                for neighbor in ((index[0] - 1, index[1]),
                                 (index[0] + 1, index[1]),
                                 (index[0], index[1] - 1),
                                 (index[0], index[1] + 1)):
                    state = self.cells.get(neighbor)
                    if state is None:
                        continue
                    neighbor_nominal = self.nominal_cells[neighbor]
                    predictions.append(tuple(
                        state.estimated_position[axis]
                        + nominal[axis] - neighbor_nominal[axis]
                        for axis in range(3)))
                    neighbor_variances.append(max(
                        float(state.covariance_xy[0]),
                        float(state.covariance_xy[3])))
                inferred = (tuple(sum(value[axis] for value in predictions)
                                  / len(predictions) for axis in range(3))
                            if predictions else nominal)
                inferred_variance = (max(neighbor_variances) + variance * 0.25
                                     if neighbor_variances else variance)
                result[index] = {
                    'position': inferred,
                    'covariance_xy': (inferred_variance, 0.0, 0.0,
                                      inferred_variance),
                    'established': False,
                }
        return result

    def _is_open_boundary_birth(self, row, col, pending):
        established = set(self.cells)
        index = (row, col)
        if not established and not pending:
            return True
        if index in established or index in pending:
            return True
        # A single connected component visible in the current frame may grow
        # across both axes.  Pending cells are trusted only for this frame;
        # without this rule a rectangular first observation degenerates into
        # an L shape because its own newly expanded bounding box makes the
        # remaining interior look like an illegal late birth.
        if any(neighbor in pending for neighbor in (
                (row - 1, col), (row + 1, col),
                (row, col - 1), (row, col + 1))):
            return True
        if not established:
            return False
        adjacent = any((neighbor in established) for neighbor in (
            (row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)))
        if not adjacent:
            return False
        if index in self.deferred_birth_evidence:
            recovery_threshold = max(2, self.min_confirmations)
            return self.deferred_birth_evidence[index] >= recovery_threshold
        # A never-before-expected cell may enter a finite FoV after the map's
        # bounding rectangle has already expanded around it. Adjacency keeps
        # this connected frontier growth rather than a disconnected jump.
        return True

    def apply_observation(self, observation, decision, timestamp=None):
        if self.frozen:
            raise RuntimeError('cannot update a finalized persistent map')
        timestamp = float(time.time() if timestamp is None else timestamp)
        if not decision.committed:
            self.events.append({'type': 'FREEZE', 'time': timestamp,
                                'reason': decision.reason, 'map_version': self.map_version})
            return []
        observed = {}
        for source in observation.get('cells', []):
            row = int(source['local_i']) + int(decision.delta_row)
            col = int(source['local_j']) + int(decision.delta_col)
            if not (1 <= row <= self.rows and 1 <= col <= self.cols):
                raise ValueError('committed observation maps outside lattice')
            observed[(row, col)] = source
        expected = set()
        for local_i in range(int(observation.get('local_rows', 0))):
            for local_j in range(int(observation.get('local_cols', 0))):
                index = (local_i + int(decision.delta_row),
                         local_j + int(decision.delta_col))
                if 1 <= index[0] <= self.rows and 1 <= index[1] <= self.cols:
                    expected.add(index)

        for index in observed:
            if index not in self.cells and index in self.deferred_birth_evidence:
                self.deferred_birth_evidence[index] += 1

        changed = False
        frame_events = []
        pending = set()
        # Process candidates adjacent to the current map first so multiple new
        # boundary cells in one frame can grow outward without jumping gaps.
        unhandled = dict(observed)
        progress = True
        while unhandled and progress:
            progress = False
            for index in sorted(list(unhandled)):
                row, col = index
                if index in self.cells or self._is_open_boundary_birth(row, col, pending):
                    source = unhandled.pop(index)
                    state = self.cells.get(index)
                    position = tuple(float(value) for value in source['projected_center'][:3])
                    covariance = tuple(float(value) for value in source.get(
                        'projected_covariance_xy', (9.0, 0.0, 0.0, 9.0)))
                    if state is None:
                        state = PersistentCellState(
                            row=row, col=col, cell_id=self.cell_id(row, col),
                            nominal_position=tuple(self.nominal_cells[index]),
                            estimated_position=position,
                            covariance_xy=covariance,
                            identity_confidence=float(source.get('confidence', 1.0)),
                            first_seen=timestamp, last_seen=timestamp,
                            observation_count=1, confirmation_count=1,
                        )
                        self.cells[index] = state
                        self.deferred_birth_evidence.pop(index, None)
                        pending.add(index)
                        changed = True
                        frame_events.append({'type': 'BIRTH', 'cell_id': state.cell_id})
                    else:
                        previous_visibility = state.visibility_state
                        count = state.observation_count
                        alpha = 1.0 / float(count + 1)
                        state.estimated_position = tuple(
                            (1.0 - alpha) * state.estimated_position[i] + alpha * position[i]
                            for i in range(3))
                        state.covariance_xy = covariance
                        state.identity_confidence = max(
                            state.identity_confidence, float(source.get('confidence', 1.0)))
                        state.last_seen = timestamp
                        state.observation_count += 1
                        state.confirmation_count += 1
                        changed = True
                        if previous_visibility == VISIBILITY_EXPECTED_BUT_UNOBSERVED:
                            frame_events.append({
                                'type': 'RECOVER',
                                'cell_id': state.cell_id,
                            })
                    state.visibility_state = VISIBILITY_VISIBLE
                    if (state.identity_state == IDENTITY_TENTATIVE
                            and state.confirmation_count >= self.min_confirmations):
                        state.identity_state = IDENTITY_CONFIRMED
                        changed = True
                        frame_events.append({'type': 'CONFIRM', 'cell_id': state.cell_id})
                    progress = True
        for index, source in unhandled.items():
            frame_events.append({
                'type': 'BIRTH_REJECTED_INTERNAL_OR_DISCONNECTED',
                'cell_id': self.cell_id(*index),
            })

        for index in expected - set(observed):
            if index not in self.cells:
                self.deferred_birth_evidence.setdefault(index, 0)

        for index, state in self.cells.items():
            if index in observed:
                continue
            if index in expected:
                previous = state.visibility_state
                state.visibility_state = VISIBILITY_EXPECTED_BUT_UNOBSERVED
                if previous != state.visibility_state:
                    frame_events.append({'type': 'MISS', 'cell_id': state.cell_id})
            else:
                state.visibility_state = VISIBILITY_OUT_OF_VIEW
        if changed:
            self.map_version += 1
        for event in frame_events:
            event.update(time=timestamp, map_version=self.map_version)
        self.events.extend(frame_events)
        return frame_events

    def snapshot(self):
        return {
            'map_uuid': self.map_uuid,
            'map_version': self.map_version,
            'task_id': self.task_id,
            'cage_id': self.cage_id,
            'frozen': self.frozen,
            'created_at': self.created_at,
            'finalized_at': self.finalized_at,
            'rows': self.rows,
            'cols': self.cols,
            'cells': copy.deepcopy(self.cells),
        }

    def _cell_polygon(self, index, state):
        polygon = self.nominal_polygons.get(index, ())
        if not polygon:
            return []
        delta = tuple(state.estimated_position[axis] - state.nominal_position[axis]
                      for axis in range(3))
        return [[point[axis] + delta[axis] for axis in range(3)]
                for point in polygon]

    def _edge_json(self):
        result = []
        present = set(self.cells)
        for row, col in sorted(present):
            for target, relation in (
                    ((row + 1, col), 'row'),
                    ((row, col + 1), 'column')):
                if target in present:
                    result.append({
                        'from_cell_id': self.cell_id(row, col),
                        'to_cell_id': self.cell_id(*target),
                        'relation': relation,
                    })
        return result

    def to_json_dict(self, metadata=None):
        metadata = dict(metadata or {})
        return {
            'schema_version': self.SCHEMA_VERSION,
            'task_id': self.task_id or str(metadata.get('task_id', '')),
            'cage_id': (self.cage_id if self.cage_id is not None
                        else metadata.get('cage_id')),
            'map_uuid': self.map_uuid,
            'map_version': self.map_version,
            'frozen': self.frozen,
            'frame_id': str(metadata.get('frame_id', 'map')),
            'anchor_definition': str(metadata.get('anchor_definition', '')),
            'anchor_enu': list(metadata.get('anchor_enu', [])),
            'geometry_sha256': str(metadata.get('geometry_sha256', '')),
            'configuration_sha256': str(metadata.get('configuration_sha256', '')),
            'mapping_config_sha256': str(metadata.get(
                'mapping_config_sha256', metadata.get('configuration_sha256', ''))),
            'software_source_sha256': str(metadata.get('software_source_sha256', '')),
            'observation_source': str(metadata.get('observation_source', '')),
            'reference_lat': metadata.get('reference_lat'),
            'reference_lon': metadata.get('reference_lon'),
            'reference_alt': metadata.get('reference_alt'),
            'reference_sha256': str(metadata.get('reference_sha256', '')),
            'nominal_geometry_sha256': str(metadata.get(
                'nominal_geometry_sha256', metadata.get('geometry_sha256', ''))),
            'environment_geometry_sha256': str(metadata.get(
                'environment_geometry_sha256', '')),
            'environment_id': str(metadata.get('environment_id', '')),
            'source_created_at_unix_sec': metadata.get('source_created_at_unix_sec'),
            'source_completed_at_unix_sec': metadata.get('source_completed_at_unix_sec'),
            'created_at': self.created_at,
            'finalized_at': self.finalized_at,
            'rows': self.rows,
            'cols': self.cols,
            'min_confirmations': self.min_confirmations,
            'metadata': metadata,
            'cells': [{
                'row': state.row,
                'col': state.col,
                'cell_id': state.cell_id,
                'nominal_position': list(state.nominal_position),
                'estimated_position': list(state.estimated_position),
                'position_delta': [
                    state.estimated_position[axis] - state.nominal_position[axis]
                    for axis in range(3)],
                'covariance_xy': list(state.covariance_xy),
                'polygon': self._cell_polygon(index, state),
                'identity_state': state.identity_state,
                'visibility_state': state.visibility_state,
                'identity_confidence': state.identity_confidence,
                'first_seen': state.first_seen,
                'last_seen': state.last_seen,
                'observation_count': state.observation_count,
                'confirmation_count': state.confirmation_count,
            } for index, state in sorted(self.cells.items())],
            'edges': self._edge_json(),
        }

    def finalize_snapshot(self, metadata=None, timestamp=None):
        """Freeze this session and return a detached serialization payload."""
        if not self.frozen:
            self.frozen = True
            self.finalized_at = float(time.time() if timestamp is None else timestamp)
        return copy.deepcopy(self.to_json_dict(metadata))

    @staticmethod
    def save_json_payload(path, payload):
        """Durably replace ``path`` with a detached JSON snapshot."""
        path = os.path.abspath(os.path.expanduser(path))
        directory = os.path.dirname(path)
        os.makedirs(directory, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix='.persistent_lattice_', suffix='.json', dir=directory)
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(copy.deepcopy(payload), stream, indent=2, sort_keys=True)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
            directory_descriptor = os.open(directory, os.O_RDONLY)
            try:
                os.fsync(directory_descriptor)
            finally:
                os.close(directory_descriptor)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def save_json(self, path, metadata=None):
        self.save_json_payload(path, self.to_json_dict(metadata))

    @staticmethod
    def _strict_json_load(stream):
        def pairs(values):
            result = {}
            for key, value in values:
                if key in result:
                    raise ValueError('duplicate JSON key: {}'.format(key))
                result[key] = value
            return result

        def reject_constant(value):
            raise ValueError('non-finite JSON constant: {}'.format(value))

        return json.load(stream, object_pairs_hook=pairs,
                         parse_constant=reject_constant)

    @staticmethod
    def _finite_vector(value, size, name):
        if not isinstance(value, (list, tuple)) or len(value) != size:
            raise ValueError('{} must contain {} values'.format(name, size))
        result = tuple(float(component) for component in value)
        if not all(math.isfinite(component) for component in result):
            raise ValueError('{} contains non-finite values'.format(name))
        return result

    @classmethod
    def from_json_payload(cls, payload, nominal_cells, expected_metadata=None,
                          nominal_polygons=None):
        if not isinstance(payload, dict):
            raise ValueError('persistent-map payload must be an object')
        if int(payload.get('schema_version', -1)) not in cls.SUPPORTED_SCHEMA_VERSIONS:
            raise ValueError('unsupported persistent-map schema version')
        expected_metadata = dict(expected_metadata or {})
        metadata = dict(payload.get('metadata') or {})
        for key in (
                'geometry_sha256', 'anchor_definition', 'observation_source',
                'configuration_sha256', 'mapping_config_sha256',
                'software_source_sha256', 'frame_id', 'anchor_enu', 'cage_id',
                'reference_lat', 'reference_lon', 'reference_alt',
                'reference_sha256', 'nominal_geometry_sha256',
                'environment_geometry_sha256', 'environment_id',
                'source_created_at_unix_sec', 'source_completed_at_unix_sec'):
            if key in metadata and key in payload and metadata[key] != payload[key]:
                raise ValueError('persistent-map top-level/metadata mismatch for {}'.format(key))
            if key not in metadata and key in payload:
                metadata[key] = payload[key]
        for key, value in expected_metadata.items():
            if metadata.get(key) != value:
                raise ValueError('persistent-map metadata mismatch for {}'.format(key))

        rows, cols = int(payload['rows']), int(payload['cols'])
        if rows < 1 or cols < 1 or len(nominal_cells) != rows * cols:
            raise ValueError('invalid persistent-map dimensions')
        min_confirmations = int(payload.get('min_confirmations', 3))
        if min_confirmations < 1:
            raise ValueError('invalid persistent-map min_confirmations')
        map_uuid = str(payload.get('map_uuid', ''))
        map_version = int(payload.get('map_version', 0))
        if not map_uuid or map_version < 0:
            raise ValueError('invalid persistent-map identity')
        created_at = float(payload.get('created_at', 0.0))
        finalized_raw = payload.get('finalized_at')
        finalized_at = None if finalized_raw is None else float(finalized_raw)
        if not math.isfinite(created_at) or (finalized_at is not None and
                                              (not math.isfinite(finalized_at) or
                                               finalized_at < created_at)):
            raise ValueError('invalid persistent-map timestamps')
        if metadata.get('anchor_enu'):
            cls._finite_vector(metadata['anchor_enu'], 3, 'anchor_enu')

        result = cls(
            rows, cols, nominal_cells,
            min_confirmations=min_confirmations,
            map_uuid=map_uuid,
            task_id=payload.get('task_id', metadata.get('task_id', '')),
            cage_id=payload.get('cage_id', metadata.get('cage_id')),
            created_at=created_at,
            nominal_polygons=nominal_polygons)
        result.map_version = map_version
        result.frozen = bool(payload.get('frozen', False))
        result.finalized_at = finalized_at

        seen_ids = set()
        cells_payload = payload.get('cells', [])
        if not isinstance(cells_payload, list) or len(cells_payload) > rows * cols:
            raise ValueError('invalid persistent-map cell list')
        for value in cells_payload:
            row, col = int(value['row']), int(value['col'])
            index = (row, col)
            cell_id = str(value['cell_id'])
            if index in result.cells or cell_id in seen_ids:
                raise ValueError('duplicate cell in persistent-map snapshot')
            if index not in nominal_cells or cell_id != cls.cell_id(*index):
                raise ValueError('invalid cell identity in persistent-map snapshot')
            nominal = cls._finite_vector(value['nominal_position'], 3,
                                         '{} nominal_position'.format(cell_id))
            expected_nominal = cls._finite_vector(nominal_cells[index], 3,
                                                  '{} expected nominal'.format(cell_id))
            if any(abs(nominal[axis] - expected_nominal[axis]) > 1e-6
                   for axis in range(3)):
                raise ValueError('nominal position mismatch for {}'.format(cell_id))
            estimated = cls._finite_vector(value['estimated_position'], 3,
                                           '{} estimated_position'.format(cell_id))
            if 'position_delta' in value:
                delta = cls._finite_vector(value['position_delta'], 3,
                                           '{} position_delta'.format(cell_id))
                expected_delta = tuple(estimated[axis] - nominal[axis]
                                       for axis in range(3))
                if any(abs(delta[axis] - expected_delta[axis]) > 1e-6
                       for axis in range(3)):
                    raise ValueError('position_delta mismatch for {}'.format(cell_id))
            covariance = cls._finite_vector(value['covariance_xy'], 4,
                                            '{} covariance_xy'.format(cell_id))
            if (covariance[0] < 0.0 or covariance[3] < 0.0 or
                    abs(covariance[1] - covariance[2]) > 1e-6 or
                    covariance[0] * covariance[3] - covariance[1] * covariance[2] < -1e-9):
                raise ValueError('invalid covariance for {}'.format(cell_id))
            confidence = float(value['identity_confidence'])
            first_seen, last_seen = float(value['first_seen']), float(value['last_seen'])
            observations = int(value['observation_count'])
            confirmations = int(value['confirmation_count'])
            identity_state = int(value['identity_state'])
            visibility_state = int(value['visibility_state'])
            if (not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0 or
                    not all(math.isfinite(item) for item in (first_seen, last_seen)) or
                    last_seen < first_seen or observations < confirmations or
                    confirmations < 0 or identity_state not in (0, 1, 2) or
                    visibility_state not in (0, 1, 2)):
                raise ValueError('invalid state for {}'.format(cell_id))
            if identity_state == IDENTITY_CONFIRMED and confirmations < min_confirmations:
                raise ValueError('confirmed cell below min confirmations: {}'.format(cell_id))
            polygon = value.get('polygon', [])
            if polygon:
                if not isinstance(polygon, list) or len(polygon) < 3 or len(polygon) > 64:
                    raise ValueError('invalid polygon for {}'.format(cell_id))
                points = [cls._finite_vector(item, 3, '{} polygon'.format(cell_id))
                          for item in polygon]
                area2 = sum(points[i][0] * points[(i + 1) % len(points)][1] -
                            points[(i + 1) % len(points)][0] * points[i][1]
                            for i in range(len(points)))
                if area2 <= 1e-9:
                    raise ValueError('degenerate or clockwise polygon for {}'.format(cell_id))
                if nominal_polygons and index in nominal_polygons:
                    nominal_polygon = list(nominal_polygons[index])
                    if len(points) != len(nominal_polygon):
                        raise ValueError('polygon vertex count mismatch for {}'.format(cell_id))
                    delta = tuple(estimated[axis] - nominal[axis] for axis in range(3))
                    for vertex, expected_vertex in zip(points, nominal_polygon):
                        expected_vertex = cls._finite_vector(
                            expected_vertex, 3,
                            '{} nominal polygon'.format(cell_id))
                        if any(abs(vertex[axis] - expected_vertex[axis] - delta[axis]) > 1e-6
                               for axis in range(3)):
                            raise ValueError('polygon geometry mismatch for {}'.format(cell_id))
            state = PersistentCellState(
                row=row, col=col, cell_id=cell_id,
                nominal_position=nominal,
                estimated_position=estimated,
                covariance_xy=covariance,
                identity_state=identity_state,
                visibility_state=visibility_state,
                identity_confidence=confidence,
                first_seen=first_seen,
                last_seen=last_seen,
                observation_count=observations,
                confirmation_count=confirmations,
            )
            result.cells[index] = state
            seen_ids.add(cell_id)

        expected_edges = set()
        present = set(result.cells)
        for row, col in present:
            if (row + 1, col) in present:
                expected_edges.add((cls.cell_id(row, col), cls.cell_id(row + 1, col), 'row'))
            if (row, col + 1) in present:
                expected_edges.add((cls.cell_id(row, col), cls.cell_id(row, col + 1), 'column'))
        actual_edges = set()
        edges_payload = payload.get('edges', [])
        if not isinstance(edges_payload, list) or len(edges_payload) > rows * cols * 2:
            raise ValueError('invalid persistent-map edge list')
        for edge in edges_payload:
            item = (str(edge['from_cell_id']), str(edge['to_cell_id']),
                    str(edge['relation']))
            if item in actual_edges:
                raise ValueError('duplicate persistent-map edge')
            actual_edges.add(item)
        if actual_edges != expected_edges:
            raise ValueError('persistent-map adjacency is incomplete or invalid')
        return result

    @classmethod
    def load_json(cls, path, nominal_cells, expected_metadata=None,
                  nominal_polygons=None):
        with open(path, encoding='utf-8') as stream:
            payload = cls._strict_json_load(stream)
        return cls.from_json_payload(
            payload, nominal_cells, expected_metadata=expected_metadata,
            nominal_polygons=nominal_polygons)
