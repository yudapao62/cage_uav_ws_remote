#!/usr/bin/env python3
"""Own, validate and publish the one frozen Scout map Feeding may consume."""
import hashlib
import json
import math
import os
import tempfile
import threading
import time
import uuid

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Header

from cage_grid.model import CageGridModel
from cage_grid.scout_map_repository import (
    ArtifactValidationError,
    ScoutMapRepository,
)
from cage_msgs.msg import (
    FeedingTaskStatus,
    LatticeCell,
    LatticeEdge,
    MissionState,
    PersistentLattice,
    ScoutMapActivationStatus,
    ScoutMapSummary,
    ScoutTaskStatus,
)
from cage_msgs.srv import (
    ActivateScoutMap,
    ActivateScoutMapResponse,
    ListScoutMaps,
    ListScoutMapsResponse,
    ManageScoutMapLease,
    ManageScoutMapLeaseRequest,
    ManageScoutMapLeaseResponse,
)


SCOUT_INACTIVE = frozenset(('IDLE', 'COMPLETED', 'FAILED', 'CANCELED', 'CANCELLED'))
FEEDING_INACTIVE = frozenset((
    'TASK_IDLE', 'TASK_PREVIEW', 'TASK_COMPLETED', 'TASK_ERROR',
    'TASK_CANCELLED', 'TASK_CANCELED', 'TASK_ABORTED_TIMEOUT'))
MISSION_INACTIVE = frozenset(('IDLE', 'FINISHED'))


def _point(values):
    values = list(values) + [0.0, 0.0, 0.0]
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def _atomic_selector_temp(directory, payload):
    descriptor, path = tempfile.mkstemp(prefix='.active_map_', suffix='.json', dir=directory)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        return path
    except Exception:
        if os.path.exists(path):
            os.unlink(path)
        raise


class ScoutMapRegistryNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.active = None
        self.active_message = None
        self.activated_at_unix_sec = 0.0
        self.last_status_code = 'NO_ACTIVE_MAP'
        self.last_status_message = 'No Scout map has been activated'
        self.registry_error = False
        self.last_attempt = {
            'task_id': '', 'at': 0.0, 'success': False,
            'reason_code': '', 'message': ''}
        self.lease = None
        self.scout_state = 'IDLE'
        self.feeding_state = 'TASK_IDLE'
        self.mission_state = 'IDLE'
        self.feeder_active = False
        self.stop_event = threading.Event()

        output_root = os.path.abspath(os.path.expanduser(str(rospy.get_param(
            '/scout_mission/output_root', '~/.ros/cage_uav/scout_runs'))))
        os.makedirs(output_root, mode=0o750, exist_ok=True)
        self.output_root = os.path.realpath(output_root)
        self.selector_path = os.path.join(self.output_root, '.active_map.json')
        self.audit_path = os.path.join(self.output_root, 'map_activation_events.jsonl')

        g = lambda name, default: rospy.get_param('/cage_grid/' + name, default)
        self.cage_id = int(g('cage_id', 0))
        rows, cols = int(g('rows', 6)), int(g('cols', 10))
        self.model = CageGridModel(
            self.cage_id, rows, cols,
            g('origin_x', 70.0), g('origin_y', -6.0), g('origin_z', 0.0),
            g('cell_size_x', 12.0), g('cell_size_y', 12.0),
            math.radians(g('yaw_deg', 0.0)),
            (g('reference_lat', -35.363261),
             g('reference_lon', 149.165230),
             g('reference_alt', 584.0)))
        nominal = {(cell.row, cell.col): (cell.x, cell.y, cell.z)
                   for cell in self.model.all_cells()}
        self.anchor_definition = 'vertex_0_0'
        self.anchor_enu = (self.model.origin_x, self.model.origin_y,
                           self.model.origin_z)
        polygons = {(row, col): self._nominal_polygon(row, col)
                    for row in range(1, rows + 1)
                    for col in range(1, cols + 1)}
        nominal_payload = json.dumps({
            'cage_id': self.cage_id, 'rows': rows, 'cols': cols,
            'origin': [self.model.origin_x, self.model.origin_y, self.model.origin_z],
            'cell_size': [self.model.cell_size_x, self.model.cell_size_y],
            'yaw': self.model.yaw,
        }, sort_keys=True, separators=(',', ':'))
        geometry_sha = hashlib.sha256(nominal_payload.encode('utf-8')).hexdigest()
        cfg = rospy.get_param('/scout_map_registry', {})
        self.status_timeout = float(cfg.get('active_status_timeout_sec', 3.0))
        self.heartbeat_rate = float(cfg.get('heartbeat_rate_hz', 1.0))
        self.default_lease_ttl = float(cfg.get('lease_ttl_sec', 5.0))
        self.repository = ScoutMapRepository(
            self.output_root, nominal, polygons, self.cage_id,
            frame_id='map', anchor_definition=self.anchor_definition,
            anchor_enu=self.anchor_enu,
            geometry_sha256=geometry_sha,
            reference_wgs84=self.model.reference,
            environment_geometry_sha256=str(cfg.get('environment_geometry_sha256', '')),
            environment_id=str(cfg.get('environment_id', '')),
            minimum_coverage=float(cfg.get('minimum_coverage', rospy.get_param(
                '/scout_mission/minimum_coverage', 0.95))),
            max_snapshot_age_sec=float(cfg.get('max_snapshot_age_sec', 0.0)),
            allow_legacy_v1=bool(cfg.get('allow_legacy_v1', False)),
            legacy_profiles=dict(cfg.get('legacy_profiles', {})),
            current_preset=str(rospy.get_param('/persistent_lattice/deformation_preset', '')),
            current_seed=int(rospy.get_param('/persistent_lattice/deformation_seed', 0)),
            max_json_bytes=int(cfg.get('max_json_bytes', 16 * 1024 * 1024)),
            max_artifact_bytes=int(cfg.get('max_artifact_bytes', 1024 * 1024 * 1024)),
            max_artifact_count=int(cfg.get('max_artifact_count', 32)))

        self.map_pub = rospy.Publisher('/scout/lattice_map', PersistentLattice,
                                       queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/scout/active_map_status',
                                          ScoutMapActivationStatus,
                                          queue_size=1, latch=True)
        rospy.Subscriber('/scout/task_status', ScoutTaskStatus,
                         lambda msg: setattr(self, 'scout_state', str(msg.state).upper()),
                         queue_size=1)
        rospy.Subscriber('/feeding/task_status', FeedingTaskStatus,
                         lambda msg: setattr(self, 'feeding_state', str(msg.task_state).upper()),
                         queue_size=1)
        rospy.Subscriber('/mission/state', MissionState,
                         lambda msg: setattr(self, 'mission_state', str(msg.state).upper()),
                         queue_size=1)
        rospy.Subscriber('/feeder/status', Bool,
                         lambda msg: setattr(self, 'feeder_active', bool(msg.data)),
                         queue_size=1)
        self.activate_service = rospy.Service('/scout/activate_map', ActivateScoutMap,
                                              self.on_activate)
        self.list_service = rospy.Service('/scout/list_maps', ListScoutMaps,
                                          self.on_list)
        self.lease_service = rospy.Service('/scout/manage_map_lease', ManageScoutMapLease,
                                           self.on_lease)
        self._restore_selector()
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        rospy.on_shutdown(self.shutdown)

    def _nominal_polygon(self, row, col):
        c, s = math.cos(self.model.yaw), math.sin(self.model.yaw)
        result = []
        for local_x, local_y in (
                ((col - 1) * self.model.cell_size_x, (row - 1) * self.model.cell_size_y),
                (col * self.model.cell_size_x, (row - 1) * self.model.cell_size_y),
                (col * self.model.cell_size_x, row * self.model.cell_size_y),
                ((col - 1) * self.model.cell_size_x, row * self.model.cell_size_y)):
            result.append((self.model.origin_x + c * local_x - s * local_y,
                           self.model.origin_y + s * local_x + c * local_y,
                           self.model.origin_z))
        return tuple(result)

    def _validated_to_message(self, value, activated_at_unix_sec):
        lattice = value.lattice
        message = PersistentLattice(
            header=Header(stamp=rospy.Time.now(), frame_id=value.frame_id),
            cage_id=value.cage_id, map_version=value.map_version,
            map_uuid=value.map_uuid,
            anchor_definition=self.anchor_definition,
            anchor_enu=_point(self.anchor_enu),
            geometry_sha256=value.geometry_sha256,
            source_task_id=value.task_id, frozen=True,
            source_created_at_ros=rospy.Time.from_sec(float(lattice.created_at)),
            source_finalized_at_ros=rospy.Time.from_sec(float(lattice.finalized_at or 0.0)),
            activated_at_ros=rospy.Time.now(),
            source_created_at_unix_sec=value.source_created_at_unix_sec,
            source_completed_at_unix_sec=value.source_completed_at_unix_sec,
            activated_at_unix_sec=activated_at_unix_sec,
            map_sha256=value.map_sha256,
            manifest_sha256=value.manifest_sha256,
            mapping_config_sha256=value.mapping_config_sha256,
            software_source_sha256=value.software_source_sha256,
            observation_source=value.observation_source,
            reference_lat=value.reference_wgs84[0],
            reference_lon=value.reference_wgs84[1],
            reference_alt=value.reference_wgs84[2],
            reference_sha256=value.reference_sha256,
            environment_geometry_sha256=value.environment_geometry_sha256,
            confirmed_cells=value.confirmed_cells,
            expected_cells=value.expected_cells,
            coverage=value.coverage,
            restored_from_history=True,
            legacy_reference_assumed=value.legacy_reference_assumed,
            legacy_environment_assumed=value.legacy_environment_assumed)
        for index, state in sorted(lattice.cells.items()):
            message.cells.append(LatticeCell(
                row=state.row, col=state.col, cell_id=state.cell_id,
                polygon=[_point(point) for point in lattice._cell_polygon(index, state)],
                nominal_position=_point(state.nominal_position),
                estimated_position=_point(state.estimated_position),
                covariance_xy=list(state.covariance_xy),
                identity_state=state.identity_state,
                visibility_state=state.visibility_state,
                identity_confidence=state.identity_confidence,
                first_seen=rospy.Time.from_sec(state.first_seen),
                last_seen=rospy.Time.from_sec(state.last_seen),
                observation_count=state.observation_count,
                exists=True))
        present = set(lattice.cells)
        for row, col in sorted(present):
            for target, relation in (((row + 1, col), LatticeEdge.RELATION_ROW),
                                     ((row, col + 1), LatticeEdge.RELATION_COLUMN)):
                if target in present:
                    message.edges.append(LatticeEdge(
                        from_cell_id=lattice.cell_id(row, col),
                        to_cell_id=lattice.cell_id(*target), relation=relation))
        return message

    def _prune_lease_locked(self):
        if self.lease is not None and time.monotonic() >= self.lease['expires_at']:
            self.lease = None

    def _busy_reason_locked(self):
        self._prune_lease_locked()
        if self.lease is not None:
            return 'MAP_SWITCH_BUSY', 'A Feeding map lease is active'
        if self.scout_state not in SCOUT_INACTIVE:
            return 'SCOUT_BUSY', 'Scout task is active'
        if self.feeding_state not in FEEDING_INACTIVE:
            return 'FEEDING_BUSY', 'Feeding task is active'
        if self.mission_state not in MISSION_INACTIVE:
            return 'MISSION_BUSY', 'Mission controller is active'
        if self.feeder_active:
            return 'FEEDER_ACTIVE', 'Feeder is enabled'
        return '', ''

    def _selector_payload(self, value, activated_at):
        return {
            'schema_version': 1, 'task_id': value.task_id,
            'cage_id': value.cage_id, 'map_uuid': value.map_uuid,
            'map_version': value.map_version, 'map_sha256': value.map_sha256,
            'manifest_sha256': value.manifest_sha256,
            'activated_at_unix_sec': activated_at}

    @staticmethod
    def _same(value, request):
        return (value is not None and value.task_id == request.task_id and
                value.cage_id == int(request.cage_id) and
                value.map_uuid == request.expected_map_uuid and
                value.map_version == int(request.expected_map_version) and
                value.map_sha256 == request.expected_map_sha256 and
                value.manifest_sha256 == request.expected_manifest_sha256)

    def _response(self, success=False, committed=False, code='', message='',
                  value=None, idempotent=False):
        value = self.active if value is None else value
        return ActivateScoutMapResponse(
            success=bool(success), committed=bool(committed),
            reason_code=str(code), message=str(message),
            map_uuid=value.map_uuid if value else '',
            map_version=value.map_version if value else 0,
            map_sha256=value.map_sha256 if value else '',
            manifest_sha256=value.manifest_sha256 if value else '',
            confirmed_cells=value.confirmed_cells if value else 0,
            expected_cells=value.expected_cells if value else 0,
            coverage=value.coverage if value else 0.0,
            source_completed_at_unix_sec=(value.source_completed_at_unix_sec
                                          if value else 0.0),
            activated_at_unix_sec=self.activated_at_unix_sec if value else 0.0,
            idempotent=bool(idempotent),
            legacy_reference_assumed=(value.legacy_reference_assumed
                                      if value else False),
            legacy_environment_assumed=(value.legacy_environment_assumed
                                        if value else False))

    def on_activate(self, request):
        attempt_at = time.time()
        with self.lock:
            initial_identity = (None if self.active is None else
                                (self.active.map_uuid, self.active.map_version,
                                 self.active.map_sha256, self.active.manifest_sha256))
            if self._same(self.active, request):
                self._set_attempt(request.task_id, True, '', 'Map is already active')
                return self._response(True, True, value=self.active, idempotent=True)
        try:
            candidate = self.repository.validate(request.task_id, now_unix_sec=attempt_at)
            expected = (str(request.expected_map_uuid), int(request.expected_map_version),
                        str(request.expected_map_sha256),
                        str(request.expected_manifest_sha256))
            actual = (candidate.map_uuid, candidate.map_version,
                      candidate.map_sha256, candidate.manifest_sha256)
            if int(request.cage_id) != candidate.cage_id:
                raise ArtifactValidationError('CAGE_ID_MISMATCH', 'requested cage does not match map')
            if actual != expected:
                raise ArtifactValidationError('MAP_HASH_MISMATCH',
                                              'expected map identity/content changed')
            selector = self._selector_payload(candidate, attempt_at)
            temporary = _atomic_selector_temp(self.output_root, selector)
        except ArtifactValidationError as exc:
            self._set_attempt(request.task_id, False, exc.code, exc.message)
            return self._response(False, False, exc.code, exc.message)
        except Exception as exc:
            self._set_attempt(request.task_id, False, 'SELECTOR_WRITE_FAILED', str(exc))
            return self._response(False, False, 'SELECTOR_WRITE_FAILED', str(exc))

        committed = False
        try:
            with self.lock:
                current_identity = (None if self.active is None else
                                    (self.active.map_uuid, self.active.map_version,
                                     self.active.map_sha256, self.active.manifest_sha256))
                if current_identity != initial_identity:
                    raise ArtifactValidationError('MAP_SWITCH_BUSY',
                                                  'active map changed during validation')
                code, message = self._busy_reason_locked()
                if code:
                    raise ArtifactValidationError(code, message)
                os.replace(temporary, self.selector_path)
                committed = True
                self.active = candidate
                self.activated_at_unix_sec = attempt_at
                self.active_message = self._validated_to_message(candidate, attempt_at)
                self.registry_error = False
                self.last_status_code = ''
                self.last_status_message = 'Scout map is active'
                self._set_attempt_locked(request.task_id, True, '', 'Scout map activated')
            directory_descriptor = os.open(self.output_root, os.O_RDONLY)
            try:
                os.fsync(directory_descriptor)
            finally:
                os.close(directory_descriptor)
            self._publish_active()
            self._append_audit('ACTIVATE', candidate, True, '')
            return self._response(True, True, value=candidate)
        except ArtifactValidationError as exc:
            if os.path.exists(temporary):
                os.unlink(temporary)
            self._set_attempt(request.task_id, False, exc.code, exc.message)
            return self._response(False, False, exc.code, exc.message)
        except Exception as exc:
            if not committed and os.path.exists(temporary):
                os.unlink(temporary)
            if committed:
                with self.lock:
                    self.registry_error = True
                    self.last_status_code = 'REGISTRY_ERROR'
                    self.last_status_message = str(exc)
                self.publish_status()
                return self._response(True, True, 'ACTIVATED_REGISTRY_DEGRADED', str(exc),
                                      value=candidate)
            self._set_attempt(request.task_id, False, 'SELECTOR_WRITE_FAILED', str(exc))
            return self._response(False, False, 'SELECTOR_WRITE_FAILED', str(exc))

    def _summary(self, task_id, include_invalid):
        try:
            value = self.repository.validate(task_id)
            with self.lock:
                active = (self.active is not None and
                          self.active.map_sha256 == value.map_sha256 and
                          self.active.manifest_sha256 == value.manifest_sha256)
            return ScoutMapSummary(
                task_id=value.task_id, cage_id=value.cage_id,
                map_uuid=value.map_uuid, map_version=value.map_version,
                map_sha256=value.map_sha256,
                manifest_sha256=value.manifest_sha256,
                source_completed_at_unix_sec=value.source_completed_at_unix_sec,
                confirmed_cells=value.confirmed_cells,
                expected_cells=value.expected_cells, coverage=value.coverage,
                frozen=value.frozen,
                validation_state=ScoutMapSummary.VALIDATION_FULL_VALID,
                compatibility_state=ScoutMapSummary.COMPATIBILITY_COMPATIBLE,
                active=active,
                legacy_reference_assumed=value.legacy_reference_assumed,
                legacy_environment_assumed=value.legacy_environment_assumed,
                last_validated_at_unix_sec=value.last_validated_at_unix_sec,
                validation_code='', validation_message='')
        except ArtifactValidationError as exc:
            if not include_invalid:
                return None
            return ScoutMapSummary(
                task_id=task_id, cage_id=-1,
                validation_state=ScoutMapSummary.VALIDATION_INVALID,
                compatibility_state=ScoutMapSummary.COMPATIBILITY_INCOMPATIBLE,
                validation_code=exc.code, validation_message=exc.message,
                last_validated_at_unix_sec=time.time())

    def on_list(self, request):
        maps = []
        for task_id in self.repository.list_task_ids():
            summary = self._summary(task_id, bool(request.include_invalid))
            if summary is None:
                continue
            if int(request.cage_id) >= 0 and summary.cage_id != int(request.cage_id):
                continue
            maps.append(summary)
        return ListScoutMapsResponse(success=True, reason_code='', message='', maps=maps)

    def on_lease(self, request):
        now = time.monotonic()
        ttl = min(30.0, max(1.0, float(request.requested_ttl_sec or self.default_lease_ttl)))
        with self.lock:
            self._prune_lease_locked()
            if int(request.operation) == ManageScoutMapLeaseRequest.OP_ACQUIRE:
                if self.active is None:
                    return ManageScoutMapLeaseResponse(
                        success=False, reason_code='NO_ACTIVE_MAP',
                        message='No active Scout map')
                requested = (str(request.map_uuid), int(request.map_version),
                             str(request.map_sha256), str(request.manifest_sha256))
                active = (self.active.map_uuid, self.active.map_version,
                          self.active.map_sha256, self.active.manifest_sha256)
                if requested != active:
                    return ManageScoutMapLeaseResponse(
                        success=False, reason_code='TASK_MAP_MISMATCH',
                        message='Requested map is not active')
                if self.lease is not None:
                    if (self.lease['feeding_task_id'] == request.feeding_task_id and
                            self.lease['pin'] == requested):
                        self.lease['expires_at'] = now + ttl
                        return ManageScoutMapLeaseResponse(
                            success=True, reason_code='', message='Lease renewed',
                            lease_id=self.lease['lease_id'], expires_in_sec=ttl)
                    return ManageScoutMapLeaseResponse(
                        success=False, reason_code='MAP_LEASE_CONFLICT',
                        message='Another Feeding task owns the active map lease')
                lease_id = str(uuid.uuid4())
                self.lease = {'lease_id': lease_id,
                              'feeding_task_id': str(request.feeding_task_id),
                              'pin': requested, 'expires_at': now + ttl}
                return ManageScoutMapLeaseResponse(
                    success=True, reason_code='', message='Lease acquired',
                    lease_id=lease_id, expires_in_sec=ttl)
            if int(request.operation) == ManageScoutMapLeaseRequest.OP_RENEW:
                if (self.lease is None or self.lease['lease_id'] != request.lease_id or
                        self.lease['feeding_task_id'] != request.feeding_task_id):
                    return ManageScoutMapLeaseResponse(
                        success=False, reason_code='MAP_LEASE_EXPIRED',
                        message='Map lease is missing or expired')
                self.lease['expires_at'] = now + ttl
                return ManageScoutMapLeaseResponse(
                    success=True, reason_code='', message='Lease renewed',
                    lease_id=self.lease['lease_id'], expires_in_sec=ttl)
            if int(request.operation) == ManageScoutMapLeaseRequest.OP_RELEASE:
                if self.lease is None:
                    return ManageScoutMapLeaseResponse(
                        success=True, reason_code='', message='Lease already released',
                        lease_id='', expires_in_sec=0.0)
                if (self.lease['lease_id'] != request.lease_id or
                        self.lease['feeding_task_id'] != request.feeding_task_id):
                    return ManageScoutMapLeaseResponse(
                        success=False, reason_code='MAP_LEASE_CONFLICT',
                        message='Lease belongs to another task')
                self.lease = None
                return ManageScoutMapLeaseResponse(
                    success=True, reason_code='', message='Lease released',
                    lease_id='', expires_in_sec=0.0)
        return ManageScoutMapLeaseResponse(
            success=False, reason_code='INVALID_LEASE_OPERATION',
            message='Unsupported map lease operation')

    def _restore_selector(self):
        if not os.path.isfile(self.selector_path):
            self.publish_status()
            return
        try:
            with open(self.selector_path, encoding='utf-8') as stream:
                selector = json.load(stream)
            value = self.repository.validate(selector['task_id'])
            actual = (value.map_uuid, value.map_version,
                      value.map_sha256, value.manifest_sha256)
            expected = (str(selector['map_uuid']), int(selector['map_version']),
                        str(selector['map_sha256']), str(selector['manifest_sha256']))
            if actual != expected or int(selector['cage_id']) != value.cage_id:
                raise ArtifactValidationError('MAP_HASH_MISMATCH',
                                              'selector no longer matches artifact bundle')
            activated_at = float(selector.get('activated_at_unix_sec', time.time()))
            with self.lock:
                self.active = value
                self.activated_at_unix_sec = activated_at
                self.active_message = self._validated_to_message(value, activated_at)
                self.last_status_code = ''
                self.last_status_message = 'Scout map restored from selector'
            self._publish_active()
        except (ArtifactValidationError, KeyError, TypeError, ValueError, OSError) as exc:
            with self.lock:
                self.active = None
                self.active_message = None
                self.last_status_code = getattr(exc, 'code', 'SELECTOR_INVALID')
                self.last_status_message = str(exc)
            self.publish_status()

    def _set_attempt_locked(self, task_id, success, code, message):
        self.last_attempt = {'task_id': str(task_id), 'at': time.time(),
                             'success': bool(success), 'reason_code': str(code),
                             'message': str(message)}

    def _set_attempt(self, task_id, success, code, message):
        with self.lock:
            self._set_attempt_locked(task_id, success, code, message)
        self.publish_status()

    def _append_audit(self, event, value, success, reason):
        payload = {
            'event': event, 'time': time.time(), 'success': bool(success),
            'reason': str(reason), 'task_id': value.task_id,
            'map_uuid': value.map_uuid, 'map_version': value.map_version,
            'map_sha256': value.map_sha256,
            'manifest_sha256': value.manifest_sha256}
        with open(self.audit_path, 'a', encoding='utf-8') as stream:
            stream.write(json.dumps(payload, sort_keys=True) + '\n')
            stream.flush()
            os.fsync(stream.fileno())

    def _publish_active(self):
        with self.lock:
            if self.active_message is not None:
                self.active_message.header.stamp = rospy.Time.now()
                self.map_pub.publish(self.active_message)
        self.publish_status()

    def publish_status(self):
        with self.lock:
            value = self.active
            attempt = dict(self.last_attempt)
            source_expired = bool(
                value and self.repository.max_snapshot_age_sec > 0.0 and
                time.time() - value.source_completed_at_unix_sec >
                self.repository.max_snapshot_age_sec)
            source_clock_invalid = bool(
                value and time.time() + 1e-6 < value.source_completed_at_unix_sec)
            if self.registry_error:
                status = ScoutMapActivationStatus.STATUS_REGISTRY_ERROR
            elif source_clock_invalid:
                status = ScoutMapActivationStatus.STATUS_INVALID
            elif source_expired:
                status = ScoutMapActivationStatus.STATUS_SOURCE_EXPIRED
            elif value is None and self.last_status_code not in ('', 'NO_ACTIVE_MAP'):
                status = ScoutMapActivationStatus.STATUS_INVALID
            elif value is None:
                status = ScoutMapActivationStatus.STATUS_NO_ACTIVE
            else:
                status = ScoutMapActivationStatus.STATUS_ACTIVE
            message = ScoutMapActivationStatus(
                header=Header(stamp=rospy.Time.now()), status=status,
                task_id=value.task_id if value else '',
                cage_id=value.cage_id if value else -1,
                map_uuid=value.map_uuid if value else '',
                map_version=value.map_version if value else 0,
                map_sha256=value.map_sha256 if value else '',
                manifest_sha256=value.manifest_sha256 if value else '',
                source_completed_at_unix_sec=(value.source_completed_at_unix_sec
                                              if value else 0.0),
                activated_at_unix_sec=self.activated_at_unix_sec if value else 0.0,
                last_validated_at_unix_sec=(value.last_validated_at_unix_sec
                                            if value else 0.0),
                confirmed_cells=value.confirmed_cells if value else 0,
                expected_cells=value.expected_cells if value else 0,
                coverage=value.coverage if value else 0.0,
                frozen=bool(value and value.frozen),
                compatible=bool(value and not self.registry_error and
                                not source_expired and not source_clock_invalid),
                legacy_reference_assumed=(value.legacy_reference_assumed
                                          if value else False),
                legacy_environment_assumed=(value.legacy_environment_assumed
                                            if value else False),
                reason_code=('CLOCK_INVALID' if source_clock_invalid else
                             ('SOURCE_MAP_EXPIRED' if source_expired
                              else self.last_status_code)),
                message=('System wall clock precedes Scout source completion time'
                         if source_clock_invalid else
                         ('Scout map exceeded source age policy' if source_expired
                          else self.last_status_message)),
                last_attempt_task_id=attempt['task_id'],
                last_attempt_at_unix_sec=attempt['at'],
                last_attempt_success=attempt['success'],
                last_attempt_reason_code=attempt['reason_code'],
                last_attempt_message=attempt['message'])
        self.status_pub.publish(message)

    def _heartbeat_loop(self):
        interval = 1.0 / max(0.1, self.heartbeat_rate)
        while not self.stop_event.wait(interval):
            if rospy.is_shutdown():
                return
            try:
                self.publish_status()
            except Exception as exc:
                rospy.logerr_throttle(2.0, 'Scout map heartbeat failed: %s', exc)

    def shutdown(self):
        self.stop_event.set()


if __name__ == '__main__':
    rospy.init_node('scout_map_registry_node')
    ScoutMapRegistryNode()
    rospy.spin()
