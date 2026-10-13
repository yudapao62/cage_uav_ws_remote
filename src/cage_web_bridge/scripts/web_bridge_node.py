#!/usr/bin/env python3
"""HTTP bridge for the independent Feeder and Scout task APIs.

The bridge deliberately publishes high-level ROS requests only. It never
talks to MAVROS and it never infers Scout completion in the browser or HTTP
layer; /scout/task_status is the single source of truth for task progress.
"""
import json
import math
import os
import re
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import rospy
from std_msgs.msg import Empty, Header, String
from cage_msgs.msg import (CageGeometry, FeederIdentityState,
                           FeedingTaskRequest, FeedingTaskStatus, MissionState,
                           OffsetHypothesisArray, PersistentLattice,
                           ScoutMapActivationStatus, ScoutTaskRequest,
                           ScoutTaskStatus)
from cage_msgs.srv import ActivateScoutMap, ListScoutMaps


SCOUT_TERMINAL_STATES = frozenset((
    'IDLE', 'COMPLETED', 'FAILED', 'CANCELED', 'CANCELLED'))
FEEDING_INACTIVE_STATES = frozenset((
    'TASK_IDLE', 'TASK_PREVIEW', 'TASK_COMPLETED', 'TASK_ERROR',
    'TASK_CANCELLED', 'TASK_CANCELED', 'TASK_ABORTED_TIMEOUT'))
MISSION_AVAILABLE_STATES = frozenset(('IDLE', 'FINISHED'))
TASK_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$')
SCOUT_FIELDS = frozenset((
    'task_id', 'cage_id', 'survey_relative_alt', 'auto_rtl', 'auto_land'))


class ApiError(Exception):
    """Expected client/API failure with stable HTTP and machine codes."""

    def __init__(self, status, code, message):
        super().__init__(message)
        self.status = int(status)
        self.code = str(code)
        self.message = str(message)

    def body(self):
        return {'accepted': False, 'error': self.message,
                'error_code': self.code}


def _state(value):
    return str(value or '').strip().upper()


class WebBridge:
    def __init__(self):
        self.geometry_mode = str(
            rospy.get_param('/cage/geometry_mode', 'rectangular')).strip().lower()
        if self.geometry_mode not in ('rectangular', 'circular'):
            raise ValueError('invalid /cage/geometry_mode: {}'.format(
                self.geometry_mode))

        self._lock = threading.RLock()
        self.status = FeedingTaskStatus(task_state='TASK_IDLE')
        self.mission = MissionState(state='WAIT_FCU')
        self.scout_status = ScoutTaskStatus(
            state='IDLE', cage_id=0,
            expected_cells=int(rospy.get_param(
                '/scout_mission/expected_cells', 60)),
            disarmed=True)
        self._pending_scout = None
        self._scout_cancel_requested_for = ''
        self._used_scout_task_ids = set()

        self.beacon = {}
        self.precheck = {}
        self.geometry = None
        self.lattice = None
        self.working_lattice = None
        self.active_map_status = ScoutMapActivationStatus(
            status=ScoutMapActivationStatus.STATUS_NO_ACTIVE)
        self.offsets = None
        self.feeder_identity = None

        configured_cages = rospy.get_param(
            '/scout_mission/targetable_cages',
            rospy.get_param('/web_bridge/scout/targetable_cages', [0]))
        try:
            self.targetable_cages = frozenset(
                int(value) for value in configured_cages)
        except (TypeError, ValueError):
            raise ValueError('targetable Scout cages must be a list of integers')
        if not self.targetable_cages:
            raise ValueError('at least one targetable Scout cage is required')
        self.default_scout_alt = float(rospy.get_param(
            '/scout_mission/survey_relative_alt_m',
            rospy.get_param(
                '/web_bridge/scout/default_survey_relative_alt_m', 25.0)))
        self.minimum_scout_alt = float(rospy.get_param(
            '/web_bridge/scout/minimum_survey_relative_alt_m', 1.0))
        self.maximum_scout_alt = float(rospy.get_param(
            '/web_bridge/scout/maximum_survey_relative_alt_m', 120.0))

        # Existing Feeding API publishers remain unchanged.
        self.preview_pub = rospy.Publisher(
            '/feeding/task_preview', FeedingTaskRequest, queue_size=1)
        self.start_pub = rospy.Publisher(
            '/feeding/task_request', FeedingTaskRequest, queue_size=1)
        self.cancel_pub = rospy.Publisher(
            '/feeding/task_cancel', Empty, queue_size=1)

        # Scout is an independent high-level API. No MAVROS service is used here.
        self.scout_start_pub = rospy.Publisher(
            '/scout/task_request', ScoutTaskRequest, queue_size=1)
        self.scout_cancel_pub = rospy.Publisher(
            '/scout/task_cancel', String, queue_size=1)

        rospy.Subscriber('/feeding/task_status', FeedingTaskStatus,
                         self.on_feeding_status, queue_size=1)
        rospy.Subscriber('/mission/state', MissionState,
                         self.on_mission, queue_size=1)
        rospy.Subscriber('/scout/task_status', ScoutTaskStatus,
                         self.on_scout_status, queue_size=1)
        rospy.Subscriber('/cage/beacon/status', String,
                         self.on_beacon, queue_size=1)
        rospy.Subscriber('/cage/realflight/precheck', String,
                         self.on_precheck, queue_size=1)
        rospy.Subscriber('/cage/geometry', CageGeometry,
                         lambda msg: setattr(self, 'geometry', msg), queue_size=1)
        rospy.Subscriber('/scout/lattice_map', PersistentLattice,
                         lambda msg: setattr(self, 'lattice', msg), queue_size=1)
        rospy.Subscriber('/scout/working_lattice_map', PersistentLattice,
                         lambda msg: setattr(self, 'working_lattice', msg), queue_size=1)
        rospy.Subscriber('/scout/active_map_status', ScoutMapActivationStatus,
                         lambda msg: setattr(self, 'active_map_status', msg), queue_size=1)
        rospy.Subscriber('/scout/offset_hypotheses', OffsetHypothesisArray,
                         lambda msg: setattr(self, 'offsets', msg), queue_size=1)
        rospy.Subscriber('/feeder/identity_state', FeederIdentityState,
                         lambda msg: setattr(self, 'feeder_identity', msg),
                         queue_size=1)
        self.list_maps_service = rospy.ServiceProxy('/scout/list_maps', ListScoutMaps)
        self.activate_map_service = rospy.ServiceProxy('/scout/activate_map', ActivateScoutMap)

    @staticmethod
    def map_summary_json(value):
        return {
            'task_id': value.task_id, 'cage_id': value.cage_id,
            'map_uuid': value.map_uuid, 'map_version': value.map_version,
            'map_sha256': value.map_sha256,
            'manifest_sha256': value.manifest_sha256,
            'source_completed_at_unix_sec': value.source_completed_at_unix_sec,
            'confirmed_cells': value.confirmed_cells,
            'expected_cells': value.expected_cells,
            'coverage': value.coverage, 'frozen': value.frozen,
            'validation_state': value.validation_state,
            'compatibility_state': value.compatibility_state,
            'active': value.active,
            'legacy_reference_assumed': value.legacy_reference_assumed,
            'legacy_environment_assumed': value.legacy_environment_assumed,
            'last_validated_at_unix_sec': value.last_validated_at_unix_sec,
            'validation_code': value.validation_code,
            'validation_message': value.validation_message,
        }

    def list_scout_maps(self, cage_id=-1, include_invalid=False):
        try:
            rospy.wait_for_service('/scout/list_maps', timeout=2.0)
            response = self.list_maps_service(
                cage_id=int(cage_id), include_invalid=bool(include_invalid))
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise ApiError(503, 'MAP_REGISTRY_UNAVAILABLE', str(exc))
        if not response.success:
            raise ApiError(503, response.reason_code or 'MAP_LIST_FAILED', response.message)
        return {'maps': [self.map_summary_json(value) for value in response.maps]}

    def active_scout_map_json(self):
        value = self.active_map_status
        return {
            'status': int(value.status), 'task_id': value.task_id,
            'cage_id': value.cage_id, 'map_uuid': value.map_uuid,
            'map_version': value.map_version, 'map_sha256': value.map_sha256,
            'manifest_sha256': value.manifest_sha256,
            'source_completed_at_unix_sec': value.source_completed_at_unix_sec,
            'activated_at_unix_sec': value.activated_at_unix_sec,
            'last_validated_at_unix_sec': value.last_validated_at_unix_sec,
            'confirmed_cells': value.confirmed_cells,
            'expected_cells': value.expected_cells, 'coverage': value.coverage,
            'frozen': value.frozen, 'compatible': value.compatible,
            'legacy_reference_assumed': value.legacy_reference_assumed,
            'legacy_environment_assumed': value.legacy_environment_assumed,
            'reason_code': value.reason_code, 'message': value.message,
            'last_attempt': {
                'task_id': value.last_attempt_task_id,
                'at_unix_sec': value.last_attempt_at_unix_sec,
                'success': value.last_attempt_success,
                'reason_code': value.last_attempt_reason_code,
                'message': value.last_attempt_message,
            },
        }

    def activate_scout_map(self, payload):
        required = {'task_id', 'cage_id', 'expected_map_uuid',
                    'expected_map_version', 'expected_map_sha256',
                    'expected_manifest_sha256'}
        if not isinstance(payload, dict) or set(payload) != required:
            raise ApiError(400, 'INVALID_MAP_ACTIVATION_REQUEST',
                           'Map activation requires exactly {}'.format(', '.join(sorted(required))))
        if not isinstance(payload['task_id'], str) or not TASK_ID_PATTERN.fullmatch(payload['task_id']):
            raise ApiError(400, 'INVALID_TASK_ID', 'Invalid Scout task_id')
        for key in ('expected_map_sha256', 'expected_manifest_sha256'):
            value = payload[key]
            if (not isinstance(value, str) or len(value) != 64 or
                    any(char not in '0123456789abcdef' for char in value)):
                raise ApiError(400, 'INVALID_HASH', '{} must be lowercase SHA-256'.format(key))
        try:
            rospy.wait_for_service('/scout/activate_map', timeout=2.0)
            response = self.activate_map_service(**payload)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise ApiError(503, 'MAP_REGISTRY_UNAVAILABLE', str(exc))
        if not response.success:
            conflict = response.reason_code in (
                'SCOUT_BUSY', 'FEEDING_BUSY', 'MISSION_BUSY', 'FEEDER_ACTIVE',
                'MAP_SWITCH_BUSY', 'MAP_LEASE_CONFLICT', 'MAP_UUID_MISMATCH',
                'MAP_VERSION_MISMATCH', 'MAP_HASH_MISMATCH', 'MANIFEST_HASH_MISMATCH')
            missing = response.reason_code in ('ARTIFACT_NOT_FOUND', 'MANIFEST_MISSING')
            raise ApiError(404 if missing else (409 if conflict else 422),
                           response.reason_code or 'MAP_ACTIVATION_FAILED', response.message)
        return {
            'success': True, 'committed': response.committed,
            'idempotent': response.idempotent, 'task_id': payload['task_id'],
            'cage_id': payload['cage_id'], 'map_uuid': response.map_uuid,
            'map_version': response.map_version,
            'map_sha256': response.map_sha256,
            'manifest_sha256': response.manifest_sha256,
            'confirmed_cells': response.confirmed_cells,
            'expected_cells': response.expected_cells,
            'coverage': response.coverage, 'active': response.committed,
            'legacy_reference_assumed': response.legacy_reference_assumed,
            'legacy_environment_assumed': response.legacy_environment_assumed,
            'reason_code': response.reason_code, 'message': response.message,
        }

    def on_feeding_status(self, msg):
        with self._lock:
            self.status = msg

    def on_mission(self, msg):
        with self._lock:
            self.mission = msg

    def on_scout_status(self, msg):
        with self._lock:
            self.scout_status = msg
            task_id = str(msg.task_id or '')
            if task_id:
                self._used_scout_task_ids.add(task_id)
            if (self._pending_scout and
                    task_id == self._pending_scout['task_id']):
                # The latched manager status now owns the public state.
                self._pending_scout = None
            if _state(msg.state) in SCOUT_TERMINAL_STATES:
                self._scout_cancel_requested_for = ''

    def on_beacon(self, msg):
        try:
            self.beacon = json.loads(msg.data)
        except (ValueError, TypeError):
            self.beacon = {}

    def on_precheck(self, msg):
        try:
            self.precheck = json.loads(msg.data)
        except (ValueError, TypeError):
            self.precheck = {}

    def make_request(self, payload, auto_start):
        mode = ('CENTER_FEED' if self.geometry_mode == 'circular' else
                str(payload.get('mode', 'FULL_GRID')).upper())
        return FeedingTaskRequest(
            header=Header(stamp=rospy.Time.now()),
            task_id=str(payload.get('task_id') or 'web-task'),
            cage_id=int(payload.get('cage_id', 0)),
            mode=mode,
            rows=[] if self.geometry_mode == 'circular' else [
                int(value) for value in payload.get('rows', [])],
            cols=[] if self.geometry_mode == 'circular' else [
                int(value) for value in payload.get('cols', [])],
            selected_cell_ids=[] if self.geometry_mode == 'circular' else [
                str(value).upper()
                for value in payload.get('selected_cell_ids', [])],
            repeat_count=int(payload.get('repeat_count', 1)),
            auto_start=bool(auto_start),
            feed_relative_alt=float(payload.get(
                'feed_relative_alt',
                rospy.get_param('/feeding/feed_relative_alt', 15.0))),
            transit_relative_alt=float(payload.get(
                'transit_relative_alt',
                rospy.get_param('/feeding/transit_relative_alt', 25.0))))

    def validate_scout_payload(self, payload):
        if not isinstance(payload, dict):
            raise ApiError(400, 'INVALID_JSON_OBJECT',
                           'Scout request body must be a JSON object')
        unknown = sorted(set(payload) - SCOUT_FIELDS)
        if unknown:
            raise ApiError(400, 'UNKNOWN_FIELD',
                           'Unknown Scout field(s): {}'.format(
                               ', '.join(unknown)))
        missing = sorted(SCOUT_FIELDS - set(payload))
        if missing:
            raise ApiError(400, 'MISSING_FIELD',
                           'Missing Scout field(s): {}'.format(
                               ', '.join(missing)))

        task_id = payload.get('task_id')
        if (not isinstance(task_id, str) or
                not TASK_ID_PATTERN.fullmatch(task_id)):
            raise ApiError(
                400, 'INVALID_TASK_ID',
                'task_id must be 1-128 safe characters: letters, digits, ., _ or -')

        cage_id = payload.get('cage_id')
        if isinstance(cage_id, bool) or not isinstance(cage_id, int):
            raise ApiError(400, 'INVALID_CAGE_ID',
                           'cage_id must be an integer')
        if cage_id not in self.targetable_cages:
            raise ApiError(
                409, 'CAGE_NOT_TARGETABLE',
                'CAGE_{} is not targetable; phase one only supports CAGE_0'.format(
                    cage_id))

        altitude = payload.get('survey_relative_alt')
        if (isinstance(altitude, bool) or
                not isinstance(altitude, (int, float)) or
                not math.isfinite(float(altitude))):
            raise ApiError(400, 'INVALID_SURVEY_ALTITUDE',
                           'survey_relative_alt must be a finite number')
        altitude = float(altitude)
        if not self.minimum_scout_alt <= altitude <= self.maximum_scout_alt:
            raise ApiError(
                400, 'INVALID_SURVEY_ALTITUDE',
                'survey_relative_alt must be in [{:.1f}, {:.1f}] m'.format(
                    self.minimum_scout_alt, self.maximum_scout_alt))

        for key in ('auto_rtl', 'auto_land'):
            if not isinstance(payload.get(key), bool):
                raise ApiError(400, 'INVALID_{}'.format(key.upper()),
                               '{} must be a JSON boolean'.format(key))
            if payload[key] is not True:
                raise ApiError(400, '{}_REQUIRED'.format(key.upper()),
                               '{} must be true in phase one'.format(key))

        return {
            'task_id': task_id,
            'cage_id': cage_id,
            'survey_relative_alt': altitude,
            'auto_rtl': True,
            'auto_land': True,
        }

    def _scout_active_locked(self):
        if self._pending_scout is not None:
            return True
        return _state(self.scout_status.state) not in SCOUT_TERMINAL_STATES

    def _feeding_active_locked(self):
        return _state(self.status.task_state) not in FEEDING_INACTIVE_STATES

    def _mission_available_locked(self):
        return _state(self.mission.state) in MISSION_AVAILABLE_STATES

    def _effective_scout_locked(self):
        if self._pending_scout is not None:
            return {
                'task_id': self._pending_scout['task_id'],
                'cage_id': self._pending_scout['cage_id'],
                'state': 'PRECHECK',
                'pending': True,
            }
        return {
            'task_id': str(self.scout_status.task_id or ''),
            'cage_id': int(self.scout_status.cage_id),
            'state': _state(self.scout_status.state) or 'IDLE',
            'pending': False,
        }

    def start_scout(self, payload):
        request = self.validate_scout_payload(payload)
        task_id = request['task_id']
        with self._lock:
            effective = self._effective_scout_locked()
            if task_id in self._used_scout_task_ids:
                if (effective['task_id'] == task_id and
                        self._scout_active_locked()):
                    # A retry/double click gets the original task and never
                    # publishes a second ROS request.
                    return {
                        'accepted': True,
                        'idempotent': True,
                        'task_id': task_id,
                        'state': effective['state'],
                    }
                raise ApiError(409, 'TASK_ID_ALREADY_USED',
                               'task_id has already been used')
            if self._scout_active_locked():
                raise ApiError(409, 'SCOUT_TASK_BUSY',
                               'Another Scout task is already active')
            if self._feeding_active_locked():
                raise ApiError(409, 'FEEDING_TASK_BUSY',
                               'A Feeding task is active')
            if _state(self.mission.state) == 'MANUAL_OVERRIDE':
                raise ApiError(409, 'MANUAL_OVERRIDE_ACTIVE',
                               'Manual control is active; return authority first')
            if not self._mission_available_locked():
                raise ApiError(409, 'MISSION_NOT_READY',
                               'Mission controller is not idle')

            self._used_scout_task_ids.add(task_id)
            self._pending_scout = dict(request)
            message = ScoutTaskRequest(
                header=Header(stamp=rospy.Time.now()),
                task_id=task_id,
                cage_id=request['cage_id'],
                survey_relative_alt=request['survey_relative_alt'],
                auto_rtl=True,
                auto_land=True)
            try:
                self.scout_start_pub.publish(message)
            except Exception as exc:
                self._pending_scout = None
                self._used_scout_task_ids.discard(task_id)
                raise ApiError(503, 'SCOUT_REQUEST_PUBLISH_FAILED', str(exc))

        return {
            'accepted': True,
            'idempotent': False,
            'task_id': task_id,
            'state': 'PRECHECK',
        }

    def cancel_scout(self, payload=None):
        payload = {} if payload is None else payload
        if not isinstance(payload, dict):
            raise ApiError(400, 'INVALID_JSON_OBJECT',
                           'Scout cancel body must be a JSON object')
        unknown = sorted(set(payload) - {'task_id'})
        if unknown:
            raise ApiError(400, 'UNKNOWN_FIELD',
                           'Unknown Scout cancel field(s): {}'.format(
                               ', '.join(unknown)))
        with self._lock:
            effective = self._effective_scout_locked()
            task_id = effective['task_id']
            if not self._scout_active_locked() or not task_id:
                raise ApiError(409, 'NO_ACTIVE_SCOUT_TASK',
                               'There is no active Scout task to cancel')
            requested_id = payload.get('task_id')
            if requested_id is not None and requested_id != task_id:
                raise ApiError(409, 'SCOUT_TASK_ID_MISMATCH',
                               'Cancel task_id does not match the active Scout task')
            if self._scout_cancel_requested_for == task_id:
                return {
                    'accepted': True,
                    'idempotent': True,
                    'task_id': task_id,
                    'state': effective['state'],
                    'message': 'Safe cancel and RTL already requested',
                }
            try:
                self.scout_cancel_pub.publish(String(data=task_id))
            except Exception as exc:
                raise ApiError(503, 'SCOUT_CANCEL_PUBLISH_FAILED', str(exc))
            self._scout_cancel_requested_for = task_id
        return {
            'accepted': True,
            'idempotent': False,
            'task_id': task_id,
            'state': effective['state'],
            'message': 'Safe cancel accepted; the backend will RTL and land',
        }

    def assert_feeding_allowed(self):
        with self._lock:
            if self._scout_active_locked():
                raise ApiError(409, 'SCOUT_TASK_BUSY',
                               'Feeding cannot start while Scout is active')

    def scout_task_json(self, map_age, offset):
        with self._lock:
            effective = self._effective_scout_locked()
            pending = effective['pending']
            status = self.scout_status
            state = effective['state']
            active = self._scout_active_locked()
            can_start = (not active and not self._feeding_active_locked() and
                         self._mission_available_locked())
            if pending:
                task_id = effective['task_id']
                cage_id = effective['cage_id']
                current_waypoint = 0
                total_waypoints = 0
                route_completed = False
                map_uuid = ''
                map_version = 0
                confirmed_cells = 0
                expected_cells = int(rospy.get_param(
                    '/scout_mission/expected_cells', 60))
                coverage = 0.0
                map_finalized = False
                map_ready = False
                rtl_commanded = False
                rtl_accepted = False
                returned_home = False
                home_error = 0.0
                landed = False
                disarmed = True
                reason = ''
                error_code = ''
                output_directory = ''
                mission_id = ''
                geometry_sha256 = ''
            else:
                task_id = str(status.task_id or '')
                cage_id = int(status.cage_id)
                current_waypoint = int(status.current_waypoint)
                total_waypoints = int(status.total_waypoints)
                route_completed = bool(status.route_completed)
                map_uuid = str(status.map_uuid or '')
                map_version = int(status.map_version)
                confirmed_cells = int(status.confirmed_cells)
                expected_cells = int(status.expected_cells)
                coverage = float(status.coverage)
                map_finalized = bool(status.map_finalized)
                map_ready = bool(status.map_ready_for_feeding)
                rtl_commanded = bool(status.rtl_commanded)
                rtl_accepted = bool(status.rtl_accepted)
                returned_home = bool(status.returned_home)
                home_error = float(status.home_horizontal_error_m)
                landed = bool(status.landed)
                disarmed = bool(status.disarmed)
                reason = str(status.reason or '')
                error_code = str(status.error_code or '')
                output_directory = str(status.output_directory or '')
                mission_id = str(status.mission_id or '')
                geometry_sha256 = str(status.geometry_sha256 or '')
        return {
            'can_start': can_start,
            'can_cancel': active,
            'task_id': task_id,
            'mission_id': mission_id,
            'cage_id': cage_id,
            'state': state,
            'waypoints': {
                'reached': current_waypoint,
                'total': total_waypoints,
            },
            'map': {
                'uuid': map_uuid,
                'version': map_version,
                'geometry_sha256': geometry_sha256,
                'confirmed': confirmed_cells,
                'expected': expected_cells,
                'coverage': coverage,
                'age_sec': map_age,
                'ready_for_feeding': map_ready,
                'offset_top1': offset,
            },
            'route_completed': route_completed,
            'map_finalized': map_finalized,
            'rtl_commanded': rtl_commanded,
            'rtl_accepted': rtl_accepted,
            'returned_home': returned_home,
            'home_horizontal_error_m': home_error,
            'landed': landed,
            'disarmed': disarmed,
            'reason': reason,
            'error_code': error_code,
            'output_directory': output_directory,
        }

    def status_json(self):
        target = None
        if self.geometry_mode == 'circular' and self.geometry is not None:
            target = {
                'lat': self.geometry.center_lat,
                'lon': self.geometry.center_lon,
                'diameter_m': self.geometry.diameter_m,
                'radius_m': self.geometry.radius_m,
            }
        map_age = None
        if self.lattice is not None and self.lattice.source_completed_at_unix_sec > 0.0:
            map_age = max(0.0, time.time() - self.lattice.source_completed_at_unix_sec)
        offset = None
        if self.offsets is not None:
            top = (self.offsets.hypotheses[0]
                   if self.offsets.hypotheses else None)
            offset = {
                'committed': self.offsets.committed,
                'delta_row': self.offsets.committed_delta_row,
                'delta_col': self.offsets.committed_delta_col,
                'confidence': top.posterior if top else 0.0,
                'reason': self.offsets.decision_reason,
            }
        feeder = None
        if self.feeder_identity is not None:
            task_has_map = bool(
                self.feeder_identity.task_map_uuid or
                self.feeder_identity.task_map_version or
                self.feeder_identity.task_map_sha256 or
                self.feeder_identity.task_manifest_sha256)
            labels = {
                FeederIdentityState.STATUS_CONFIRMED: 'CONFIRMED',
                FeederIdentityState.STATUS_AMBIGUOUS: 'AMBIGUOUS',
                FeederIdentityState.STATUS_LOST: 'LOST',
                FeederIdentityState.STATUS_STALE_MAP: 'STALE_MAP',
                FeederIdentityState.STATUS_MAP_MISMATCH: 'MAP_MISMATCH',
            }
            feeder = {
                'status': labels.get(self.feeder_identity.status, 'UNKNOWN'),
                'previous_cell_id': self.feeder_identity.previous_cell_id,
                'current_cell_id': self.feeder_identity.current_cell_id,
                'next_cell_id': self.feeder_identity.next_cell_id,
                'top_candidates': [
                    {'cell_id': value.cell_id, 'posterior': value.posterior}
                    for value in self.feeder_identity.hypotheses[:3]],
                'reason': self.feeder_identity.decision_reason,
                'map_version': self.feeder_identity.map_version,
                'map_uuid': self.feeder_identity.map_uuid,
                'map_sha256': self.feeder_identity.map_sha256,
                'task_map_version': self.feeder_identity.task_map_version,
                'task_map_uuid': self.feeder_identity.task_map_uuid,
                'task_uses_old_map': task_has_map and (
                    self.feeder_identity.task_map_version !=
                    self.feeder_identity.map_version or
                    self.feeder_identity.task_map_uuid != self.feeder_identity.map_uuid or
                    self.feeder_identity.task_map_sha256 != self.feeder_identity.map_sha256 or
                    self.feeder_identity.task_manifest_sha256 !=
                    self.feeder_identity.manifest_sha256),
            }
        return {
            # Existing Feeding response fields remain at their original paths.
            'geometry_mode': self.geometry_mode,
            'task_mode': ('CENTER_FEED'
                          if self.geometry_mode == 'circular'
                          else self.status.mode),
            'feeding_target': target,
            'feeding_dwell_sec': (
                rospy.get_param('/cage/circular/feeding_dwell_sec', 5.0)
                if self.geometry_mode == 'circular' else None),
            'task_state': self.status.task_state,
            'task_id': self.status.task_id,
            'mode': self.status.mode,
            'cage_id': self.status.cage_id,
            'current_repeat': self.status.current_repeat,
            'total_repeat': self.status.total_repeat,
            'completed_targets': self.status.completed_targets,
            'total_targets': self.status.total_targets,
            'current_cell_id': self.status.current_cell_id,
            'message': self.status.message,
            'fsm': self.mission.state,
            'manual_override': self.mission.manual_override,
            'real_flight': bool(rospy.get_param(
                '/web_bridge/real_flight', False)),
            'fcu_label': rospy.get_param('/web_bridge/fcu_label', ''),
            'auto_arm': bool(rospy.get_param('/vehicle/auto_arm', False)),
            'beacon': self.beacon,
            'precheck': self.precheck,
            'persistent_lattice': {
                'map_version': self.lattice.map_version if self.lattice else 0,
                'map_uuid': self.lattice.map_uuid if self.lattice else '',
                'map_sha256': self.lattice.map_sha256 if self.lattice else '',
                'manifest_sha256': self.lattice.manifest_sha256 if self.lattice else '',
                'map_age_sec': map_age,
                'confirmed_cells': (sum(
                    1 for value in self.lattice.cells
                    if value.identity_state == value.IDENTITY_CONFIRMED)
                    if self.lattice else 0),
                'scout_offset': offset,
                'feeder': feeder,
                'active_map': self.active_scout_map_json(),
            },
            'scout_task': self.scout_task_json(map_age, offset),
        }

    def grid_json(self):
        if self.geometry_mode == 'circular':
            return {
                'geometry_mode': 'circular',
                'cage_id': int(rospy.get_param('/cage_grid/cage_id', 0)),
                'rows': 0,
                'cols': 0,
                'cell_ids': [],
            }
        rows = int(rospy.get_param('/cage_grid/rows', 6))
        cols = int(rospy.get_param('/cage_grid/cols', 10))
        confirmed = None
        if self.lattice is not None:
            confirmed = {
                cell.cell_id for cell in self.lattice.cells
                if cell.identity_state == cell.IDENTITY_CONFIRMED}
        ids = [
            'R{:02d}C{:02d}'.format(row, col)
            for row in range(1, rows + 1)
            for col in range(1, cols + 1)]
        return {
            'geometry_mode': 'rectangular',
            'cage_id': int(rospy.get_param('/cage_grid/cage_id', 0)),
            'rows': rows,
            'cols': cols,
            'cell_ids': ids,
            'confirmed_cell_ids': (
                sorted(confirmed) if confirmed is not None else []),
        }


def handler_factory(bridge, web_root):
    class Handler(BaseHTTPRequestHandler):
        MAX_BODY_BYTES = 64 * 1024

        def send_json(self, value, code=200):
            body = json.dumps(value, ensure_ascii=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type',
                             'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def read_json(self, optional=False):
            raw_length = self.headers.get('Content-Length', '0')
            try:
                length = int(raw_length)
            except (TypeError, ValueError):
                raise ApiError(400, 'INVALID_CONTENT_LENGTH',
                               'Invalid Content-Length')
            if length < 0 or length > self.MAX_BODY_BYTES:
                raise ApiError(400, 'REQUEST_BODY_TOO_LARGE',
                               'JSON request body is too large')
            if length == 0:
                if optional:
                    return {}
                raise ApiError(400, 'MISSING_JSON_BODY',
                               'A JSON request body is required')
            try:
                value = json.loads(self.rfile.read(length).decode('utf-8'))
            except (UnicodeDecodeError, ValueError):
                raise ApiError(400, 'INVALID_JSON',
                               'Malformed JSON request body')
            if not isinstance(value, dict):
                raise ApiError(400, 'INVALID_JSON_OBJECT',
                               'Request body must be a JSON object')
            return value

        def do_GET(self):
            parsed = urlsplit(self.path)
            path = parsed.path
            if path == '/api/status':
                return self.send_json(bridge.status_json())
            if path == '/api/grid':
                return self.send_json(bridge.grid_json())
            if path == '/api/scout/maps/active':
                return self.send_json(bridge.active_scout_map_json())
            if path == '/api/scout/maps':
                try:
                    query = parse_qs(parsed.query)
                    cage_id = int(query.get('cage_id', ['-1'])[0])
                    include_invalid = query.get('include_invalid', ['false'])[0].lower() == 'true'
                    return self.send_json(bridge.list_scout_maps(cage_id, include_invalid))
                except ApiError as exc:
                    return self.send_json(exc.body(), exc.status)
                except (TypeError, ValueError) as exc:
                    return self.send_json(ApiError(400, 'INVALID_QUERY', str(exc)).body(), 400)
            if path not in ('/', '/index.html'):
                return self.send_json({'error': 'not found'}, 404)
            index_path = os.path.join(web_root, 'index.html')
            try:
                with open(index_path, 'rb') as stream:
                    body = stream.read()
            except OSError as exc:
                return self.send_json({'error': str(exc)}, 500)
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            path = urlsplit(self.path).path
            try:
                if path == '/api/scout/start':
                    return self.send_json(
                        bridge.start_scout(self.read_json()), 202)
                if path == '/api/scout/cancel':
                    return self.send_json(
                        bridge.cancel_scout(self.read_json(optional=True)), 202)
                if path == '/api/scout/maps/activate':
                    return self.send_json(
                        bridge.activate_scout_map(self.read_json()), 200)
                if path not in ('/api/preview', '/api/start', '/api/cancel'):
                    return self.send_json({'error': 'not found'}, 404)

                payload = self.read_json(optional=True)
                if path == '/api/preview':
                    bridge.assert_feeding_allowed()
                    beacon_fresh = (
                        rospy.Time.now().to_sec() -
                        float(bridge.beacon.get('published_at', 0)) <=
                        float(rospy.get_param(
                            '/beacon/receive_timeout_sec', 2.0)))
                    if (rospy.get_param('/web_bridge/real_flight', False) and
                            (not bridge.beacon.get('geometry_valid', False) or
                             not beacon_fresh)):
                        raise ApiError(409, 'BEACON_GEOMETRY_INVALID',
                                       'Beacon geometry is invalid')
                    bridge.preview_pub.publish(
                        bridge.make_request(payload, False))
                    return self.send_json({'ok': True})
                if path == '/api/start':
                    bridge.assert_feeding_allowed()
                    if (rospy.get_param('/web_bridge/real_flight', False) and
                            not rospy.get_param(
                                '/realflight/start_ready', False)):
                        raise ApiError(409, 'REALFLIGHT_PRECHECK_FAILED',
                                       'Realflight precheck failed')
                    bridge.start_pub.publish(
                        bridge.make_request(payload, True))
                    return self.send_json({'ok': True})
                bridge.cancel_pub.publish(Empty())
                return self.send_json({'ok': True})
            except ApiError as exc:
                return self.send_json(exc.body(), exc.status)
            except (TypeError, ValueError) as exc:
                return self.send_json(
                    ApiError(400, 'INVALID_FIELD', str(exc)).body(), 400)
            except Exception as exc:
                rospy.logerr('web_bridge request failed: %s', exc)
                return self.send_json(
                    ApiError(500, 'INTERNAL_ERROR',
                             'Internal server error').body(), 500)

        def log_message(self, fmt, *args):
            rospy.loginfo('web_bridge ' + fmt, *args)

    return Handler


if __name__ == '__main__':
    rospy.init_node('web_bridge_node')
    bridge = WebBridge()
    web_root = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'www')
    host = rospy.get_param('/web_bridge/host', '0.0.0.0')
    requested_port = int(rospy.get_param('/web_bridge/port', 8080))
    for candidate in range(requested_port, requested_port + 20):
        try:
            server = ThreadingHTTPServer(
                (host, candidate), handler_factory(bridge, web_root))
            selected_port = candidate
            break
        except OSError as exc:
            if exc.errno not in (48, 98):
                raise
    else:
        raise socket.error('no available web bridge port in {}..{}'.format(
            requested_port, requested_port + 19))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rospy.loginfo(
        'Cage Scout + Feeding Web UI listening on http://%s:%d',
        host, selected_port)
    rospy.spin()
    server.shutdown()
