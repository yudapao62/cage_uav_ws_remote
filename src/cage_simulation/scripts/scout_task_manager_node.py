#!/usr/bin/env python3
"""Request-driven Scout task coordinator.

This node owns task/session state and artifacts, but deliberately never calls a
MAVROS service.  It publishes the generic mission contract and leaves every
FCU write to mission_fsm_node + MavrosAdapter.
"""
import hashlib
import json
import math
import os
import re
import signal
import subprocess
import tempfile
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header, String

from cage_msgs.msg import (
    FeedingTaskStatus,
    LocalLatticeObservation,
    MissionRequest,
    MissionState,
    MissionWaypoint,
    MissionWaypointArray,
    PersistentLattice,
    ScoutTaskRequest,
    ScoutTaskStatus,
)
from cage_msgs.srv import ActivateScoutMap, FinalizeScoutMap, ResetScoutMap
from cage_waypoint_planner.scout_planner import ScoutRouteConfig, plan_scout_mission


ACTIVE_FEEDING_STATES = {
    'TASK_READY', 'TASK_EXECUTING', 'TASK_REPEAT_WAIT', 'TASK_MANUAL_OVERRIDE',
}
STARTABLE_MISSION_STATES = {'IDLE', 'FINISHED'}
TERMINAL_TASK_STATES = {'COMPLETED', 'FAILED', 'CANCELED'}
TASK_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$')


def _atomic_json(path, value):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix='.scout_', suffix='.json', dir=directory)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _safe_set(message, name, value):
    if hasattr(message, name):
        setattr(message, name, value)


class ScoutTaskManager:
    def __init__(self):
        self.lock = threading.RLock()
        self.state = 'IDLE'
        self.reason = ''
        self.error_code = ''
        self.request = None
        self.mission_id = ''
        self.output_directory = ''
        self.started_wall = 0.0
        self.started_ros = rospy.Time(0)
        self.precheck_deadline = 0.0
        self.finalize_requested_at = 0.0
        self.finalize_attempted = False
        self.plan_published = False
        self.cancel_requested = False
        self.route = []
        self.route_completed = False
        self.map_finalized = False
        self.map_ready_for_feeding = False
        self.mapping_outcome = 'PENDING'
        self.flight_outcome = 'PENDING'
        self.snapshot_path = ''
        self.home_gps = None
        self.home_pose = None
        self.home_distance = float('inf')
        self.returned_home = False
        self.landed = False
        self.disarmed = True
        self.disarmed_since = None
        self.rtl_commanded = False
        self.rtl_accepted = False
        self.latest_map = None
        self.map_session_started = False
        self.fcu = State()
        self.extended = ExtendedState()
        self.gps = None
        self.pose = None
        self.mission = MissionState(state='WAIT_FCU')
        self.feeding = FeedingTaskStatus(task_state='TASK_IDLE')
        self.control_owner = 'AUTO'
        self.last_gps = self.last_pose = self.last_observation = 0.0
        self.event_stream = None
        self.bag_process = None
        self.auto_start_sent = False
        self.node_started = time.monotonic()

        prefix = '/scout_mission/'
        p = lambda name, default: rospy.get_param(prefix + name, default)
        self.targetable_cages = {int(value) for value in p('targetable_cages', [0])}
        self.expected_cells = int(p('expected_cells', 60))
        self.minimum_coverage = float(p('minimum_coverage', 0.95))
        self.observation_timeout = float(p('observation_timeout_sec', 2.0))
        self.pose_timeout = float(p('pose_timeout_sec', 2.0))
        self.precheck_timeout = float(p('precheck_timeout_sec', 15.0))
        self.finalize_settle = float(p('finalize_settle_sec', 1.0))
        self.finalize_timeout = float(p('finalize_timeout_sec', 10.0))
        self.home_radius = float(p('home_radius_m', 1.0))
        self.landed_altitude = float(p('landed_relative_alt_m', 0.5))
        self.disarm_stable = float(p('disarm_stable_sec', 1.0))
        self.output_root = os.path.abspath(os.path.expanduser(str(
            p('output_root', '~/.ros/cage_uav/scout_runs'))))
        self.record_bag = bool(p('record_bag', False))
        self.bag_topics = list(p('bag_topics', []))
        self.auto_start = bool(rospy.get_param('~auto_start', False))
        self.auto_start_delay = float(rospy.get_param('~auto_start_delay_sec', 3.0))
        self.auto_activate_completed_map = bool(p('auto_activate_completed_map', True))

        self.status_pub = rospy.Publisher('/scout/task_status', ScoutTaskStatus,
                                          queue_size=1, latch=True)
        self.mission_request_pub = rospy.Publisher('/mission/request', MissionRequest,
                                                   queue_size=1)
        self.waypoint_pub = rospy.Publisher('/mission/waypoints', MissionWaypointArray,
                                            queue_size=1, latch=True)
        self.mission_cancel_pub = rospy.Publisher('/mission/cancel', String, queue_size=1)
        self.route_pub = rospy.Publisher('/scout/route_path', Path, queue_size=1, latch=True)

        rospy.Subscriber('/scout/task_request', ScoutTaskRequest, self.on_request, queue_size=5)
        rospy.Subscriber('/scout/task_cancel', String, self.on_cancel, queue_size=5)
        rospy.Subscriber('/mission/state', MissionState, self.on_mission, queue_size=10)
        rospy.Subscriber('/feeding/task_status', FeedingTaskStatus,
                         lambda value: setattr(self, 'feeding', value), queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.on_fcu, queue_size=1)
        rospy.Subscriber('/mavros/extended_state', ExtendedState,
                         self.on_extended, queue_size=1)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix,
                         self.on_gps, queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                         self.on_pose, queue_size=1)
        rospy.Subscriber('/scout/local_lattice_observation', LocalLatticeObservation,
                         self.on_observation, queue_size=1)
        rospy.Subscriber('/scout/working_lattice_map', PersistentLattice,
                         self.on_map, queue_size=1)
        rospy.Subscriber('/scout/map_events', String, self.on_map_event, queue_size=100)
        rospy.Subscriber('/control/owner', String,
                         lambda value: setattr(self, 'control_owner', value.data.upper()),
                         queue_size=1)
        self.reset_map = rospy.ServiceProxy('/scout/reset_map', ResetScoutMap)
        self.finalize_map = rospy.ServiceProxy('/scout/finalize_map', FinalizeScoutMap)
        self.activate_map = rospy.ServiceProxy('/scout/activate_map', ActivateScoutMap)
        rospy.Timer(rospy.Duration(0.2), self.tick)
        rospy.on_shutdown(self.shutdown)
        self.publish_status()

    def on_fcu(self, message):
        self.fcu = message
        now = time.monotonic()
        if message.armed:
            self.disarmed_since = None
            self.disarmed = False
        elif self.disarmed_since is None:
            self.disarmed_since = now

    def on_extended(self, message):
        self.extended = message

    def on_gps(self, message):
        if (message.status.status >= NavSatStatus.STATUS_FIX and
                all(math.isfinite(value) for value in
                    (message.latitude, message.longitude, message.altitude))):
            self.gps = message
            self.last_gps = time.monotonic()

    def on_pose(self, message):
        self.pose = message
        self.last_pose = time.monotonic()

    def on_observation(self, _message):
        self.last_observation = time.monotonic()

    def on_map(self, message):
        self.latest_map = message

    def on_map_event(self, message):
        with self.lock:
            if self.event_stream is None:
                return
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                payload = {'raw': message.data}
            self._write_event('map_event', payload)

    def _request_payload(self, request):
        return {
            'schema_version': 2,
            'task_id': request.task_id,
            'cage_id': int(request.cage_id),
            'survey_relative_alt': float(request.survey_relative_alt),
            'auto_rtl': bool(request.auto_rtl),
            'auto_land': bool(request.auto_land),
            'requested_at': rospy.Time.now().to_sec(),
        }

    def on_request(self, request):
        with self.lock:
            if self.state not in TERMINAL_TASK_STATES | {'IDLE'}:
                rospy.logwarn('Scout request %s rejected: task is %s', request.task_id, self.state)
                return
            validation = self._validate_request(request)
            if validation:
                self._publish_rejection(request, validation)
                return
            directory = os.path.join(self.output_root, request.task_id)
            try:
                os.makedirs(directory, mode=0o750, exist_ok=False)
            except FileExistsError:
                self._publish_rejection(request, 'DUPLICATE_TASK_ID')
                return
            except OSError as exc:
                self._publish_rejection(request, 'OUTPUT_DIRECTORY_ERROR:{}'.format(exc))
                return
            self._reset_runtime()
            self.request = request
            self.mission_id = '{}-flight'.format(request.task_id)
            self.output_directory = directory
            self.started_wall = time.time()
            self.started_ros = rospy.Time.now()
            self.precheck_deadline = time.monotonic() + self.precheck_timeout
            self.event_stream = open(os.path.join(directory, 'events.jsonl'), 'a',
                                     encoding='utf-8', buffering=1)
            _atomic_json(os.path.join(directory, 'request.json'),
                         self._request_payload(request))
            self._transition('PRECHECK')
            self._start_bag()

    def _validate_request(self, request):
        if not TASK_ID_PATTERN.match(str(request.task_id or '')):
            return 'INVALID_TASK_ID'
        if int(request.cage_id) not in self.targetable_cages:
            return 'CAGE_NOT_TARGETABLE'
        if not bool(request.auto_rtl) or not bool(request.auto_land):
            return 'AUTO_RTL_AND_LAND_REQUIRED'
        if (not math.isfinite(float(request.survey_relative_alt)) or
                float(request.survey_relative_alt) <= 0.0):
            return 'INVALID_SURVEY_ALTITUDE'
        return ''

    def _publish_rejection(self, request, reason):
        rospy.logwarn('Scout request rejected: %s', reason)
        message = ScoutTaskStatus()
        _safe_set(message, 'header', Header(stamp=rospy.Time.now()))
        _safe_set(message, 'task_id', str(request.task_id))
        _safe_set(message, 'mission_id', '')
        _safe_set(message, 'cage_id', int(request.cage_id))
        _safe_set(message, 'state', 'FAILED')
        _safe_set(message, 'reason', reason)
        _safe_set(message, 'error_code', reason.split(':', 1)[0])
        _safe_set(message, 'disarmed', not self.fcu.armed)
        self.status_pub.publish(message)

    def _reset_runtime(self):
        self.reason = ''
        self.error_code = ''
        self.route = []
        self.route_completed = False
        self.map_finalized = False
        self.map_ready_for_feeding = False
        self.mapping_outcome = 'PENDING'
        self.flight_outcome = 'PENDING'
        self.snapshot_path = ''
        self.latest_map = None
        self.map_session_started = False
        self.home_gps = None
        self.home_pose = None
        self.home_distance = float('inf')
        self.returned_home = False
        self.landed = False
        self.disarmed = not self.fcu.armed
        self.rtl_commanded = False
        self.rtl_accepted = False
        self.finalize_requested_at = 0.0
        self.finalize_attempted = False
        self.plan_published = False
        self.cancel_requested = False

    def _precheck_reason(self):
        now = time.monotonic()
        if self.control_owner != 'AUTO':
            return 'CONTROL_OWNER_NOT_AUTO'
        if not self.fcu.connected:
            return 'FCU_NOT_CONNECTED'
        if self.gps is None or now - self.last_gps > self.pose_timeout:
            return 'GPS_NOT_READY'
        if self.pose is None or now - self.last_pose > self.pose_timeout:
            return 'LOCAL_POSE_NOT_READY'
        if now - self.last_observation > self.observation_timeout:
            return 'SCOUT_OBSERVATION_NOT_READY'
        if self.mission.state not in STARTABLE_MISSION_STATES:
            return 'MISSION_BUSY_{}'.format(self.mission.state)
        if self.feeding.task_state in ACTIVE_FEEDING_STATES:
            return 'FEEDING_TASK_BUSY'
        return ''

    def _begin_task(self):
        self.home_gps = (self.gps.latitude, self.gps.longitude, self.gps.altitude)
        self.home_pose = (self.pose.pose.position.x,
                          self.pose.pose.position.y,
                          self.pose.pose.position.z)
        try:
            rospy.wait_for_service('/scout/reset_map', timeout=2.0)
            response = self.reset_map(task_id=self.request.task_id,
                                      cage_id=int(self.request.cage_id))
            if not bool(getattr(response, 'success', False)):
                raise RuntimeError(getattr(response, 'reason', 'MAP_RESET_REJECTED'))
            self.map_session_started = True
        except (rospy.ROSException, rospy.ServiceException, RuntimeError) as exc:
            self._finish_before_flight('MAP_RESET_FAILED', str(exc))
            return
        try:
            config = ScoutRouteConfig(
                rows=int(rospy.get_param('/cage_grid/rows', 6)),
                cols=int(rospy.get_param('/cage_grid/cols', 10)),
                origin_x=float(rospy.get_param('/cage_grid/origin_x', 70.0)),
                origin_y=float(rospy.get_param('/cage_grid/origin_y', -6.0)),
                cell_size_x=float(rospy.get_param('/cage_grid/cell_size_x', 12.0)),
                cell_size_y=float(rospy.get_param('/cage_grid/cell_size_y', 12.0)),
                yaw_deg=float(rospy.get_param('/cage_grid/yaw_deg', 0.0)),
                survey_relative_alt=float(self.request.survey_relative_alt),
                enter_distance=float(rospy.get_param('/scout_mission/enter_distance_m', 8.0)),
                acceptance_radius=float(rospy.get_param(
                    '/scout_mission/waypoint_acceptance_radius_m', 1.0)),
                speed=float(rospy.get_param('/scout_mission/speed_mps', 8.0)),
                max_leg_distance=float(rospy.get_param('/safety/max_leg_distance', 5000.0)),
            )
            reference = (
                float(rospy.get_param('/cage_grid/reference_lat', self.home_gps[0])),
                float(rospy.get_param('/cage_grid/reference_lon', self.home_gps[1])),
                float(rospy.get_param('/cage_grid/reference_alt', self.home_gps[2])),
            )
            self.route = plan_scout_mission(self.home_gps, reference, config)
        except (TypeError, ValueError) as exc:
            self._finish_before_flight('ROUTE_PLANNING_FAILED', str(exc))
            return
        _atomic_json(os.path.join(self.output_directory, 'route.json'), {
            'schema_version': 1,
            'route_source': 'nominal_survey_region',
            'mission_id': self.mission_id,
            'waypoints': [vars(value) for value in self.route],
        })
        self._publish_route_path()
        request = MissionRequest()
        request.header = Header(stamp=rospy.Time.now())
        request.mission_id = self.mission_id
        request.target_cage_id = int(self.request.cage_id)
        request.start = True
        request.resume_after_override = False
        request.rtl_after_mission = True
        request.feed_relative_alt = float(self.request.survey_relative_alt)
        request.transit_relative_alt = float(self.request.survey_relative_alt)
        _safe_set(request, 'mission_type', getattr(MissionRequest, 'MISSION_SCOUT', 1))
        _safe_set(request, 'auto_land', True)
        self._transition('PLANNING')
        self.mission_request_pub.publish(request)

    def _publish_route_path(self):
        path = Path(header=Header(stamp=rospy.Time.now(), frame_id='map'))
        reference_lat = float(rospy.get_param('/cage_grid/reference_lat', self.home_gps[0]))
        reference_lon = float(rospy.get_param('/cage_grid/reference_lon', self.home_gps[1]))
        reference_alt = float(rospy.get_param('/cage_grid/reference_alt', self.home_gps[2]))
        from cage_geo.geodesy import LocalENU
        local = LocalENU(reference_lat, reference_lon, reference_alt)
        for waypoint in self.route:
            if not waypoint.waypoint_type.startswith('SCOUT_'):
                continue
            east, north, _ = local.to_enu(waypoint.lat, waypoint.lon, reference_alt)
            pose = PoseStamped(header=path.header)
            pose.pose.position.x = east
            pose.pose.position.y = north
            pose.pose.position.z = waypoint.alt
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.route_pub.publish(path)

    def _publish_waypoints(self):
        message = MissionWaypointArray(
            header=Header(stamp=rospy.Time.now(), frame_id='map'),
            mission_id=self.mission_id,
            cage_id=int(self.request.cage_id),
        )
        message.waypoints = [MissionWaypoint(
            seq=value.seq, lat=value.lat, lon=value.lon, alt=value.alt,
            yaw=value.yaw, acceptance_radius=value.acceptance_radius,
            speed=value.speed, hold_time=value.hold_time,
            waypoint_type=value.waypoint_type, cell_id=value.cell_id)
            for value in self.route]
        self.waypoint_pub.publish(message)
        self.plan_published = True

    def on_mission(self, message):
        with self.lock:
            self.mission = message
            if self.request is None or message.mission_id != self.mission_id:
                return
            state = message.state
            if state == 'PLAN' and not self.plan_published:
                self._publish_waypoints()
            mapping = {
                'PLAN': 'PLANNING',
                'UPLOAD_MISSION': 'UPLOADING',
                'WAIT_ARM': 'ARMING',
                'TAKEOFF': 'TAKEOFF',
                'TRANSIT': 'TRANSIT',
                'SCOUTING': 'SCOUTING',
                'FINALIZING_MAP': 'FINALIZING_MAP',
                'RTL': 'RTL_RETURNING',
                'LAND': 'LANDING',
                'TIMEOUT_ABORT': 'ABORTING',
                'RECOVERY_RTL': 'RTL_RETURNING',
                'RECOVERY_RTL_VERIFY': 'RTL_RETURNING',
                'RECOVERY_RETURNING': 'RTL_RETURNING',
                'RECOVERY_LANDING': 'LANDING',
                'MANUAL_OVERRIDE': 'MANUAL_OVERRIDE',
            }
            if state in mapping:
                # Recovery-state reasons are implementation details.  A user
                # cancellation remains USER_CANCELED all the way into the
                # immutable result instead of being overwritten by
                # TIMEOUT_RETURNING/RECOVERY_COMPLETE.
                transition_reason = '' if self.cancel_requested else message.reason
                self._transition(mapping[state], transition_reason)
            if state == 'FINALIZING_MAP' and not self.finalize_requested_at:
                self.route_completed = True
                self.finalize_requested_at = time.monotonic()
            if (state in ('TIMEOUT_ABORT', 'ERROR', 'FAILSAFE') and
                    self.map_session_started and not self.finalize_attempted):
                self._finalize(quality_required=False, preserve_error=True)
            if state in ('RTL', 'RECOVERY_RTL_VERIFY', 'RECOVERY_RETURNING'):
                self.rtl_commanded = True
                self.rtl_accepted = state in ('RTL', 'RECOVERY_RETURNING')
            if state == 'FINISHED':
                self._maybe_finish_after_landing()
            elif state in ('ERROR', 'FAILSAFE'):
                self.error_code = message.reason or state
                self.reason = message.reason or state
                if not self.fcu.armed:
                    self._complete_terminal('FAILED')

    def on_cancel(self, message):
        with self.lock:
            if self.request is None or message.data != self.request.task_id:
                return
            if self.state in TERMINAL_TASK_STATES:
                return
            self.cancel_requested = True
            self.error_code = 'USER_CANCELED'
            self.reason = 'User requested safe cancel and RTL'
            if self.state == 'PRECHECK':
                self._complete_terminal('CANCELED')
                return
            self._transition('ABORTING', self.reason)
            if self.map_session_started and not self.finalize_attempted:
                self._finalize(quality_required=False, preserve_error=True)
            self.mission_cancel_pub.publish(String(data=self.mission_id))

    def _finalize(self, quality_required=True, preserve_error=False):
        self.finalize_attempted = True
        previous_error = self.error_code
        previous_reason = self.reason
        try:
            rospy.wait_for_service('/scout/finalize_map', timeout=2.0)
            response = self.finalize_map(
                task_id=self.request.task_id,
                cage_id=int(self.request.cage_id),
                output_directory=self.output_directory,
                min_coverage=(self.minimum_coverage if quality_required else 0.0))
            self.map_finalized = bool(getattr(response, 'success', False))
            self.snapshot_path = str(getattr(response, 'snapshot_path', ''))
            response_coverage = float(getattr(response, 'coverage', self._coverage()))
            self.map_ready_for_feeding = (
                self.map_finalized and response_coverage >= self.minimum_coverage)
            if not quality_required and self.map_finalized:
                self.mapping_outcome = ('CANCELED_PARTIAL_MAP' if self.cancel_requested
                                        else 'FAILED_FLIGHT_PARTIAL_MAP')
            elif self.map_ready_for_feeding:
                self.mapping_outcome = 'SUCCESS'
            else:
                self.mapping_outcome = 'FAILED_MAP_QUALITY'
                self.error_code = 'FAILED_MAP_QUALITY'
                self.reason = str(getattr(response, 'reason', 'Map quality gate failed'))
            _atomic_json(os.path.join(self.output_directory, 'quality.json'),
                         self._quality_payload(response_coverage))
        except (rospy.ROSException, rospy.ServiceException, OSError, ValueError) as exc:
            self.mapping_outcome = 'FAILED_MAP_FINALIZE'
            self.error_code = 'MAP_FINALIZE_FAILED'
            self.reason = str(exc)
        if preserve_error:
            # Flight/cancel error owns terminal semantics.  Map outcome remains
            # separately visible in result.json.
            self.error_code = previous_error
            self.reason = previous_reason
        self.publish_status()

    def _coverage(self):
        confirmed = self._confirmed_cells()
        return float(confirmed) / self.expected_cells if self.expected_cells else 0.0

    def _confirmed_cells(self):
        if self.latest_map is None:
            return 0
        return sum(1 for value in self.latest_map.cells
                   if value.identity_state == value.IDENTITY_CONFIRMED)

    def _quality_payload(self, coverage=None):
        cells = list(self.latest_map.cells) if self.latest_map is not None else []
        confirmed = [value for value in cells
                     if value.identity_state == value.IDENTITY_CONFIRMED]
        confidences = [float(value.identity_confidence) for value in confirmed]
        return {
            'schema_version': 2,
            'artifact_bundle_schema_version': 2,
            'task_id': self.request.task_id,
            'cage_id': int(self.request.cage_id),
            'map_uuid': self.latest_map.map_uuid if self.latest_map else '',
            'map_version': int(self.latest_map.map_version) if self.latest_map else 0,
            'expected_cells': self.expected_cells,
            'confirmed_cells': len(confirmed),
            'coverage': self._coverage() if coverage is None else float(coverage),
            'geometry_sha256': self.latest_map.geometry_sha256 if self.latest_map else '',
            'mean_identity_confidence': (
                sum(confidences) / len(confidences) if confidences else 0.0),
            'online_only': True,
            'note': 'GT identity/RMSE metrics are evaluator-only',
        }

    def tick(self, _event):
        with self.lock:
            now = time.monotonic()
            if (self.auto_start and not self.auto_start_sent and
                    now - self.node_started >= self.auto_start_delay):
                self.auto_start_sent = True
                request = ScoutTaskRequest()
                request.header = Header(stamp=rospy.Time.now())
                request.task_id = 'scout-ci-{}'.format(int(time.time()))
                request.cage_id = min(self.targetable_cages)
                request.survey_relative_alt = float(rospy.get_param(
                    '/scout_mission/survey_relative_alt_m', 25.0))
                request.auto_rtl = True
                request.auto_land = True
                self.on_request(request)
            self._update_flight_observations(now)
            if self.state == 'PRECHECK':
                reason = self._precheck_reason()
                if not reason:
                    self._begin_task()
                elif now >= self.precheck_deadline:
                    self._finish_before_flight('PRECHECK_TIMEOUT', reason)
                else:
                    self.reason = reason
                    self.publish_status()
            if (self.state == 'FINALIZING_MAP' and not self.finalize_attempted and
                    now - self.finalize_requested_at >= self.finalize_settle):
                self._finalize()
            if (self.state == 'FINALIZING_MAP' and not self.finalize_attempted and
                    now - self.finalize_requested_at >= self.finalize_timeout):
                self.mapping_outcome = 'FAILED_MAP_FINALIZE'
                self.error_code = 'MAP_FINALIZE_TIMEOUT'
                self.reason = 'Map finalization timeout'
                self.finalize_attempted = True
                self.publish_status()
            if self.mission.mission_id == self.mission_id and self.mission.state == 'FINISHED':
                self._maybe_finish_after_landing()
            if self.request is not None and self.state not in TERMINAL_TASK_STATES:
                self.publish_status()

    def _update_flight_observations(self, now):
        if self.home_pose is not None and self.pose is not None:
            dx = self.pose.pose.position.x - self.home_pose[0]
            dy = self.pose.pose.position.y - self.home_pose[1]
            self.home_distance = math.hypot(dx, dy)
            self.returned_home = self.home_distance <= self.home_radius
        extended_landed = (
            int(getattr(self.extended, 'landed_state', 0)) ==
            int(getattr(ExtendedState, 'LANDED_STATE_ON_GROUND', 1)))
        low_altitude = (self.pose is not None and
                        self.pose.pose.position.z <= self.landed_altitude)
        self.landed = bool(extended_landed or (low_altitude and not self.fcu.armed))
        self.disarmed = bool(not self.fcu.armed and self.disarmed_since is not None and
                             now - self.disarmed_since >= self.disarm_stable)

    def _maybe_finish_after_landing(self):
        if not (self.returned_home and self.landed and self.disarmed):
            return
        if self.cancel_requested:
            self.flight_outcome = 'CANCELED_SAFE_RECOVERY'
            terminal = 'CANCELED'
        else:
            self.flight_outcome = 'SUCCESS' if self.route_completed else 'FAILED_ROUTE'
        if (not self.cancel_requested and self.flight_outcome == 'SUCCESS' and
              self.mapping_outcome == 'SUCCESS' and self.map_ready_for_feeding):
            terminal = 'COMPLETED'
        elif not self.cancel_requested:
            terminal = 'FAILED'
        self._complete_terminal(terminal)

    def _finish_before_flight(self, code, reason):
        self.error_code = code
        self.reason = reason
        self.flight_outcome = 'NOT_STARTED'
        self.mapping_outcome = 'NOT_STARTED'
        self._complete_terminal('FAILED')

    def _result_payload(self, terminal):
        return {
            'schema_version': 2,
            'artifact_bundle_schema_version': 2,
            'task_id': self.request.task_id,
            'mission_id': self.mission_id,
            'cage_id': int(self.request.cage_id),
            'terminal_status': terminal,
            'reason': self.reason,
            'error_code': self.error_code,
            'started_at': self.started_wall,
            'finished_at': time.time(),
            'flight_outcome': self.flight_outcome,
            'mapping_outcome': self.mapping_outcome,
            'flight': {
                'waypoints_total': self._survey_total(),
                'waypoints_reached': self._survey_completed(),
                'route_completed': self.route_completed,
                'rtl_commanded': self.rtl_commanded,
                'rtl_accepted': self.rtl_accepted,
                'returned_home': self.returned_home,
                'home_horizontal_error_m': self.home_distance,
                'landed': self.landed,
                'disarmed': self.disarmed,
            },
            'map': {
                'map_uuid': self.latest_map.map_uuid if self.latest_map else '',
                'map_version': int(self.latest_map.map_version) if self.latest_map else 0,
                'frozen': self.map_finalized,
                'geometry_sha256': self.latest_map.geometry_sha256 if self.latest_map else '',
                'expected_cells': self.expected_cells,
                'confirmed_cells': self._confirmed_cells(),
                'coverage': self._coverage(),
                'map_sha256': (_sha256(self.snapshot_path)
                               if self.snapshot_path and os.path.isfile(self.snapshot_path)
                               else ''),
            },
            'artifacts': {
                'persistent_map': os.path.basename(self.snapshot_path) if self.snapshot_path else '',
                'events': 'events.jsonl',
                'rosbag': 'mission.bag' if os.path.exists(
                    os.path.join(self.output_directory, 'mission.bag')) else '',
                'quality_report': 'quality.json' if os.path.exists(
                    os.path.join(self.output_directory, 'quality.json')) else '',
                'manifest': 'manifest.json',
            },
        }

    def _complete_terminal(self, terminal):
        if self.state in TERMINAL_TASK_STATES:
            return
        self._stop_bag()
        if self.event_stream is not None:
            self._write_event('task_terminal', {'state': terminal})
            self.event_stream.flush()
            os.fsync(self.event_stream.fileno())
            self.event_stream.close()
            self.event_stream = None
        result_path = os.path.join(self.output_directory, 'result.json')
        _atomic_json(result_path, self._result_payload(terminal))
        files = {}
        for name in ('request.json', 'route.json', 'map.json', 'events.jsonl',
                     'mission.bag', 'quality.json', 'result.json'):
            path = os.path.join(self.output_directory, name)
            if os.path.isfile(path):
                files[name] = _sha256(path)
        manifest = {
            'schema_version': 2,
            'artifact_bundle_schema_version': 2,
            'map_schema_version': 2,
            'result_schema_version': 2,
            'quality_schema_version': 2,
            'task_id': self.request.task_id,
            'cage_id': int(self.request.cage_id),
            'map_uuid': self.latest_map.map_uuid if self.latest_map else '',
            'map_version': int(self.latest_map.map_version) if self.latest_map else 0,
            'observation_source': rospy.get_param(
                '/persistent_lattice/observation_source', ''),
            'preset': rospy.get_param('/persistent_lattice/deformation_preset', ''),
            'seed': int(rospy.get_param('/persistent_lattice/deformation_seed', 0)),
            'source_sha256': self._source_hash(),
            'software_source_sha256': (
                self.latest_map.software_source_sha256 if self.latest_map else self._source_hash()),
            'mapping_config_sha256': (
                self.latest_map.mapping_config_sha256 if self.latest_map else ''),
            'reference_sha256': self.latest_map.reference_sha256 if self.latest_map else '',
            'nominal_geometry_sha256': (
                self.latest_map.geometry_sha256 if self.latest_map else ''),
            'environment_geometry_sha256': (
                self.latest_map.environment_geometry_sha256 if self.latest_map else ''),
            'environment_id': str(rospy.get_param(
                '/scout_map_registry/environment_id', '')),
            'runtime_config_sha256': self._runtime_config_hash(),
            'files': files,
            'created_at': self.started_wall,
            'finalized_at': time.time(),
            'created_at_unix_sec': self.started_wall,
        }
        manifest['finalized_at_unix_sec'] = manifest['finalized_at']
        manifest_path = os.path.join(self.output_directory, 'manifest.json')
        _atomic_json(manifest_path, manifest)
        self._transition(terminal, self.reason)
        if terminal == 'COMPLETED' and self.auto_activate_completed_map:
            cage_id = int(self.request.cage_id)
            map_uuid = self.latest_map.map_uuid if self.latest_map else ''
            map_version = int(self.latest_map.map_version) if self.latest_map else 0
            threading.Thread(
                target=self._activate_completed_artifact,
                args=(self.request.task_id, cage_id, map_uuid, map_version,
                      files.get('map.json', ''), _sha256(manifest_path)),
                daemon=True).start()

    def _activate_completed_artifact(self, task_id, cage_id, map_uuid, map_version,
                                     map_sha256, manifest_sha256):
        # Terminal Scout status must be visible before registry busy checks.
        time.sleep(0.25)
        try:
            rospy.wait_for_service('/scout/activate_map', timeout=10.0)
            response = self.activate_map(
                task_id=task_id, cage_id=cage_id,
                expected_map_uuid=map_uuid,
                expected_map_version=map_version,
                expected_map_sha256=map_sha256,
                expected_manifest_sha256=manifest_sha256)
            if not response.success:
                rospy.logerr('Completed Scout map activation failed: %s %s',
                             response.reason_code, response.message)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr('Completed Scout map activation service failed: %s', exc)

    def _source_hash(self):
        configured = str(rospy.get_param('/scout_mission/source_sha256', '')).strip()
        if configured:
            return configured
        digest = hashlib.sha256()
        for path in sorted((__file__,)):
            with open(path, 'rb') as stream:
                digest.update(stream.read())
        return digest.hexdigest()

    @staticmethod
    def _runtime_config_hash():
        payload = {
            'persistent_lattice': rospy.get_param('/persistent_lattice', {}),
            'scout_mission': rospy.get_param('/scout_mission', {}),
            'cage_grid': rospy.get_param('/cage_grid', {}),
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'),
                             default=str, allow_nan=False).encode('utf-8')
        return hashlib.sha256(encoded).hexdigest()

    def _survey_total(self):
        return sum(value.waypoint_type.startswith('SCOUT_') for value in self.route)

    def _survey_completed(self):
        current = int(getattr(self.mission, 'current_waypoint', -1))
        return sum(value.waypoint_type.startswith('SCOUT_') and value.seq <= current
                   for value in self.route)

    def _transition(self, state, reason=''):
        changed = state != self.state
        self.state = state
        if reason:
            self.reason = reason
        if changed:
            rospy.loginfo('Scout task %s -> %s (%s)',
                          self.request.task_id if self.request else '', state, self.reason)
            self._write_event('task_state', {'state': state, 'reason': self.reason})
        self.publish_status()

    def publish_status(self):
        message = ScoutTaskStatus()
        _safe_set(message, 'header', Header(stamp=rospy.Time.now()))
        _safe_set(message, 'task_id', self.request.task_id if self.request else '')
        _safe_set(message, 'mission_id', self.mission_id)
        _safe_set(message, 'cage_id', int(self.request.cage_id) if self.request else 0)
        _safe_set(message, 'state', self.state)
        _safe_set(message, 'reason', self.reason)
        _safe_set(message, 'error_code', self.error_code)
        _safe_set(message, 'current_waypoint', self._survey_completed())
        _safe_set(message, 'total_waypoints', self._survey_total())
        _safe_set(message, 'route_completed', self.route_completed)
        _safe_set(message, 'map_uuid', self.latest_map.map_uuid if self.latest_map else '')
        _safe_set(message, 'map_version', int(self.latest_map.map_version) if self.latest_map else 0)
        _safe_set(message, 'geometry_sha256',
                  self.latest_map.geometry_sha256 if self.latest_map else '')
        _safe_set(message, 'confirmed_cells', self._confirmed_cells())
        _safe_set(message, 'expected_cells', self.expected_cells)
        _safe_set(message, 'coverage', self._coverage())
        _safe_set(message, 'map_finalized', self.map_finalized)
        _safe_set(message, 'map_ready_for_feeding', self.map_ready_for_feeding)
        _safe_set(message, 'rtl_commanded', self.rtl_commanded)
        _safe_set(message, 'rtl_accepted', self.rtl_accepted)
        _safe_set(message, 'returned_home', self.returned_home)
        _safe_set(message, 'home_horizontal_error_m',
                  self.home_distance if math.isfinite(self.home_distance) else -1.0)
        _safe_set(message, 'landed', self.landed)
        _safe_set(message, 'disarmed', self.disarmed)
        _safe_set(message, 'output_directory', self.output_directory)
        self.status_pub.publish(message)

    def _write_event(self, record_type, payload):
        if self.event_stream is None:
            return
        value = {
            'schema_version': 1,
            'record_type': record_type,
            'stamp': rospy.Time.now().to_sec(),
            'task_id': self.request.task_id if self.request else '',
            'mission_id': self.mission_id,
            'payload': payload,
        }
        self.event_stream.write(json.dumps(value, sort_keys=True) + '\n')

    def _start_bag(self):
        if not self.record_bag or not self.bag_topics:
            return
        path = os.path.join(self.output_directory, 'mission.bag')
        try:
            self.bag_process = subprocess.Popen(
                ['rosbag', 'record', '-O', path] + self.bag_topics,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            self.reason = 'ROSBAG_START_FAILED:{}'.format(exc)

    def _stop_bag(self):
        if self.bag_process is None:
            return
        if self.bag_process.poll() is None:
            self.bag_process.send_signal(signal.SIGINT)
            try:
                self.bag_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self.bag_process.terminate()
                self.bag_process.wait(timeout=5.0)
        self.bag_process = None

    def shutdown(self):
        with self.lock:
            self._stop_bag()
            if self.event_stream is not None and not self.event_stream.closed:
                self.event_stream.close()


if __name__ == '__main__':
    rospy.init_node('scout_task_manager')
    ScoutTaskManager()
    rospy.spin()
