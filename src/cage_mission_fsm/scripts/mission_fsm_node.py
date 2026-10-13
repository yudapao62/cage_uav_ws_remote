#!/usr/bin/env python3
import math
import json
import time
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State, WaypointReached
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Header, String
from cage_msgs.msg import (FeederCommand, FeederIdentityState, MissionRequest,
                           MissionState, MissionWaypointArray, ScoutTaskStatus)
from cage_mavros_adapter.adapter import MavrosAdapter
from cage_mission_fsm.core import MissionFSM, States
from cage_mission_fsm.identity_gate import identity_gate_decision
from cage_geo.geodesy import LocalENU


class MissionFSMNode:
    def __init__(self):
        self.fsm = MissionFSM(); self.fsm.initialize()
        self.adapter = MavrosAdapter()
        self.auto_arm = rospy.get_param('/vehicle/auto_arm', False)
        self.auto_mode = rospy.get_param('/vehicle/auto_mode', 'AUTO')
        self.takeoff_mode = rospy.get_param('/vehicle/takeoff_mode', 'GUIDED')
        self.use_guided_takeoff = bool(rospy.get_param('/vehicle/use_guided_takeoff', False))
        self.prearm_mode = rospy.get_param('/vehicle/prearm_mode', 'STABILIZE')
        self.rtl_mode = rospy.get_param('/vehicle/rtl_mode', 'RTL')
        self.land_mode = rospy.get_param('/vehicle/land_mode', 'LAND')
        self.geometry_mode = str(rospy.get_param('/cage/geometry_mode', 'rectangular')).strip().lower()
        if self.geometry_mode not in ('rectangular', 'circular'):
            raise ValueError('invalid /cage/geometry_mode: {}'.format(self.geometry_mode))
        self.circular_dwell_sec = float(rospy.get_param('/cage/circular/feeding_dwell_sec', 5.0))
        self.circular_altitude_tolerance = float(rospy.get_param('/cage/circular/feeding_altitude_tolerance_m', 3.0))
        if self.geometry_mode == 'circular' and self.circular_dwell_sec <= 0:
            raise ValueError('circular feeding_dwell_sec must be positive')
        self.fsm.MANUAL_MODES = set(rospy.get_param('/vehicle/manual_modes', list(self.fsm.MANUAL_MODES)))
        self.gps_timeout = rospy.get_param('/safety/gps_timeout', 2.0)
        self.fcu_timeout = rospy.get_param('/safety/fcu_timeout', 2.0)
        self.low_battery_threshold = rospy.get_param('/safety/low_battery_threshold', 0.20)
        self.require_beacon = rospy.get_param('/safety/require_beacon_before_start', False)
        self.beacon_loss_timeout = float(rospy.get_param('/safety/beacon_loss_during_active_mission/timeout_sec', 5.0))
        self.beacon_loss_action = rospy.get_param('/safety/beacon_loss_during_active_mission/action', 'RTL')
        self.last_beacon_valid = 0.0
        self.last_beacon_report = 0.0
        self.beacon_current_valid = False
        self.timeouts = {States.WAIT_ARM: rospy.get_param('/mission/arm_timeout',60.0),
                         States.TAKEOFF: rospy.get_param('/mission/takeoff_timeout',60.0),
                         States.TRANSIT: rospy.get_param('/mission/transit_timeout',600.0),
                         States.FEEDING: rospy.get_param('/mission/feeding_timeout',180.0),
                         States.FEEDING_DWELL: rospy.get_param('/mission/feeding_timeout',180.0),
                         States.EXIT: rospy.get_param('/mission/feeding_timeout',180.0),
                         States.SCOUTING: rospy.get_param('/mission/scout_timeout',600.0),
                         States.FINALIZING_MAP: rospy.get_param('/scout_mission/finalize_timeout_sec',10.0),
                         States.RTL: rospy.get_param('/mission/rtl_timeout',600.0),
                         States.LAND: rospy.get_param('/mission/rtl_timeout',600.0)}
        self.home_radius = float(rospy.get_param('/scout_mission/home_radius_m', 1.0))
        self.landed_altitude = float(rospy.get_param(
            '/scout_mission/landed_relative_alt_m', 0.5))
        self.disarm_stable_sec = float(rospy.get_param(
            '/scout_mission/disarm_stable_sec', 1.0))
        self.feeder_off_timeout = rospy.get_param('/mission/timeout_recovery/feeder_off_timeout_sec', 2.0)
        rtl_cfg = rospy.get_param('/mission/timeout_recovery/rtl', {})
        self.rtl_retries = int(rtl_cfg.get('retries', 3))
        self.rtl_request_interval = float(rtl_cfg.get('request_interval_sec', 1.0))
        self.rtl_confirm_timeout = float(rtl_cfg.get('mode_confirm_timeout_sec', 5.0))
        self.recovery_entry_time = 0.0
        self.last_recovery_rtl_request = 0.0
        self.recovery_rtl_attempts = 0
        self.recovery_stage = ''
        now = time.monotonic()
        self.last_fcu, self.last_gps, self.state_since = now, 0.0, now
        self.last_state, self.waypoints = self.fsm.state, []
        self.upload_attempted = self.arm_attempted = self.auto_attempted = self.rtl_attempted = False
        self.land_attempted = False
        self.auto_position_probe_ok = False
        self.armed_since = None
        self.takeoff_commanded = False
        self.last_arm_attempt = self.last_auto_attempt = 0.0
        self.last_prearm_mode_attempt = 0.0
        self.stream_rate_requested = False
        self.feeder_status, self.feeder_command_time = False, 0.0
        self.require_feeder_identity_gate = bool(rospy.get_param(
            '/persistent_lattice/require_feeder_identity_gate', False))
        self.feeder_identity_timeout = float(rospy.get_param(
            '/persistent_lattice/feeder/identity_timeout_sec', 2.0))
        self.feeder_identity_state = FeederIdentityState.STATUS_STALE_MAP
        self.feeder_identity_cell = ''
        self.feeder_identity_reason = 'NO_IDENTITY_RECEIVED'
        self.feeder_active_map_pin = ()
        self.feeder_task_map_pin = ()
        self.expected_task_map_pin = ()
        self.last_feeder_identity = 0.0
        self.current_gps = None
        self.current_pose = None
        self.home_pose = None
        self.extended_state = ExtendedState()
        self.disarmed_since = None
        self.dwell_started_at = 0.0
        self.control_owner = str(rospy.get_param('/control_authority/initial_owner', 'AUTO')).upper()
        self.state_pub = rospy.Publisher('/mission/state', MissionState, queue_size=1, latch=True)
        self.feeder_pub = rospy.Publisher('/feeder/command', FeederCommand, queue_size=1, latch=True)
        self.takeover_pub = rospy.Publisher('/manual/takeover_request', Bool, queue_size=10)
        rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=1)
        rospy.Subscriber('/mavros/extended_state', ExtendedState,
                         self.on_extended_state, queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                         self.on_local_pose, queue_size=1)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.on_gps, queue_size=1)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.on_reached, queue_size=10)
        rospy.Subscriber('/mavros/battery', BatteryState, self.on_battery, queue_size=1)
        rospy.Subscriber('/mission/request', MissionRequest, self.on_request, queue_size=1)
        rospy.Subscriber('/mission/cancel', String, self.on_cancel, queue_size=5)
        rospy.Subscriber('/mission/waypoints', MissionWaypointArray, self.on_waypoints, queue_size=1)
        rospy.Subscriber('/scout/task_status', ScoutTaskStatus,
                         self.on_scout_status, queue_size=5)
        rospy.Subscriber('/feeder/status', Bool, self.on_feeder_status, queue_size=1)
        if self.require_feeder_identity_gate:
            rospy.Subscriber('/feeder/identity_state', FeederIdentityState,
                             self.on_feeder_identity, queue_size=1)
        rospy.Subscriber('/control/owner', String, self.on_control_owner, queue_size=1)
        if self.require_beacon:
            rospy.Subscriber('/cage/beacon/status', String, self.on_beacon_status, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.tick)
        self.publish_state()

    def on_state(self, msg):
        now = time.monotonic()
        if msg.connected: self.last_fcu = now
        if msg.armed:
            self.disarmed_since = None
        elif self.disarmed_since is None:
            self.disarmed_since = now
        self._check_manual_takeover(msg.mode)

    def on_extended_state(self, msg):
        self.extended_state = msg

    def on_local_pose(self, msg):
        self.current_pose = msg

    def _check_manual_takeover(self, mode):
        if self.control_owner != 'AUTO':
            return self.fsm.state == States.MANUAL_OVERRIDE
        if self.fsm.mode_changed(mode, self.auto_mode):
            rospy.logwarn('Manual takeover detected (%s); NX relinquishes control', mode)
            # Stop this node synchronously, then notify the authority manager.
            self.control_owner = 'MANUAL'
            self.takeover_pub.publish(Bool(data=True))
            self.command_feeder(False, 'MANUAL_OVERRIDE')
            self._state_changed()
            return True
        return False

    def on_control_owner(self, msg):
        owner = str(msg.data).upper()
        if owner not in ('AUTO', 'MANUAL'):
            rospy.logwarn_throttle(2.0, 'Ignoring invalid control owner %s', msg.data)
            return
        previous = self.control_owner
        self.control_owner = owner
        if owner == 'MANUAL':
            if self.fsm.manual_takeover('CONTROL_OWNER_MANUAL'):
                self.command_feeder(False, 'MANUAL_OVERRIDE')
                self._state_changed()
        elif previous == 'MANUAL' and self.fsm.auto_reacquired():
            # Explicit recovery never resumes the old mission in phase one.
            self._state_changed()

    def on_gps(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX and all(math.isfinite(v) for v in (msg.latitude,msg.longitude,msg.altitude)) and (msg.latitude != 0 or msg.longitude != 0):
            self.current_gps = msg
            self.last_gps = time.monotonic()

    def on_battery(self, msg):
        if math.isfinite(msg.percentage) and 0 <= msg.percentage < self.low_battery_threshold:
            if self.fsm.state == States.IDLE:
                self.fsm.failsafe('LOW_BATTERY'); self._state_changed()
            elif self.fsm.state in self.fsm.ACTIVE:
                self.fsm.timeout_abort('LOW_BATTERY')
                self._enter_timeout_abort(time.monotonic())
                self.command_feeder(False, 'LOW_BATTERY'); self._state_changed()

    def on_request(self, msg):
        if not msg.start: return
        if self.control_owner != 'AUTO':
            rospy.logwarn('Mission request rejected: control owner is %s', self.control_owner); return
        if self.require_beacon and not rospy.get_param('/realflight/start_ready', False):
            rospy.logerr('Realflight mission rejected: precheck not ready'); return
        rtl_after = True if not hasattr(msg, 'rtl_after_mission') else msg.rtl_after_mission
        mission_type = int(getattr(msg, 'mission_type', MissionRequest.MISSION_FEEDER))
        auto_land = bool(getattr(msg, 'auto_land', True))
        if mission_type == MissionRequest.MISSION_SCOUT:
            rtl_after = True
            auto_land = True
        if self.fsm.request(msg.mission_id, msg.resume_after_override, rtl_after,
                            mission_type, auto_land):
            if mission_type == MissionRequest.MISSION_FEEDER:
                self.expected_task_map_pin = (
                    str(msg.task_map_uuid), int(msg.task_map_version),
                    str(msg.task_map_sha256), str(msg.task_manifest_sha256))
            else:
                self.expected_task_map_pin = ()
            self.upload_attempted = self.arm_attempted = self.auto_attempted = self.rtl_attempted = False
            self.land_attempted = False
            self.auto_position_probe_ok = False
            self.armed_since = None
            self.takeoff_commanded = False
            self.last_arm_attempt = self.last_auto_attempt = self.last_prearm_mode_attempt = 0.0
            self.dwell_started_at = 0.0
            self.home_pose = (None if self.current_pose is None else
                              (self.current_pose.pose.position.x,
                               self.current_pose.pose.position.y,
                               self.current_pose.pose.position.z))
            self._state_changed()
        else: rospy.logwarn('Mission request rejected in state %s', self.fsm.state)

    def on_waypoints(self, msg):
        if self.control_owner != 'AUTO': return
        if self.fsm.state != States.PLAN or msg.mission_id != self.fsm.mission_id: return
        if not self.valid_waypoints(msg):
            self.fsm.transition(States.ERROR, 'INVALID_MISSION'); self._state_changed(); return
        self.waypoints = list(msg.waypoints); self.fsm.planned(); self._state_changed()

    def valid_waypoints(self, msg):
        kinds = [w.waypoint_type for w in msg.waypoints]
        legacy = ['TAKEOFF','TRANSIT','CAGE_ENTER','FEED_START','FEED_END','CAGE_EXIT']
        grid_ok = len(kinds) >= 4 and kinds[:3] == ['TAKEOFF','TRANSIT','GRID_ENTER'] and kinds[-1] == 'GRID_EXIT' and all(k == 'CELL_TARGET' for k in kinds[3:-1])
        circular_ok = kinds == ['TAKEOFF','TRANSIT','FEED_POINT'] and msg.waypoints[-1].hold_time > 0
        scout_ok = (len(kinds) >= 4 and kinds[:3] == ['TAKEOFF','TRANSIT','SCOUT_ENTER']
                    and kinds[-1] == 'SCOUT_EXIT'
                    and all(k == 'SCOUT_PASS' for k in kinds[3:-1]))
        if self.fsm.mission_type == MissionRequest.MISSION_SCOUT and not scout_ok:
            return False
        if self.fsm.mission_type != MissionRequest.MISSION_SCOUT and scout_ok:
            return False
        if kinds != legacy and not grid_ok and not circular_ok and not scout_ok: return False
        return all(w.seq == i and -90 <= w.lat <= 90 and -180 <= w.lon <= 180 and w.alt > 0
                   and all(math.isfinite(v) for v in (w.lat,w.lon,w.alt,w.yaw,w.acceptance_radius,w.speed,w.hold_time))
                   for i,w in enumerate(msg.waypoints))

    def on_reached(self, msg):
        if self.control_owner != 'AUTO': return
        mission_index = msg.wp_seq - self.adapter.mission_seq_offset
        if mission_index < 0 or mission_index >= len(self.waypoints): return
        old_feeder = self.fsm.feeder_enabled
        reached_wp = self.waypoints[mission_index]
        self.fsm.waypoint_reached(mission_index, reached_wp.waypoint_type, reached_wp.cell_id)
        if reached_wp.waypoint_type == 'FEED_POINT' and self.fsm.state == States.FEEDING_DWELL:
            self._begin_circular_dwell(mission_index, 'MISSION_ITEM_REACHED')
        if self.fsm.feeder_enabled != old_feeder and reached_wp.waypoint_type != 'FEED_POINT':
            self.command_feeder(self.fsm.feeder_enabled, self.waypoints[mission_index].waypoint_type)
        if self.fsm.state == States.RTL and not self.rtl_attempted:
            self._request_rtl()
        self._state_changed()

    def on_cancel(self, msg):
        if msg.data != self.fsm.mission_id:
            return
        if self.fsm.cancel('USER_CANCELED'):
            if self.fsm.state == States.TIMEOUT_ABORT:
                self._enter_timeout_abort(time.monotonic())
            self._state_changed()

    def on_scout_status(self, msg):
        if (self.fsm.mission_type != MissionRequest.MISSION_SCOUT or
                msg.mission_id != self.fsm.mission_id or
                self.fsm.state != States.FINALIZING_MAP):
            return
        if msg.map_finalized and msg.map_ready_for_feeding:
            self.fsm.scout_finalized(True, 'SCOUT_MAP_FINALIZED')
        elif msg.error_code:
            self.fsm.scout_finalized(False, msg.error_code)
        else:
            return
        self._request_rtl()
        self._state_changed()

    def _request_rtl(self):
        if self.rtl_attempted:
            return
        self.rtl_attempted = True
        if not self.adapter.rtl(self.rtl_mode):
            self.fsm.timeout_abort('RTL_MODE_REJECTED')
            self._enter_timeout_abort(time.monotonic())

    def on_feeder_status(self, msg): self.feeder_status = msg.data

    def on_feeder_identity(self, msg):
        self.feeder_identity_state = int(msg.status)
        self.feeder_identity_cell = msg.current_cell_id
        self.feeder_identity_reason = msg.decision_reason
        self.feeder_active_map_pin = (
            str(msg.map_uuid), int(msg.map_version),
            str(msg.map_sha256), str(msg.manifest_sha256))
        self.feeder_task_map_pin = (
            str(msg.task_map_uuid), int(msg.task_map_version),
            str(msg.task_map_sha256), str(msg.task_manifest_sha256))
        self.last_feeder_identity = time.monotonic()
        allowed, reason = self._identity_gate_allows()
        if not allowed and (self.feeder_status or self.fsm.feeder_enabled):
            rospy.logerr('Feeder identity gate closed while feeding: %s', reason)
            self.feeder_pub.publish(FeederCommand(
                header=Header(stamp=rospy.Time.now()),
                mission_id=self.fsm.mission_id,
                enable=False,
                reason='IDENTITY_GATE_'+reason))
            self.feeder_command_time = time.monotonic()
            if self.fsm.feeder_enabled:
                if reason == 'CELL_MISMATCH' and self._safe_cell_departure():
                    self.fsm.feeder_departed_cell()
                    rospy.loginfo(
                        'Feeder OFF between cell targets; identity advanced to %s',
                        self.feeder_identity_cell)
                else:
                    self.fsm.failsafe('FEEDER_IDENTITY_'+reason)
                self._state_changed()

    def _safe_cell_departure(self):
        """Recognize only the next planned cell or final grid exit as transit."""
        if (self.fsm.state != States.FEEDING or
                self.fsm.current_waypoint < 0 or
                not self.feeder_identity_cell):
            return False
        for waypoint in self.waypoints[self.fsm.current_waypoint + 1:]:
            if waypoint.waypoint_type == 'CELL_TARGET':
                return waypoint.cell_id == self.feeder_identity_cell
            if waypoint.waypoint_type == 'GRID_EXIT':
                return True
        return False

    def on_beacon_status(self, msg):
        try:
            self.beacon_current_valid = json.loads(msg.data).get('geometry_valid') is True
            self.last_beacon_report = time.monotonic()
            if self.beacon_current_valid:
                self.last_beacon_valid = time.monotonic()
        except (ValueError, TypeError):
            pass

    def command_feeder(self, enable, reason):
        self.feeder_command_time = time.monotonic()
        if enable:
            allowed, gate_reason = self._identity_gate_allows()
            if not allowed:
                rospy.logerr('Feeder ON denied by identity gate: %s', gate_reason)
                enable = False
                reason = 'IDENTITY_GATE_'+gate_reason
                if self.require_feeder_identity_gate:
                    self.fsm.failsafe('FEEDER_IDENTITY_'+gate_reason)
        self.feeder_pub.publish(FeederCommand(header=Header(stamp=rospy.Time.now()),
                                mission_id=self.fsm.mission_id, enable=enable, reason=reason))

    def _identity_gate_allows(self):
        age = (None if self.last_feeder_identity <= 0.0
               else time.monotonic() - self.last_feeder_identity)
        return identity_gate_decision(
            self.require_feeder_identity_gate,
            self.feeder_identity_state,
            self.feeder_identity_cell,
            self.fsm.current_cell_id,
            age,
            self.feeder_identity_timeout,
            self.feeder_active_map_pin,
            self.feeder_task_map_pin,
            self.expected_task_map_pin)

    def tick(self, _event):
        now = time.monotonic()
        if self.adapter.state.armed:
            if self.armed_since is None:
                self.armed_since = now
        else:
            self.armed_since = None
        # Redundant polling closes any transport/callback race: manual mode is
        # always evaluated before mission progression or dwell completion.
        self._check_manual_takeover(self.adapter.get_mode())
        if self.control_owner != 'AUTO':
            if self.fsm.state != States.MANUAL_OVERRIDE:
                self.fsm.manual_takeover('CONTROL_OWNER_MANUAL')
                self.command_feeder(False, 'MANUAL_OVERRIDE')
                self._state_changed()
            self.publish_state()
            return
        if self.fsm.state == States.WAIT_FCU and self.adapter.is_connected():
            if rospy.get_param('/vehicle/request_stream_rate', True) and not self.stream_rate_requested:
                self.stream_rate_requested = self.adapter.request_stream_rate(rospy.get_param('/vehicle/stream_rate', 10))
            self.fsm.fcu_connected(); self._state_changed()
        if self.fsm.state == States.WAIT_GPS and now-self.last_gps <= self.gps_timeout:
            self.fsm.gps_valid(); self._state_changed()
        self._maybe_begin_circular_dwell(now)
        if self.fsm.state in self.fsm.ACTIVE:
            if now-self.last_fcu > self.fcu_timeout:
                self.fsm.timeout_abort('FCU_DISCONNECTED')
                self._enter_timeout_abort(now)
                self.command_feeder(False,'FCU_DISCONNECTED'); self._state_changed()
            elif now-self.last_gps > self.gps_timeout:
                self.fsm.timeout_abort('GPS_LOST')
                self._enter_timeout_abort(now)
                self.command_feeder(False,'GPS_LOST'); self._state_changed()
        if (self.require_beacon and self.fsm.state in (States.PLAN,States.UPLOAD_MISSION,States.WAIT_ARM) and
                (not self.beacon_current_valid or now-self.last_beacon_report > float(rospy.get_param('/beacon/receive_timeout_sec',2.0)))):
            self.fsm.failsafe('BEACON_LOST_BEFORE_TAKEOFF')
            self.command_feeder(False,'BEACON_LOST')
            self._state_changed()
        if (self.require_beacon and self.beacon_loss_action == 'RTL' and
                self.fsm.state in (States.TAKEOFF,States.TRANSIT,States.FEEDING,States.FEEDING_DWELL,States.EXIT) and
                now-self.last_beacon_valid > self.beacon_loss_timeout):
            self.fsm.timeout_abort('BEACON_LOST')
            self._enter_timeout_abort(now)
            self._state_changed()
        if self.fsm.state == States.UPLOAD_MISSION and not self.upload_attempted:
            self.upload_attempted = True
            # TODO_K9_VERIFY: mission clear/push/pull compatibility is verified in SITL only.
            self.fsm.upload_result(self.adapter.upload_mission(self.waypoints)); self._state_changed()
        if self.fsm.state == States.WAIT_ARM:
            mode = self.adapter.get_mode()
            probe_accepted_now = False
            # A disarmed GUIDED request is an FCU-side EKF readiness probe:
            # ArduCopter rejects it until horizontal position is usable. Probe
            # before arming so the motor auto-disarm window is not consumed
            # while waiting for GPS/EKF readiness.
            if (self.auto_arm and self.use_guided_takeoff and not self.adapter.state.armed and
                    not self.auto_position_probe_ok and now-self.last_auto_attempt >= 2.0):
                self.last_auto_attempt = now
                if self.adapter.set_mode(self.takeoff_mode):
                    self.auto_position_probe_ok = True
                    probe_accepted_now = True
                    rospy.loginfo('FCU accepted disarmed %s probe; position control is ready', self.takeoff_mode)
                else:
                    rospy.logwarn_throttle(5.0, 'Waiting for FCU to accept %s position-control probe', self.takeoff_mode)
                mode = self.adapter.get_mode()
            # After a previous mission ends in LAND/RTL, ArduPilot may refuse arming
            # until we explicitly return to the pre-arm mode again.
            arm_ready = self.auto_position_probe_ok if self.use_guided_takeoff else True
            if (self.auto_arm and arm_ready and not self.adapter.state.armed and
                    mode != self.prearm_mode and now-self.last_prearm_mode_attempt >= 2.0):
                self.last_prearm_mode_attempt = now
                if not self.adapter.set_mode(self.prearm_mode): rospy.logwarn('Pre-arm mode request rejected; remaining in WAIT_ARM')
                mode = self.adapter.get_mode()
            if (self.auto_arm and arm_ready and not probe_accepted_now and
                    not self.adapter.state.armed and
                    mode == self.prearm_mode and now-self.last_arm_attempt >= 2.0):
                self.arm_attempted = True; self.last_arm_attempt = now
                # TODO_K9_VERIFY: automatic arming stays disabled in real-vehicle configuration.
                if not self.adapter.arm(): rospy.logwarn('ARM rejected; remaining in WAIT_ARM until timeout')
            # Ground launch is deliberately GUIDED: ArduCopter 4.5.7 keeps an
            # uploaded AUTO NAV_TAKEOFF at zero throttle with this legacy SITL
            # home-item convention. Once airborne we hand the remaining route
            # to AUTO, preserving true FCU mission execution.
            if (self.use_guided_takeoff and self.adapter.state.armed and self.armed_since is not None and
                    now-self.armed_since >= 2.0 and
                    now-self.last_auto_attempt >= 2.0):
                self.last_auto_attempt = now
                if mode != self.takeoff_mode:
                    if not self.adapter.set_mode(self.takeoff_mode):
                        rospy.logwarn('GUIDED takeoff mode rejected; remaining in WAIT_ARM')
                elif not self.takeoff_commanded:
                    target_alt = float(self.waypoints[0].alt)
                    if self.adapter.takeoff(target_alt):
                        self.takeoff_commanded = True
                        self.fsm.armed_and_auto(); self._state_changed()
                    else:
                        rospy.logwarn('GUIDED takeoff command rejected; remaining in WAIT_ARM')
            if (not self.use_guided_takeoff and self.adapter.state.armed and
                    self.armed_since is not None and now-self.armed_since >= 2.0 and
                    now-self.last_auto_attempt >= 2.0):
                self.auto_attempted = True; self.last_auto_attempt = now
                if not self.adapter.start_mission(self.auto_mode):
                    rospy.logwarn('AUTO request rejected; remaining in WAIT_ARM until timeout')
            if (not self.use_guided_takeoff and self.adapter.state.armed and
                    self.adapter.get_mode() == self.auto_mode):
                self.fsm.armed_and_auto(); self._state_changed()
        if (self.use_guided_takeoff and self.fsm.state == States.TAKEOFF and self.takeoff_commanded and
                self.adapter.relative_alt is not None and self.waypoints and
                self.adapter.relative_alt >= max(1.0, float(self.waypoints[0].alt) - 1.0) and
                now-self.last_auto_attempt >= 2.0):
            self.last_auto_attempt = now
            transit_seq = self.adapter.mission_seq_offset + 1
            if self.adapter.start_mission(self.auto_mode, transit_seq):
                self.fsm.waypoint_reached(0, 'TAKEOFF')
                self._state_changed()
            else:
                rospy.logwarn('AUTO transit start rejected after GUIDED takeoff')
        if self.fsm.state == States.RTL:
            if not self.rtl_attempted:
                self._request_rtl(); self._state_changed()
            returned_home = self._returned_home()
            on_ground = self._on_ground()
            if returned_home and not self.adapter.state.armed and on_ground:
                self.fsm.landed(); self._state_changed()
            elif (returned_home and self.fsm.auto_land and self.adapter.state.armed and
                  not self.land_attempted):
                self.land_attempted = True
                if self.adapter.land(self.land_mode):
                    self.fsm.landed(); self._state_changed()
                else:
                    self.fsm.timeout_abort('LAND_MODE_REJECTED')
                    self._enter_timeout_abort(now); self._state_changed()
        if (self.fsm.state == States.LAND and self._returned_home() and
                self._on_ground() and not self.adapter.state.armed and
                self.disarmed_since is not None and
                now-self.disarmed_since >= self.disarm_stable_sec):
            self.fsm.landed(); self._state_changed()
        limit = self.timeouts.get(self.fsm.state)
        if limit and now-self.state_since > limit:
            if self.fsm.state in (States.TAKEOFF, States.TRANSIT, States.SCOUTING,
                                  States.FINALIZING_MAP, States.FEEDING, States.EXIT,
                                  States.RTL, States.LAND):
                self.fsm.timeout_abort(self.fsm.state+'_TIMEOUT'); self._enter_timeout_abort(now); self._state_changed()
            else:
                self.fsm.failsafe(self.fsm.state+'_TIMEOUT'); self.command_feeder(False,'MISSION_TIMEOUT'); self._state_changed()
        if self.fsm.feeder_enabled and not self.feeder_status and now-self.feeder_command_time > 2.0:
            self.fsm.timeout_abort('FEEDER_FAILURE')
            self._enter_timeout_abort(now)
            self.command_feeder(False,'FEEDER_FAILURE'); self._state_changed()
        if self.fsm.state == States.FEEDING_DWELL and self.dwell_started_at and now-self.dwell_started_at >= self.circular_dwell_sec:
            if self.fsm.feeding_dwell_complete():
                rospy.loginfo('[CIRCULAR] feeder OFF; feeding dwell complete -> %s', self.fsm.state)
                self.command_feeder(False, 'CIRCULAR_DWELL_COMPLETE')
                if self.fsm.state == States.RTL and not self.rtl_attempted:
                    self._request_rtl()
                self._state_changed()
        self._handle_timeout_recovery(now)
        self.publish_state()

    def _maybe_begin_circular_dwell(self, now):
        if self.geometry_mode != 'circular' or self.fsm.state not in (States.TAKEOFF, States.TRANSIT):
            return
        feed_index = next((i for i,w in enumerate(self.waypoints) if w.waypoint_type == 'FEED_POINT'), None)
        if feed_index is None or self.current_gps is None or self.fsm.current_waypoint < feed_index - 1:
            return
        waypoint = self.waypoints[feed_index]
        local = LocalENU(self.current_gps.latitude, self.current_gps.longitude, self.current_gps.altitude)
        east, north, _ = local.to_enu(waypoint.lat, waypoint.lon, self.current_gps.altitude)
        altitude_ready = (self.adapter.relative_alt is not None and
                          abs(self.adapter.relative_alt-waypoint.alt) <= self.circular_altitude_tolerance)
        if math.hypot(east,north) <= waypoint.acceptance_radius and altitude_ready:
            if self.fsm.feed_point_arrived(feed_index):
                self._begin_circular_dwell(feed_index, 'ACCEPTANCE_RADIUS')
                self._state_changed()

    def _begin_circular_dwell(self, mission_index, source):
        if self.dwell_started_at:
            return
        self.dwell_started_at = time.monotonic()
        self.circular_dwell_sec = float(self.waypoints[mission_index].hold_time or self.circular_dwell_sec)
        rospy.loginfo('[CIRCULAR] FEED_POINT reached via %s; feeder ON; dwell %.1f sec',
                      source, self.circular_dwell_sec)
        self.command_feeder(True, 'FEED_POINT')

    def _home_distance(self):
        if self.home_pose is None or self.current_pose is None:
            return float('inf')
        return math.hypot(self.current_pose.pose.position.x-self.home_pose[0],
                          self.current_pose.pose.position.y-self.home_pose[1])

    def _returned_home(self):
        return self._home_distance() <= self.home_radius

    def _on_ground(self):
        extended = (self.extended_state.landed_state ==
                    ExtendedState.LANDED_STATE_ON_GROUND)
        low_altitude = (self.adapter.relative_alt is not None and
                        self.adapter.relative_alt <= self.landed_altitude)
        return bool(extended or low_altitude)

    def _enter_timeout_abort(self, now):
        self.recovery_entry_time = now
        self.last_recovery_rtl_request = 0.0
        self.recovery_rtl_attempts = 0
        self.recovery_stage = 'FEEDER_OFF'
        self.command_feeder(False, 'TIMEOUT_ABORT')

    def _handle_timeout_recovery(self, now):
        if self.fsm.state == States.TIMEOUT_ABORT:
            if self.recovery_stage != 'FEEDER_OFF':
                self.recovery_stage = 'FEEDER_OFF'
                self.recovery_entry_time = now
                self.command_feeder(False, 'TIMEOUT_ABORT')
            if self.feeder_status is False or now - self.recovery_entry_time >= self.feeder_off_timeout:
                self.fsm.recovery_start()
                self.recovery_stage = 'RTL_REQUEST'
                self._state_changed()
        elif self.fsm.state == States.RECOVERY_RTL:
            if now - self.last_recovery_rtl_request >= self.rtl_request_interval:
                if self.recovery_rtl_attempts >= self.rtl_retries:
                    self.fsm.failsafe('RTL_REQUEST_FAILED')
                    self._state_changed()
                    return
                self.last_recovery_rtl_request = now
                self.recovery_rtl_attempts += 1
                if self.adapter.request_rtl(self.rtl_mode):
                    self.fsm.transition(States.RECOVERY_RTL_VERIFY, 'RTL_REQUESTED')
                    self.recovery_stage = 'RTL_VERIFY'
                    self._state_changed()
                elif self.recovery_rtl_attempts >= self.rtl_retries:
                    self.fsm.failsafe('RTL_REQUEST_FAILED')
                    self._state_changed()
        elif self.fsm.state == States.RECOVERY_RTL_VERIFY:
            if self.adapter.is_rtl_mode(self.rtl_mode):
                self.fsm.recovery_rtl_verified()
                self.recovery_stage = 'RETURNING'
                self._state_changed()
            elif now - self.state_since > self.rtl_confirm_timeout:
                self.fsm.transition(States.RECOVERY_RTL, 'RTL_CONFIRM_TIMEOUT')
                self._state_changed()
        elif self.fsm.state == States.RECOVERY_RETURNING:
            if self._returned_home() and (self._on_ground() or not self.adapter.state.armed):
                self.fsm.recovery_landing()
                self.recovery_stage = 'LANDING'
                self._state_changed()
            elif (self._returned_home() and self.fsm.auto_land and
                  self.adapter.state.armed and not self.land_attempted):
                self.land_attempted = True
                if self.adapter.land(self.land_mode):
                    self.fsm.recovery_landing()
                    self.recovery_stage = 'LANDING'
                    self._state_changed()
                else:
                    self.fsm.failsafe('LAND_MODE_REJECTED')
                    self._state_changed()
        elif self.fsm.state == States.RECOVERY_LANDING:
            if (self._returned_home() and self.adapter.is_landed() and
                    self.disarmed_since is not None and
                    now-self.disarmed_since >= self.disarm_stable_sec):
                self.fsm.recovery_landed()
                self.recovery_stage = 'LANDED'
                self._state_changed()

    def _state_changed(self):
        if self.fsm.state != self.last_state:
            rospy.loginfo('FSM %s -> %s (%s)', self.last_state, self.fsm.state, self.fsm.reason)
            self.last_state, self.state_since = self.fsm.state, time.monotonic()
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(MissionState(header=Header(stamp=rospy.Time.now()), mission_id=self.fsm.mission_id,
            state=self.fsm.state, reason=self.fsm.reason, current_waypoint=self.fsm.current_waypoint,
            feeder_enabled=(self.fsm.feeder_enabled and self._identity_gate_allows()[0]), manual_override=self.fsm.state == States.MANUAL_OVERRIDE,
            current_cell_id=self.fsm.current_cell_id, mission_type=self.fsm.mission_type,
            auto_land=self.fsm.auto_land))


if __name__ == '__main__':
    rospy.init_node('mission_fsm_node'); MissionFSMNode(); rospy.spin()
