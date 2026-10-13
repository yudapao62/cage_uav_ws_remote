"""Deterministic safety policy for the mission FSM, independent of ROS transport."""
class States:
    INIT='INIT'; WAIT_FCU='WAIT_FCU'; WAIT_GPS='WAIT_GPS'; IDLE='IDLE'; PLAN='PLAN'
    UPLOAD_MISSION='UPLOAD_MISSION'; WAIT_ARM='WAIT_ARM'; TAKEOFF='TAKEOFF'
    TRANSIT='TRANSIT'; SCOUTING='SCOUTING'; FINALIZING_MAP='FINALIZING_MAP'
    FEEDING='FEEDING'; FEEDING_DWELL='FEEDING_DWELL'; EXIT='EXIT'; RTL='RTL'; LAND='LAND'
    FINISHED='FINISHED'; MANUAL_OVERRIDE='MANUAL_OVERRIDE'; FAILSAFE='FAILSAFE'; ERROR='ERROR'
    TIMEOUT_ABORT='TIMEOUT_ABORT'; RECOVERY_RTL='RECOVERY_RTL'; RECOVERY_RTL_VERIFY='RECOVERY_RTL_VERIFY'
    RECOVERY_RETURNING='RECOVERY_RETURNING'; RECOVERY_LANDING='RECOVERY_LANDING'; RECOVERY_LANDED='RECOVERY_LANDED'


class MissionFSM:
    ACTIVE = {States.UPLOAD_MISSION, States.WAIT_ARM, States.TAKEOFF, States.TRANSIT,
              States.SCOUTING, States.FINALIZING_MAP,
              States.FEEDING, States.FEEDING_DWELL, States.EXIT, States.RTL, States.LAND}
    CONTROLLED = ACTIVE | {States.TIMEOUT_ABORT, States.RECOVERY_RTL, States.RECOVERY_RTL_VERIFY,
                            States.RECOVERY_RETURNING, States.RECOVERY_LANDING}
    # Mode-only detection is a fallback after automatic flight has begun.
    # Explicit authority requests use manual_takeover() and preempt every state,
    # including UPLOAD_MISSION and WAIT_ARM, without mistaking normal pre-arm
    # STABILIZE for pilot takeover.
    TAKEOVER_STATES = CONTROLLED - {States.UPLOAD_MISSION, States.WAIT_ARM}
    MANUAL_MODES = {'STABILIZE','LOITER','POSHOLD','ALT_HOLD','MANUAL'}

    def __init__(self):
        self.state, self.reason = States.INIT, ''
        self.mission_id, self.current_waypoint = '', -1
        self.current_cell_id, self.rtl_after_mission = '', True
        self.mission_type, self.auto_land = 0, True
        self.feeder_enabled, self.auto_control_started = False, False
        self.mission_aborted, self.recovery_active = False, False

    def transition(self, state, reason=''):
        self.state, self.reason = state, reason
        if state in (States.MANUAL_OVERRIDE, States.FAILSAFE, States.ERROR):
            self.feeder_enabled = False
        if state == States.FINISHED:
            self.recovery_active = False

    def initialize(self): self.transition(States.WAIT_FCU)
    def fcu_connected(self):
        if self.state == States.WAIT_FCU: self.transition(States.WAIT_GPS)
    def gps_valid(self):
        if self.state == States.WAIT_GPS: self.transition(States.IDLE)
    def request(self, mission_id, resume=False, rtl_after_mission=True,
                mission_type=0, auto_land=True):
        # Phase one never resumes a previous mission directly from MANUAL.
        # Authority must first be explicitly returned to AUTO, which places
        # the FSM in IDLE/HOLD; a subsequent request starts a new mission.
        if self.state == States.MANUAL_OVERRIDE: return False
        if self.state not in (States.IDLE, States.FINISHED): return False
        self.mission_id, self.current_waypoint = mission_id, -1
        self.current_cell_id, self.rtl_after_mission = '', bool(rtl_after_mission)
        self.mission_type, self.auto_land = int(mission_type), bool(auto_land)
        self.auto_control_started = False
        self.mission_aborted, self.recovery_active = False, False
        self.transition(States.PLAN, 'explicit resume' if resume else '')
        return True
    def planned(self):
        if self.state == States.PLAN: self.transition(States.UPLOAD_MISSION)
    def upload_result(self, success):
        if self.state == States.UPLOAD_MISSION:
            self.transition(States.WAIT_ARM if success else States.ERROR,
                            '' if success else 'MISSION_UPLOAD_FAILED')
    def armed_and_auto(self):
        if self.state == States.WAIT_ARM:
            self.auto_control_started = True; self.transition(States.TAKEOFF)
    def mode_changed(self, mode, expected_mode=None):
        # Once navigation has started, state itself is the authoritative proof
        # that NX owns automation. Do not depend solely on a transient arming
        # callback flag when deciding whether an H12 manual mode must preempt.
        if self.state in self.TAKEOVER_STATES and mode in self.MANUAL_MODES and mode != expected_mode:
            self.transition(States.MANUAL_OVERRIDE, 'MANUAL_MODE_'+mode); return True
        return False
    def manual_takeover(self, reason='MANUAL_REQUEST'):
        if self.state == States.MANUAL_OVERRIDE: return False
        self.transition(States.MANUAL_OVERRIDE, reason)
        return True
    def auto_reacquired(self):
        if self.state != States.MANUAL_OVERRIDE: return False
        self.auto_control_started = False
        self.mission_aborted = True
        self.recovery_active = False
        self.current_waypoint = -1
        self.current_cell_id = ''
        self.transition(States.IDLE, 'AUTO_REACQUIRED_HOLD')
        return True
    def waypoint_reached(self, seq, waypoint_type, cell_id=''):
        if self.state not in self.ACTIVE: return
        self.current_waypoint = seq
        if cell_id: self.current_cell_id = cell_id
        if waypoint_type in ('TAKEOFF','TRANSIT','CAGE_ENTER','GRID_ENTER'): self.transition(States.TRANSIT)
        elif waypoint_type == 'FEED_START': self.feeder_enabled=True; self.transition(States.FEEDING)
        elif waypoint_type == 'CELL_TARGET': self.feeder_enabled=True; self.transition(States.FEEDING)
        elif waypoint_type == 'FEED_POINT': self.feed_point_arrived(seq)
        elif waypoint_type == 'FEED_END': self.feeder_enabled=False; self.transition(States.EXIT)
        elif waypoint_type == 'CAGE_EXIT': self.feeder_enabled=False; self.transition(States.RTL)
        elif waypoint_type == 'GRID_EXIT':
            self.feeder_enabled=False; self.current_cell_id=''
            self.transition(States.RTL if self.rtl_after_mission else States.FINISHED)
        elif waypoint_type in ('SCOUT_ENTER', 'SCOUT_PASS'):
            self.feeder_enabled=False; self.current_cell_id=''
            self.transition(States.SCOUTING)
        elif waypoint_type == 'SCOUT_EXIT':
            self.feeder_enabled=False; self.current_cell_id=''
            self.transition(States.FINALIZING_MAP, 'SCOUT_ROUTE_COMPLETE')
    def scout_finalized(self, success, reason=''):
        if self.state != States.FINALIZING_MAP: return False
        if not success:
            self.mission_aborted = True
        self.transition(States.RTL,
                        reason or ('SCOUT_MAP_FINALIZED' if success else 'SCOUT_MAP_FINALIZE_FAILED'))
        return True
    def cancel(self, reason='USER_CANCELED'):
        self.feeder_enabled = False
        self.mission_aborted = True
        if self.state in (States.PLAN, States.UPLOAD_MISSION, States.WAIT_ARM):
            self.transition(States.FINISHED, reason+'_BEFORE_TAKEOFF')
            return True
        if self.state in self.ACTIVE:
            self.recovery_active = True
            self.transition(States.TIMEOUT_ABORT, reason)
            return True
        return False
    def feed_point_arrived(self, seq):
        if self.state not in self.ACTIVE or self.state == States.FEEDING_DWELL: return False
        self.current_waypoint = seq
        self.feeder_enabled = True
        self.transition(States.FEEDING_DWELL, 'CIRCULAR_FEED_POINT_REACHED')
        return True
    def feeding_dwell_complete(self):
        if self.state != States.FEEDING_DWELL: return False
        self.feeder_enabled = False
        self.transition(States.RTL if self.rtl_after_mission else States.FINISHED,
                        'CIRCULAR_FEEDING_COMPLETE')
        return True
    def feeder_departed_cell(self):
        """Turn the feeder off between rectangular cell targets.

        The FCU reports a CELL_TARGET only on arrival.  Identity can already
        move to the next cell while the vehicle is travelling there, so the
        previous cell's feeder command must not remain asserted in transit.
        """
        if self.state != States.FEEDING or not self.feeder_enabled:
            return False
        self.feeder_enabled = False
        self.reason = 'BETWEEN_CELL_TARGETS'
        return True
    def landed(self):
        if self.state == States.RTL: self.transition(States.LAND)
        elif self.state == States.LAND: self.transition(States.FINISHED)
    def timeout_abort(self, reason):
        if self.state not in self.ACTIVE: return
        self.feeder_enabled = False
        self.mission_aborted = True
        self.recovery_active = True
        self.transition(States.TIMEOUT_ABORT, reason)
    def recovery_start(self):
        if self.state == States.TIMEOUT_ABORT: self.transition(States.RECOVERY_RTL, 'TIMEOUT_RTL_REQUESTED')
    def recovery_rtl_verified(self):
        if self.state == States.RECOVERY_RTL_VERIFY: self.transition(States.RECOVERY_RETURNING, 'TIMEOUT_RTL_CONFIRMED')
    def recovery_landing(self):
        if self.state == States.RECOVERY_RETURNING: self.transition(States.RECOVERY_LANDING, 'TIMEOUT_RETURNING')
    def recovery_landed(self):
        if self.state == States.RECOVERY_LANDING:
            self.recovery_active = False
            self.transition(States.FINISHED, 'RECOVERY_COMPLETE')
    def failsafe(self, reason):
        self.mission_aborted = True
        self.recovery_active = False
        self.transition(States.FAILSAFE, reason)
