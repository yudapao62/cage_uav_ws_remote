"""The only project layer allowed to call MAVROS APIs."""
import math
import threading
import rospy
from mavros_msgs.msg import ExtendedState, State, Waypoint, WaypointList
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate, WaypointClear, WaypointPull, WaypointPush, WaypointSetCurrent
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


class MavrosAdapter:
    def __init__(self, namespace='/mavros'):
        self.ns = namespace.rstrip('/')
        self.state, self.extended_state = State(), ExtendedState()
        self.global_position, self.relative_alt = None, None
        self.waypoints = []
        self._waypoint_event = threading.Event()
        self.mission_home_item_required = rospy.get_param('/vehicle/mission_home_item_required', True)
        self.mission_seq_offset = 1 if self.mission_home_item_required else 0
        rospy.Subscriber(self.ns+'/state', State, self._on_state, queue_size=1)
        rospy.Subscriber(self.ns+'/extended_state', ExtendedState,
                         self._on_extended_state, queue_size=1)
        rospy.Subscriber(self.ns+'/global_position/global', NavSatFix, self._on_gps, queue_size=1)
        rospy.Subscriber(self.ns+'/global_position/rel_alt', Float64, self._on_rel_alt, queue_size=1)
        rospy.Subscriber(self.ns+'/mission/waypoints', WaypointList, self._on_waypoints, queue_size=1)
        # TODO_K9_VERIFY: K9 support for these standard MAVROS/MAVLink services must be bench-tested.
        self._arm = rospy.ServiceProxy(self.ns+'/cmd/arming', CommandBool)
        self._takeoff = rospy.ServiceProxy(self.ns+'/cmd/takeoff', CommandTOL)
        self._mode = rospy.ServiceProxy(self.ns+'/set_mode', SetMode)
        self._clear = rospy.ServiceProxy(self.ns+'/mission/clear', WaypointClear)
        self._push = rospy.ServiceProxy(self.ns+'/mission/push', WaypointPush)
        self._pull = rospy.ServiceProxy(self.ns+'/mission/pull', WaypointPull)
        self._set_current = rospy.ServiceProxy(self.ns+'/mission/set_current', WaypointSetCurrent)
        self._stream_rate = rospy.ServiceProxy(self.ns+'/set_stream_rate', StreamRate)

    def _on_state(self, msg): self.state = msg
    def _on_extended_state(self, msg): self.extended_state = msg
    def _on_gps(self, msg): self.global_position = msg
    def _on_rel_alt(self, msg): self.relative_alt = msg.data
    def _on_waypoints(self, msg):
        self.waypoints = list(msg.waypoints); self._waypoint_event.set()

    def is_connected(self): return bool(self.state.connected)
    def get_mode(self): return self.state.mode
    def get_current_mode(self): return self.state.mode
    def get_global_position(self): return self.global_position
    def get_relative_altitude(self): return self.relative_alt
    def arm(self): return bool(self._arm(True).success)
    def disarm(self): return bool(self._arm(False).success)
    def set_mode(self, mode): return bool(self._mode(0, mode).mode_sent)
    def rtl(self, mode='RTL'): return self.set_mode(mode)
    def land(self, mode='LAND'): return self.set_mode(mode)
    def request_rtl(self, mode='RTL'): return self.set_mode(mode)
    def is_rtl_mode(self, mode='RTL'): return self.state.mode == mode
    def is_landed(self):
        extended_landed = (self.extended_state.landed_state ==
                           ExtendedState.LANDED_STATE_ON_GROUND)
        altitude_landed = self.relative_alt is not None and self.relative_alt <= 0.5
        return not self.state.armed and (extended_landed or altitude_landed)
    def request_stream_rate(self, hz=10):
        # TODO_K9_VERIFY: K9 telemetry stream-rate handling may differ from ArduPilot.
        try:
            self._stream_rate(stream_id=StreamRate._request_class.STREAM_ALL,
                              message_rate=hz, on_off=True)
            return True
        except rospy.ServiceException:
            return False
    def takeoff(self, altitude):
        return bool(self._takeoff(min_pitch=0.0, yaw=0.0, latitude=0.0,
                                  longitude=0.0, altitude=float(altitude)).success)
    def start_mission(self, auto_mode='AUTO', wp_seq=None):
        # TODO_K9_VERIFY: K9 mission/set_current support and sequence convention require bench testing.
        sequence = self.mission_seq_offset if wp_seq is None else int(wp_seq)
        if not self._set_current(wp_seq=sequence).success: return False
        return self.set_mode(auto_mode)
    def clear_mission(self): return bool(self._clear().success)

    @staticmethod
    def to_mavros_waypoint(item):
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = 22 if item.waypoint_type == 'TAKEOFF' else 16
        wp.is_current = item.seq == 0
        wp.autocontinue = True
        # NAV_WAYPOINT param1 is hold time. FEED_POINT uses it as a flight-controller
        # side position hold while the FSM independently owns feeder timing.
        wp.param1 = float(item.hold_time)
        wp.param2 = item.acceptance_radius
        wp.param3 = 0.0
        wp.param4 = item.yaw
        wp.x_lat, wp.y_long, wp.z_alt = item.lat, item.lon, item.alt
        return wp

    def upload_mission(self, items, timeout=10.0):
        """Clear, push, pull and compare the mission; never arm on failure."""
        if not items or not self.clear_mission(): return False
        outbound = [self.to_mavros_waypoint(item) for item in items]
        if self.mission_home_item_required:
            if self.global_position is None: return False
            home = Waypoint(frame=Waypoint.FRAME_GLOBAL, command=16, is_current=True, autocontinue=True,
                            x_lat=self.global_position.latitude, y_long=self.global_position.longitude,
                            z_alt=self.global_position.altitude)
            for wp in outbound: wp.is_current = False
            outbound.insert(0, home)
        response = self._push(start_index=0, waypoints=outbound)
        if not response.success or response.wp_transfered != len(outbound): return False
        self._waypoint_event.clear()
        pulled = self._pull()
        if not pulled.success or pulled.wp_received != len(outbound): return False
        self._waypoint_event.wait(timeout)
        return self._same_mission(outbound, self.waypoints, self.mission_home_item_required)

    def download_mission(self):
        self._waypoint_event.clear()
        response = self._pull()
        if response.success: self._waypoint_event.wait(5.0)
        return list(self.waypoints) if response.success else []

    @staticmethod
    def _same_mission(expected, actual, skip_home=False):
        if len(expected) != len(actual): return False
        start = 1 if skip_home else 0
        for lhs, rhs in zip(expected[start:], actual[start:]):
            if lhs.command != rhs.command or not math.isclose(lhs.x_lat, rhs.x_lat, abs_tol=2e-6) or \
               not math.isclose(lhs.y_long, rhs.y_long, abs_tol=2e-6) or \
               not math.isclose(lhs.z_alt, rhs.z_alt, abs_tol=0.2) or \
               not math.isclose(lhs.param1, rhs.param1, abs_tol=0.1): return False
        return True
