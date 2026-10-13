#!/usr/bin/env python3
"""ARM-safe MAVROS transport stub with continuous kinematic motion.

The mock deliberately uses the public MAVROS mission API. Uploaded waypoints
are followed at a finite speed and the resulting GPS/local-pose streams drive
the same observation path used by a real vehicle.
"""
import math
import threading

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State, Waypoint, WaypointList, WaypointReached
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from mavros_msgs.srv import StreamRate, StreamRateResponse, WaypointClear, WaypointClearResponse
from mavros_msgs.srv import WaypointPull, WaypointPullResponse, WaypointPush, WaypointPushResponse
from mavros_msgs.srv import WaypointSetCurrent, WaypointSetCurrentResponse
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from cage_geo.geodesy import LocalENU


def _approach(current, target, limit):
    delta = target - current
    if abs(delta) <= limit:
        return target
    return current + math.copysign(limit, delta)


class MockMavros:
    def __init__(self):
        self.state = State(connected=True, armed=False, mode='STABILIZE')
        self.home_lat = float(rospy.get_param('~latitude', -35.363261))
        self.home_lon = float(rospy.get_param('~longitude', 149.165230))
        self.home_alt = float(rospy.get_param('~altitude', 584.0))
        self.local_frame = LocalENU(self.home_lat, self.home_lon, self.home_alt)
        self.east = float(rospy.get_param('~initial_east_m', 0.0))
        self.north = float(rospy.get_param('~initial_north_m', 0.0))
        self.relative_alt = float(rospy.get_param('~initial_up_m', 0.0))
        self.lat, self.lon, self.alt = self.local_frame.to_wgs84(
            self.east, self.north, self.relative_alt)
        self.cruise_speed = max(0.1, float(rospy.get_param('~cruise_speed_mps', 8.0)))
        self.climb_rate = max(0.1, float(rospy.get_param('~climb_rate_mps', 4.0)))
        self.altitude_tolerance = max(
            0.05, float(rospy.get_param('~altitude_tolerance_m', 0.25)))
        self.home_radius = max(0.05, float(rospy.get_param(
            '~home_radius_m', rospy.get_param('/scout_mission/home_radius_m', 1.0))))
        self.land_altitude = max(0.02, float(rospy.get_param(
            '~land_altitude_m', rospy.get_param(
                '/scout_mission/landed_relative_alt_m', 0.5))))
        self.gazebo_model_name = str(rospy.get_param('~gazebo_model_name', 'iris_demo'))
        self.publish_gazebo_state = bool(rospy.get_param('~publish_gazebo_model_state', True))
        self.waypoints = []
        self.current = 0
        self.last_reached = -1
        self.arrival_at = None
        self.last_tick = None
        self.yaw = 0.0
        self.velocity = (0.0, 0.0, 0.0)
        self.lock = threading.RLock()

        self.state_pub = rospy.Publisher('/mavros/state', State, queue_size=1, latch=True)
        self.extended_pub = rospy.Publisher('/mavros/extended_state', ExtendedState,
                                            queue_size=1, latch=True)
        self.gps_pub = rospy.Publisher('/mavros/global_position/global', NavSatFix,
                                       queue_size=1, latch=True)
        self.rel_pub = rospy.Publisher('/mavros/global_position/rel_alt', Float64,
                                       queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped,
                                        queue_size=1, latch=True)
        self.velocity_pub = rospy.Publisher('/mavros/local_position/velocity_local',
                                            TwistStamped, queue_size=1)
        self.wp_pub = rospy.Publisher('/mavros/mission/waypoints', WaypointList,
                                      queue_size=1, latch=True)
        self.reached_pub = rospy.Publisher('/mavros/mission/reached', WaypointReached,
                                           queue_size=1)
        self.battery_pub = rospy.Publisher('/mavros/battery', BatteryState,
                                           queue_size=1, latch=True)
        self.gazebo_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState,
                                                 queue_size=1)
        rospy.Service('/mavros/cmd/arming', CommandBool, self.arm)
        rospy.Service('/mavros/set_mode', SetMode, self.mode)
        rospy.Service('/mavros/mission/clear', WaypointClear, self.clear)
        rospy.Service('/mavros/mission/push', WaypointPush, self.push)
        rospy.Service('/mavros/mission/pull', WaypointPull, self.pull)
        rospy.Service('/mavros/mission/set_current', WaypointSetCurrent, self.set_current)
        rospy.Service('/mavros/set_stream_rate', StreamRate,
                      lambda _req: StreamRateResponse())
        publish_rate = max(2.0, float(rospy.get_param('~publish_rate_hz', 20.0)))
        rospy.Timer(rospy.Duration(1.0 / publish_rate), self.tick)

    def arm(self, req):
        with self.lock:
            self.state.armed = bool(req.value)
        return CommandBoolResponse(success=True, result=0)

    def mode(self, req):
        with self.lock:
            self.state.mode = req.custom_mode
            if self.state.mode != 'AUTO':
                self.arrival_at = None
        return SetModeResponse(mode_sent=True)

    def clear(self, _req):
        with self.lock:
            self.waypoints = []
            self.current = 0
            self.last_reached = -1
            self.arrival_at = None
        self.wp_pub.publish(WaypointList())
        return WaypointClearResponse(success=True)

    def push(self, req):
        with self.lock:
            self.waypoints = list(req.waypoints)
            self.current = int(req.start_index) if self.waypoints else 0
            self.last_reached = -1
            self.arrival_at = None
            self._update_current_flags()
        self._publish_waypoints()
        return WaypointPushResponse(success=True, wp_transfered=len(req.waypoints))

    def pull(self, _req):
        self._publish_waypoints()
        return WaypointPullResponse(success=True, wp_received=len(self.waypoints))

    def set_current(self, req):
        with self.lock:
            if req.wp_seq < 0 or req.wp_seq >= len(self.waypoints):
                return WaypointSetCurrentResponse(success=False)
            self.current = int(req.wp_seq)
            self.arrival_at = None
            self._update_current_flags()
        self._publish_waypoints()
        return WaypointSetCurrentResponse(success=True)

    def _update_current_flags(self):
        for index, waypoint in enumerate(self.waypoints):
            waypoint.is_current = index == self.current

    def _publish_waypoints(self):
        self.wp_pub.publish(WaypointList(
            waypoints=list(self.waypoints), current_seq=self.current))

    def _waypoint_target(self, waypoint):
        if waypoint.frame in (Waypoint.FRAME_GLOBAL_REL_ALT,
                              Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT):
            target_alt = self.home_alt + float(waypoint.z_alt)
        else:
            target_alt = float(waypoint.z_alt)
        east, north, up = self.local_frame.to_enu(
            float(waypoint.x_lat), float(waypoint.y_long), target_alt)
        return east, north, up

    def _move_toward(self, target, dt):
        old = (self.east, self.north, self.relative_alt)
        dx, dy = target[0] - self.east, target[1] - self.north
        horizontal = math.hypot(dx, dy)
        if horizontal > 1e-9:
            step = min(horizontal, self.cruise_speed * dt)
            self.east += dx / horizontal * step
            self.north += dy / horizontal * step
            self.yaw = math.atan2(dy, dx)
        self.relative_alt = _approach(
            self.relative_alt, target[2], self.climb_rate * dt)
        if dt > 0.0:
            self.velocity = ((self.east - old[0]) / dt,
                             (self.north - old[1]) / dt,
                             (self.relative_alt - old[2]) / dt)

    def _advance_mission(self, stamp, dt):
        self.velocity = (0.0, 0.0, 0.0)
        if not (self.state.armed and self.state.mode == 'AUTO' and
                self.waypoints and self.current < len(self.waypoints)):
            return
        waypoint = self.waypoints[self.current]
        target = self._waypoint_target(waypoint)
        self._move_toward(target, dt)
        horizontal_error = math.hypot(target[0] - self.east, target[1] - self.north)
        vertical_error = abs(target[2] - self.relative_alt)
        acceptance = max(0.25, float(waypoint.param2 or 1.0))
        arrived = horizontal_error <= acceptance and vertical_error <= self.altitude_tolerance
        if not arrived:
            self.arrival_at = None
            return
        if self.arrival_at is None:
            self.arrival_at = stamp.to_sec()
        if stamp.to_sec() - self.arrival_at < max(0.0, float(waypoint.param1)):
            return
        self.last_reached = self.current
        self.reached_pub.publish(WaypointReached(wp_seq=self.current))
        self.current += 1
        self.arrival_at = None
        self._update_current_flags()
        self._publish_waypoints()

    def _advance_rtl(self, dt):
        if self.state.mode != 'RTL':
            return
        # Return horizontally first, then descend.  This makes the mock expose
        # the same home/landing phases that the real task FSM must verify.
        horizontal = math.hypot(self.east, self.north)
        target_altitude = self.relative_alt if horizontal > self.home_radius else 0.0
        self._move_toward((0.0, 0.0, target_altitude), dt)
        if (math.hypot(self.east, self.north) <= self.home_radius and
                self.relative_alt <= self.land_altitude):
            self.east = self.north = self.relative_alt = 0.0
            self.state.armed = False

    def _advance_land(self, dt):
        if self.state.mode != 'LAND':
            return
        self._move_toward((self.east, self.north, 0.0), dt)
        if self.relative_alt <= self.land_altitude:
            self.relative_alt = 0.0
            self.state.armed = False

    def _publish_position(self, stamp):
        self.lat, self.lon, self.alt = self.local_frame.to_wgs84(
            self.east, self.north, self.relative_alt)
        gps = NavSatFix()
        gps.header.stamp = stamp
        gps.header.frame_id = 'map'
        gps.status.status = NavSatStatus.STATUS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS
        gps.latitude, gps.longitude, gps.altitude = self.lat, self.lon, self.alt

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.east
        pose.pose.position.y = self.north
        pose.pose.position.z = self.relative_alt
        pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        pose.pose.orientation.w = math.cos(0.5 * self.yaw)

        velocity = TwistStamped()
        velocity.header = pose.header
        velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z = self.velocity

        self.state_pub.publish(self.state)
        landed = (not self.state.armed and self.relative_alt <= self.land_altitude)
        self.extended_pub.publish(ExtendedState(
            landed_state=(ExtendedState.LANDED_STATE_ON_GROUND if landed
                          else ExtendedState.LANDED_STATE_IN_AIR)))
        self.gps_pub.publish(gps)
        self.rel_pub.publish(Float64(self.relative_alt))
        self.pose_pub.publish(pose)
        self.velocity_pub.publish(velocity)
        self.battery_pub.publish(BatteryState(percentage=1.0))
        if self.publish_gazebo_state:
            model = ModelState(model_name=self.gazebo_model_name,
                               pose=pose.pose, twist=velocity.twist,
                               reference_frame='world')
            self.gazebo_state_pub.publish(model)

    def tick(self, _event):
        stamp = rospy.Time.now()
        with self.lock:
            if self.last_tick is None:
                dt = 0.0
            else:
                dt = max(0.0, min(0.5, (stamp - self.last_tick).to_sec()))
            self.last_tick = stamp
            self._advance_mission(stamp, dt)
            self._advance_rtl(dt)
            self._advance_land(dt)
            self._publish_position(stamp)


if __name__ == '__main__':
    rospy.init_node('mock_mavros')
    MockMavros()
    rospy.spin()
