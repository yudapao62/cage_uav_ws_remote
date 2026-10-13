#!/usr/bin/env python3
"""Publish bounded planned/actual paths plus HOME and UAV heading markers."""
import math
from collections import deque

import rospy
from cage_geo.geodesy import LocalENU
from cage_msgs.msg import MissionWaypointArray
from cage_visualization.markers import color, marker, point, text_marker
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class TrajectoryVisualizer:
    NS = 'flight_debug'

    def __init__(self):
        self.frame_id = rospy.get_param('/visualization/frame_id', 'map')
        self.realflight = rospy.get_param('/cage_grid/geometry_source','fixed') == 'beacon'
        self.home_alt = rospy.get_param('/visualization/home_alt', 0.0)
        self.local_enu = None if self.realflight else LocalENU(
            rospy.get_param('/visualization/home_lat'),
            rospy.get_param('/visualization/home_lon'), self.home_alt)
        self.max_points = rospy.get_param('/visualization/actual_path_max_points', 5000)
        self.min_spacing = rospy.get_param('/visualization/actual_path_min_spacing', 0.10)
        self.heading_length = rospy.get_param('/visualization/heading_arrow_length', 4.0)
        self.text_height = rospy.get_param('/visualization/text_height', 1.2)
        self.show = rospy.get_param('/visualization/show', {})
        self.path_cfg = rospy.get_param('/visualization/path', {})
        self.uav_scale = float(rospy.get_param('/visualization/uav_rviz_scale', 2.0))
        self.actual_poses = deque(maxlen=self.max_points)

        self.planned_pub = rospy.Publisher('/visualization/planned_path', Path,
                                           queue_size=1, latch=True)
        self.actual_pub = rospy.Publisher('/visualization/actual_path', Path,
                                          queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('/visualization/flight_markers', MarkerArray,
                                          queue_size=1, latch=True)
        rospy.Subscriber('/mission/waypoints', MissionWaypointArray,
                         self.on_waypoints, queue_size=1)
        if self.realflight:
            self.relative_alt=0.0
            rospy.Subscriber('/cage/home',NavSatFix,self.on_home,queue_size=1)
            rospy.Subscriber('/mavros/global_position/global',NavSatFix,self.on_global_gps,queue_size=1)
            rospy.Subscriber('/mavros/global_position/rel_alt',Float64,lambda m:setattr(self,'relative_alt',m.data),queue_size=1)
        else:
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                             self.on_pose, queue_size=1)
        self.publish_flight_markers(None)

    def on_home(self,msg):
        self.home_alt=msg.altitude
        self.local_enu=LocalENU(msg.latitude,msg.longitude,msg.altitude)

    def on_global_gps(self,msg):
        if self.local_enu is None: return
        x,y,_=self.local_enu.to_enu(msg.latitude,msg.longitude,self.home_alt)
        pose=PoseStamped(header=msg.header)
        pose.header.frame_id=self.frame_id
        pose.pose.position=point(x,y,self.relative_alt)
        pose.pose.orientation.w=1.0
        self.on_pose(pose)

    def on_waypoints(self, msg):
        if self.local_enu is None: return
        if not self.show.get('planned_path', True) or not self.path_cfg.get('planned', True):
            return
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        for waypoint in msg.waypoints:
            east, north, _ = self.local_enu.to_enu(
                waypoint.lat, waypoint.lon, self.home_alt)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = point(east, north, waypoint.alt)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.planned_pub.publish(path)

    def on_pose(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            rospy.logwarn_throttle(5.0, 'Expected local pose in %s, received %s',
                                   self.frame_id, msg.header.frame_id)
            return
        if self.show.get('actual_path', True) and self.path_cfg.get('actual', True):
            position = msg.pose.position
            if not self.actual_poses or self.distance(self.actual_poses[-1].pose.position,
                                                       position) >= self.min_spacing:
                saved = PoseStamped()
                saved.header.frame_id = self.frame_id
                saved.header.stamp = msg.header.stamp or rospy.Time.now()
                saved.pose = msg.pose
                self.actual_poses.append(saved)
                actual = Path()
                actual.header.frame_id = self.frame_id
                actual.header.stamp = rospy.Time.now()
                actual.poses = list(self.actual_poses)
                self.actual_pub.publish(actual)
        self.publish_flight_markers(msg)

    @staticmethod
    def distance(first, second):
        return math.sqrt((first.x-second.x)**2 + (first.y-second.y)**2 +
                         (first.z-second.z)**2)

    def publish_flight_markers(self, aircraft_pose):
        stamp = rospy.Time.now()
        markers = []
        home = marker(self.frame_id, self.NS, 0, Marker.CYLINDER, stamp)
        home.pose.position = point(0.0, 0.0, 0.25)
        home.scale.x = home.scale.y = 1.8
        home.scale.z = 0.50
        home.color = color(0.10, 0.95, 0.25, 1.0)
        markers.append(home)
        markers.append(text_marker(self.frame_id, self.NS, 1, stamp,
            point(0.0, -3.0, 2.0), 'H\nHOME', self.text_height,
            color(0.20, 1.0, 0.30, 1.0)))

        if aircraft_pose is not None:
            # A deliberately enlarged display-only UAV body.  It is anchored
            # to the live MAVROS pose and does not affect Gazebo dynamics.
            body = marker(self.frame_id, self.NS, 4, Marker.CUBE, stamp)
            body.pose = aircraft_pose.pose
            body.scale.x = 1.15 * self.uav_scale
            body.scale.y = 0.72 * self.uav_scale
            body.scale.z = 0.30 * self.uav_scale
            body.color = color(0.22, 0.52, 0.95, 0.92)
            markers.append(body)

            arms = marker(self.frame_id, self.NS, 5, Marker.LINE_LIST, stamp)
            arms.scale.x = 0.14 * self.uav_scale
            arms.color = color(0.92, 0.94, 0.98, 0.95)
            yaw = yaw_from_quaternion(aircraft_pose.pose.orientation)
            c, s = math.cos(yaw), math.sin(yaw)
            center = aircraft_pose.pose.position
            arm_x, arm_y = 1.10 * self.uav_scale, 0.72 * self.uav_scale
            offsets = ((arm_x, arm_y), (-arm_x, -arm_y),
                       (arm_x, -arm_y), (-arm_x, arm_y))
            for (x1, y1), (x2, y2) in ((offsets[0], offsets[1]),
                                       (offsets[2], offsets[3])):
                arms.points.extend([
                    point(center.x + c*x1 - s*y1, center.y + s*x1 + c*y1, center.z),
                    point(center.x + c*x2 - s*y2, center.y + s*x2 + c*y2, center.z)])
            markers.append(arms)

            current = marker(self.frame_id, self.NS, 2, Marker.SPHERE, stamp)
            current.pose.position = aircraft_pose.pose.position
            current.scale.x = current.scale.y = current.scale.z = 0.55 * self.uav_scale
            current.color = color(1.0, 1.0, 1.0, 0.95)
            markers.append(current)
            if self.show.get('heading_arrow', True):
                yaw = yaw_from_quaternion(aircraft_pose.pose.orientation)
                start = aircraft_pose.pose.position
                heading = marker(self.frame_id, self.NS, 3, Marker.ARROW, stamp)
                heading.scale.x = 0.30
                heading.scale.y = 0.85
                heading.scale.z = 1.20
                heading.color = color(1.0, 0.25, 0.15, 1.0)
                heading.points = [point(start.x, start.y, start.z),
                                  point(start.x + self.heading_length * math.cos(yaw),
                                        start.y + self.heading_length * math.sin(yaw),
                                        start.z)]
                markers.append(heading)
        self.marker_pub.publish(MarkerArray(markers=markers))


if __name__ == '__main__':
    rospy.init_node('trajectory_visualizer')
    TrajectoryVisualizer()
    rospy.spin()
