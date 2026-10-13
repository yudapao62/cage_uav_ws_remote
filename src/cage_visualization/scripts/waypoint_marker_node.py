#!/usr/bin/env python3
"""Render mission waypoints, labels, completion state and current target."""
import rospy
from cage_geo.geodesy import LocalENU
from cage_msgs.msg import MissionState, MissionWaypointArray
from sensor_msgs.msg import NavSatFix
from cage_visualization.markers import color, marker, point, text_marker
from visualization_msgs.msg import Marker, MarkerArray


FUTURE_COLORS = {
    'CAGE_ENTER': color(0.35, 0.65, 1.0, 0.95),
    'FEED_START': color(0.45, 0.90, 0.45, 0.95),
    'FEED_END': color(1.0, 0.60, 0.20, 0.95),
    'CAGE_EXIT': color(0.75, 0.50, 0.95, 0.95),
    'GRID_ENTER': color(0.35, 0.65, 1.0, 0.95),
    'CELL_TARGET': color(0.30, 0.95, 0.35, 0.95),
    'FEED_POINT': color(0.15, 1.0, 0.35, 1.0),
    'GRID_EXIT': color(0.75, 0.50, 0.95, 0.95),
}


class WaypointMarkerNode:
    NS = 'mission_waypoints'
    TERMINAL_STATES = {'FINISHED', 'ERROR', 'FAILSAFE', 'MANUAL_OVERRIDE'}

    def __init__(self):
        self.frame_id = rospy.get_param('/visualization/frame_id', 'map')
        self.realflight = rospy.get_param('/cage_grid/geometry_source','fixed') == 'beacon'
        self.local_enu = None if self.realflight else LocalENU(
            rospy.get_param('/visualization/home_lat'),
            rospy.get_param('/visualization/home_lon'),
            rospy.get_param('/visualization/home_alt', 0.0))
        self.text_height = rospy.get_param('/visualization/text_height', 1.2)
        self.show = rospy.get_param('/visualization/show', {})
        self.waypoint_cfg = rospy.get_param('/visualization/waypoint', {})
        self.waypoints = []
        self.state = MissionState(current_waypoint=-1, state='WAIT_FCU')
        self.publisher = rospy.Publisher('/mission/visualization/markers', MarkerArray,
                                         queue_size=1, latch=True)
        rospy.Subscriber('/mission/waypoints', MissionWaypointArray,
                         self.on_waypoints, queue_size=1)
        rospy.Subscriber('/mission/state', MissionState, self.on_state, queue_size=1)
        if self.realflight: rospy.Subscriber('/cage/home', NavSatFix, self.on_home, queue_size=1)

    def on_home(self, msg):
        self.local_enu = LocalENU(msg.latitude,msg.longitude,msg.altitude)
        self.publish()

    def on_waypoints(self, msg):
        self.waypoints = list(msg.waypoints)
        self.publish()

    def on_state(self, msg):
        self.state = msg
        self.publish()

    def waypoint_color(self, seq, waypoint_type, active_seq):
        if self.state.state in {'ERROR', 'FAILSAFE'} and seq == active_seq:
            return color(1.0, 0.10, 0.10, 1.0)
        if seq <= self.state.current_waypoint:
            return color(0.45, 0.48, 0.50, 0.72)
        if active_seq is not None and seq == active_seq:
            return color(0.20, 1.0, 0.25, 1.0)
        return FUTURE_COLORS.get(waypoint_type, color(1.0, 0.92, 0.60, 0.95))

    def publish(self):
        if self.local_enu is None or not self.waypoints or not self.show.get('waypoints', True) or not self.waypoint_cfg.get('enabled', True):
            return
        stamp = rospy.Time.now()
        active_seq = None
        if self.state.current_waypoint >= 0 and self.state.state not in {'WAIT_FCU', 'WAIT_GPS', 'IDLE', 'FINISHED'}:
            active_seq = min(self.state.current_waypoint + 1, len(self.waypoints) - 1)
        markers = []
        positions = []
        for waypoint in self.waypoints:
            east, north, _ = self.local_enu.to_enu(
                waypoint.lat, waypoint.lon,
                self.local_enu.home_alt if self.realflight else rospy.get_param('/visualization/home_alt', 0.0))
            positions.append((east, north, waypoint))
        feed_start = next((p for p in positions if p[2].waypoint_type == 'FEED_START'), None)
        feed_end = next((p for p in positions if p[2].waypoint_type == 'FEED_END'), None)
        normal = (0.0, 1.0)
        if feed_start and feed_end:
            dx, dy = feed_end[0] - feed_start[0], feed_end[1] - feed_start[1]
            length = (dx*dx + dy*dy) ** 0.5
            if length > 1e-9:
                normal = (-dy/length, dx/length)
        label_offsets = {
            'TAKEOFF': 3.0, 'TRANSIT': 3.0, 'CAGE_ENTER': -3.5, 'GRID_ENTER': -3.5,
            'FEED_START': -4.0, 'FEED_END': -4.0, 'CAGE_EXIT': 3.0,
            'CELL_TARGET': -4.0, 'GRID_EXIT': 3.0,
            'FEED_POINT': -4.0,
        }
        for east, north, waypoint in positions:
            rgba = self.waypoint_color(waypoint.seq, waypoint.waypoint_type, active_seq)
            sphere = marker(self.frame_id, self.NS, waypoint.seq * 2,
                            Marker.SPHERE, stamp)
            sphere.pose.position = point(east, north, waypoint.alt)
            base_scale = 1.25
            scale = (base_scale * float(self.waypoint_cfg.get('current_scale_factor', 1.5))
                     if waypoint.seq == active_seq else base_scale)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = scale
            sphere.color = rgba
            markers.append(sphere)
            if self.show.get('waypoint_labels', True) and self.waypoint_cfg.get('show_label', True):
                label = 'WP{}\n{}'.format(waypoint.seq, waypoint.cell_id or waypoint.waypoint_type)
                if self.waypoint_cfg.get('show_altitude', True):
                    label += '\n{:.1f} m'.format(waypoint.alt)
                if waypoint.seq == active_seq:
                    label += '\nCURRENT'
                offset = label_offsets.get(waypoint.waypoint_type, 2.5)
                markers.append(text_marker(self.frame_id, self.NS,
                    waypoint.seq * 2 + 1, stamp,
                    point(east + offset * normal[0], north + offset * normal[1],
                    waypoint.alt + 2.0), label,
                    self.text_height, rgba))
            if self.waypoint_cfg.get('show_vertical_guide', True):
                guide = marker(self.frame_id, self.NS, 100 + waypoint.seq,
                               Marker.LINE_LIST, stamp)
                guide.scale.x = 0.06
                guide.color = color(rgba.r, rgba.g, rgba.b, 0.34)
                guide.points = [point(east, north, 0.02), point(east, north, waypoint.alt)]
                markers.append(guide)
        self.publisher.publish(MarkerArray(markers=markers))


if __name__ == '__main__':
    rospy.init_node('waypoint_marker_node')
    WaypointMarkerNode()
    rospy.spin()
