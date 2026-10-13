#!/usr/bin/env python3
"""Display cage geometry plus persistent Scout/Feeder lattice state in RViz."""
import math

import rospy
from cage_geo.geodesy import LocalENU
from cage_msgs.msg import CageGeometry, FeederIdentityState, PersistentLattice
from cage_visualization.geometry import operation_zone
from cage_visualization.markers import color, marker, point, text_marker
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray


class CageMarkerNode:
    NS = 'cage_geometry'

    def __init__(self):
        self.frame_id = rospy.get_param('/visualization/frame_id', 'map')
        self.realflight = rospy.get_param('/cage_grid/geometry_source','fixed') == 'beacon'
        self.local_enu = None if self.realflight else LocalENU(
            rospy.get_param('/visualization/home_lat'),
            rospy.get_param('/visualization/home_lon'),
            rospy.get_param('/visualization/home_alt', 0.0))
        self.samples = rospy.get_param('/visualization/circle_samples', 100)
        self.gps_scale = rospy.get_param('/visualization/gps_marker_scale', 1.2)
        self.center_scale = rospy.get_param('/visualization/center_marker_scale', 0.8)
        self.text_height = rospy.get_param('/visualization/text_height', 1.2)
        self.diameter_width = rospy.get_param('/visualization/line_width/diameter', 0.20)
        self.circle_width = rospy.get_param('/visualization/line_width/circle', 0.15)
        self.show = rospy.get_param('/visualization/show', {})
        self.cage_cfg = rospy.get_param('/visualization/cage', {})
        self.sea_cfg = rospy.get_param('/visualization/sea_plane', {})
        self.publisher = rospy.Publisher('/cage/visualization/markers', MarkerArray,
                                         queue_size=1, latch=True)
        self.lattice_publisher = rospy.Publisher(
            '/persistent_lattice/visualization/markers', MarkerArray,
            queue_size=1, latch=True)
        self.latest_lattice = None
        self.latest_identity = None
        rospy.Subscriber('/cage/geometry', CageGeometry, self.on_geometry, queue_size=1)
        rospy.Subscriber('/scout/lattice_map', PersistentLattice,
                         self.on_lattice, queue_size=1)
        rospy.Subscriber('/feeder/identity_state', FeederIdentityState,
                         self.on_feeder_identity, queue_size=1)
        if self.realflight:
            rospy.Subscriber('/cage/home', NavSatFix, self.on_home, queue_size=1)

    def on_home(self, msg):
        self.local_enu = LocalENU(msg.latitude, msg.longitude, msg.altitude)

    @staticmethod
    def _xy_marker(frame_id, namespace, marker_id, marker_type, stamp,
                   scale, rgba):
        value = marker(frame_id, namespace, marker_id, marker_type, stamp)
        value.scale.x = scale
        if marker_type in (Marker.SPHERE_LIST, Marker.CUBE_LIST):
            value.scale.y = scale
            value.scale.z = scale
        value.color = color(*rgba)
        return value

    def on_lattice(self, msg):
        self.latest_lattice = msg
        self.publish_persistent_lattice()

    def on_feeder_identity(self, msg):
        self.latest_identity = msg
        self.publish_persistent_lattice()

    def publish_persistent_lattice(self):
        msg = self.latest_lattice
        if msg is None:
            return
        stamp = rospy.Time.now()
        frame_id = msg.header.frame_id or self.frame_id
        markers = []
        by_id = {cell.cell_id: cell for cell in msg.cells}

        nominal = self._xy_marker(
            frame_id, 'lattice_nominal', 0, Marker.LINE_LIST, stamp,
            0.045, (0.55, 0.58, 0.60, 0.50))
        estimated = self._xy_marker(
            frame_id, 'lattice_estimated', 0, Marker.LINE_LIST, stamp,
            0.14, (0.05, 0.85, 0.95, 0.95))
        for edge in msg.edges:
            source, target = by_id.get(edge.from_cell_id), by_id.get(edge.to_cell_id)
            if source is None or target is None:
                continue
            nominal.points.extend([
                point(source.nominal_position.x, source.nominal_position.y, 0.65),
                point(target.nominal_position.x, target.nominal_position.y, 0.65)])
            estimated.points.extend([
                point(source.estimated_position.x, source.estimated_position.y, 0.78),
                point(target.estimated_position.x, target.estimated_position.y, 0.78)])
        markers.extend([nominal, estimated])

        states = {
            'tentative': (0, (1.0, 0.78, 0.10, 0.92)),
            'visible': (1, (0.15, 0.95, 0.32, 0.95)),
            'out_of_view': (2, (0.15, 0.45, 1.0, 0.88)),
            'missed': (3, (1.0, 0.42, 0.08, 0.95)),
            'rejected': (4, (1.0, 0.08, 0.12, 0.95)),
        }
        state_markers = {
            name: self._xy_marker(frame_id, 'lattice_cells_'+name, marker_id,
                                  Marker.SPHERE_LIST, stamp, 0.80, rgba)
            for name, (marker_id, rgba) in states.items()}
        covariance = self._xy_marker(
            frame_id, 'lattice_covariance', 0, Marker.LINE_LIST, stamp,
            0.035, (0.95, 0.95, 0.25, 0.72))
        for cell in msg.cells:
            if cell.identity_state == cell.IDENTITY_REJECTED:
                name = 'rejected'
            elif cell.identity_state == cell.IDENTITY_TENTATIVE:
                name = 'tentative'
            elif cell.visibility_state == cell.VISIBILITY_VISIBLE:
                name = 'visible'
            elif cell.visibility_state == cell.VISIBILITY_EXPECTED_BUT_UNOBSERVED:
                name = 'missed'
            else:
                name = 'out_of_view'
            center = point(cell.estimated_position.x, cell.estimated_position.y, 1.00)
            state_markers[name].points.append(center)

            radius_x = 2.0 * math.sqrt(max(0.0, cell.covariance_xy[0]))
            radius_y = 2.0 * math.sqrt(max(0.0, cell.covariance_xy[3]))
            previous = None
            for sample in range(25):
                angle = 2.0 * math.pi * sample / 24.0
                current = point(cell.estimated_position.x + radius_x * math.cos(angle),
                                cell.estimated_position.y + radius_y * math.sin(angle),
                                0.90)
                if previous is not None:
                    covariance.points.extend([previous, current])
                previous = current
        markers.extend(state_markers.values())
        markers.append(covariance)

        identity = self.latest_identity
        candidates = self._xy_marker(
            frame_id, 'feeder_candidates', 0, Marker.SPHERE_LIST, stamp,
            1.80, (1.0, 0.15, 0.75, 0.60))
        if identity is not None:
            for hypothesis in identity.hypotheses:
                cell = by_id.get(hypothesis.cell_id)
                if cell is None:
                    continue
                candidates.points.append(point(
                    cell.estimated_position.x, cell.estimated_position.y, 1.65))
                candidates.colors.append(color(
                    1.0, 0.12, 0.72, max(0.10, min(1.0, hypothesis.posterior))))
            labels = {
                FeederIdentityState.STATUS_CONFIRMED: 'CONFIRMED',
                FeederIdentityState.STATUS_AMBIGUOUS: 'AMBIGUOUS',
                FeederIdentityState.STATUS_LOST: 'LOST',
                FeederIdentityState.STATUS_STALE_MAP: 'STALE_MAP',
            }
            summary = text_marker(
                frame_id, 'feeder_identity', 0, stamp,
                point(msg.anchor_enu.x, msg.anchor_enu.y, 4.0),
                'Feeder {}  top1={}  top2={}\nmap={} task_map={}'.format(
                    labels.get(identity.status, 'UNKNOWN'), identity.top1_cell_id,
                    identity.top2_cell_id, identity.map_version,
                    identity.task_map_version),
                1.2, color(1.0, 0.9, 0.2, 1.0))
            markers.append(summary)
        markers.append(candidates)
        self.lattice_publisher.publish(MarkerArray(markers=markers))

    def on_geometry(self, msg):
        if (msg.geometry_mode or rospy.get_param('/cage/geometry_mode', 'rectangular')) != 'circular':
            clear = marker(self.frame_id, self.NS, 0, Marker.SPHERE, rospy.Time.now())
            clear.action = Marker.DELETEALL
            self.publisher.publish(MarkerArray(markers=[clear]))
            return
        if self.local_enu is None:
            return
        ax, ay, _ = self.local_enu.to_enu(
            msg.endpoint_a_lat, msg.endpoint_a_lon, msg.endpoint_a_alt)
        bx, by, _ = self.local_enu.to_enu(
            msg.endpoint_b_lat, msg.endpoint_b_lon, msg.endpoint_b_alt)
        zone = operation_zone((ax, ay), (bx, by), msg.width, self.samples)
        stamp = rospy.Time.now()
        markers = []

        gps_a = marker(self.frame_id, self.NS, 0, Marker.SPHERE, stamp)
        gps_a.pose.position = point(ax, ay, 0.65)
        gps_a.scale.x = gps_a.scale.y = gps_a.scale.z = self.gps_scale
        gps_a.color = color(0.15, 0.85, 0.30, 1.0)
        markers.append(gps_a)

        gps_b = marker(self.frame_id, self.NS, 1, Marker.SPHERE, stamp)
        gps_b.pose.position = point(bx, by, 0.65)
        gps_b.scale.x = gps_b.scale.y = gps_b.scale.z = self.gps_scale
        gps_b.color = color(1.0, 0.50, 0.10, 1.0)
        markers.append(gps_b)

        diameter = marker(self.frame_id, self.NS, 2, Marker.LINE_STRIP, stamp)
        diameter.scale.x = self.diameter_width
        diameter.color = color(0.85, 0.95, 1.0, 0.90)
        ring_height = float(self.cage_cfg.get('ring_height', 0.35))
        diameter.points = [point(ax, ay, ring_height), point(bx, by, ring_height)]
        markers.append(diameter)

        cage_operation_zone = marker(self.frame_id, self.NS, 3, Marker.LINE_STRIP, stamp)
        cage_operation_zone.scale.x = self.circle_width
        cage_operation_zone.color = color(0.92, 0.95, 0.97, 0.95)
        cage_operation_zone.points = [point(x, y, ring_height) for x, y in zone.circle]
        markers.append(cage_operation_zone)

        # A low-density sphere ring makes the circular cage readable from an
        # oblique 3D view while keeping the geometric source identical to the
        # diameter-defined circle above.
        ring = marker(self.frame_id, self.NS, 12, Marker.SPHERE_LIST, stamp)
        ring.scale.x = ring.scale.y = ring.scale.z = float(
            self.cage_cfg.get('ring_buoy_scale', 0.70))
        ring.color = color(0.88, 0.92, 0.95, 0.92)
        ring.points = [point(x, y, ring_height) for x, y in zone.circle[:-1]]
        markers.append(ring)

        if self.show.get('cage_center', True):
            center = marker(self.frame_id, self.NS, 4, Marker.SPHERE, stamp)
            center.pose.position = point(zone.center[0], zone.center[1], ring_height + 0.25)
            center.scale.x = center.scale.y = center.scale.z = self.center_scale
            center.color = color(1.0, 0.95, 0.45, 1.0)
            markers.append(center)

        if self.show.get('cage_direction', True):
            direction = marker(self.frame_id, self.NS, 5, Marker.ARROW, stamp)
            direction.scale.x = 0.30
            direction.scale.y = 1.00
            direction.scale.z = 1.50
            direction.color = color(0.20, 0.80, 1.0, 0.90)
            direction.points = [point(ax, ay, ring_height + 0.15),
                                point(bx, by, ring_height + 0.15)]
            markers.append(direction)

        if self.show.get('cage_rectangle', False) and self.cage_cfg.get('show_rectangle', False):
            rectangle = marker(self.frame_id, self.NS, 6, Marker.LINE_STRIP, stamp)
            rectangle.scale.x = 0.22
            rectangle.color = color(0.92, 0.92, 0.95, 0.72)
            rectangle.points = [point(x, y, ring_height) for x, y in zone.rectangle]
            markers.append(rectangle)
        else:
            rectangle = marker(self.frame_id, self.NS, 6, Marker.LINE_STRIP, stamp)
            rectangle.action = Marker.DELETE
            markers.append(rectangle)

        if self.show.get('gps_labels', True):
            markers.append(text_marker(self.frame_id, self.NS, 7, stamp,
                point(ax + 3.0 * zone.normal[0], ay + 3.0 * zone.normal[1], 2.2),
                'GPS_A', self.text_height,
                color(0.15, 0.95, 0.35, 1.0)))
            markers.append(text_marker(self.frame_id, self.NS, 8, stamp,
                point(bx + 3.0 * zone.normal[0], by + 3.0 * zone.normal[1], 2.2),
                'GPS_B', self.text_height,
                color(1.0, 0.60, 0.15, 1.0)))
        if self.show.get('cage_center', True):
            markers.append(text_marker(self.frame_id, self.NS, 9, stamp,
                point(zone.center[0] - 3.0 * zone.normal[0],
                      zone.center[1] - 3.0 * zone.normal[1], 2.0), 'CAGE_CENTER',
                self.text_height, color(1.0, 1.0, 0.70, 1.0)))

        feed = marker(self.frame_id, self.NS, 10, Marker.SPHERE, stamp)
        feed.pose.position = point(zone.center[0], zone.center[1], ring_height + 0.45)
        feed.scale.x = feed.scale.y = feed.scale.z = 1.6
        feed.color = color(0.25, 1.0, 0.45, 0.72)
        markers.append(feed)
        markers.append(text_marker(self.frame_id, self.NS, 11, stamp,
            point(zone.center[0] + 5.0 * zone.normal[0],
                  zone.center[1] + 5.0 * zone.normal[1], 3.4), 'FEED_POINT',
            self.text_height, color(0.45, 1.0, 0.55, 1.0)))

        if self.sea_cfg.get('enabled', True):
            sea = marker(self.frame_id, self.NS, 13, Marker.CUBE, stamp)
            sea.pose.position = point(0.0, 0.0, -float(self.sea_cfg.get('thickness', 0.02)) / 2.0)
            sea.scale.x = sea.scale.y = float(self.sea_cfg.get('size', 1000.0))
            sea.scale.z = float(self.sea_cfg.get('thickness', 0.02))
            sea.color = color(0.32, 0.68, 0.86, float(self.sea_cfg.get('alpha', 0.32)))
            markers.append(sea)

        self.publisher.publish(MarkerArray(markers=markers))
        rospy.loginfo('CAGE_%02d visualization: A=(%.2f, %.2f), B=(%.2f, %.2f), R=%.2f m',
                      msg.cage_id, ax, ay, bx, by, zone.radius)


if __name__ == '__main__':
    rospy.init_node('cage_marker_node')
    CageMarkerNode()
    rospy.spin()
