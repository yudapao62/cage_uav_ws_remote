#!/usr/bin/env python3
import math
import statistics
from collections import deque
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from cage_msgs.msg import CageGeometry, CageGrid, CellTargetArray, FeedingTaskStatus, MissionState
from cage_grid.model import CageGridModel, CircularCageGeometry, normalize_geometry_mode
from cage_geo.geodesy import LocalENU


class GridNode:
    def __init__(self):
        p = lambda n, d: rospy.get_param('/cage_grid/' + n, d)
        self.geometry_mode = normalize_geometry_mode(rospy.get_param('/cage/geometry_mode', 'rectangular'))
        self.beacon_mode = p('geometry_source', 'fixed') == 'beacon'
        self.model = None if self.beacon_mode else CageGridModel(int(p('cage_id', 0)), int(p('rows', 6)), int(p('cols', 10)),
            p('origin_x', 70.0), p('origin_y', -6.0), p('origin_z', 0.0),
            p('cell_size_x', 12.0), p('cell_size_y', 12.0), math.radians(p('yaw_deg', 0.0)),
            (p('reference_lat', -35.363261), p('reference_lon', 149.165230), p('reference_alt', 584.0)))
        self.home = None; self.geometry = None; self.geometry_received = 0.0; self.frozen = False
        self.gps_samples = deque(maxlen=int(p('home_stable_samples', 10)))
        self.home_pub = rospy.Publisher('/cage/home', NavSatFix, queue_size=1, latch=True)
        self.target_ids = set(); self.current = ''; self.completed = set(); self.batch_seq = None
        self.grid_pub = rospy.Publisher('/cage/grid_info', CageGrid, queue_size=1, latch=True)
        self.geometry_pub = rospy.Publisher('/cage/geometry', CageGeometry, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('/cage/grid/markers', MarkerArray, queue_size=1, latch=True)
        self.target_pub = rospy.Publisher('/feeding/target_cell_markers', MarkerArray, queue_size=1, latch=True)
        self.target_sub = rospy.Subscriber('/feeding/target_cells', CellTargetArray, self.on_targets, queue_size=1)
        rospy.Subscriber('/mission/state', MissionState, self.on_state, queue_size=1)
        if self.beacon_mode:
            rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.on_gps, queue_size=1)
            rospy.Subscriber('/cage/geometry_validated', CageGeometry, self.on_geometry, queue_size=1)
            rospy.Subscriber('/feeding/task_status', FeedingTaskStatus, self.on_task, queue_size=1)
        else:
            self.publish_fixed_geometry()
            if self.geometry_mode == 'rectangular':
                self.publish_grid()
            self.publish_markers()
        rospy.loginfo('Cage geometry mode: %s', self.geometry_mode)

    def on_gps(self, msg):
        if self.home is not None or msg.status.status < NavSatStatus.STATUS_FIX:
            return
        values = (msg.latitude, msg.longitude, msg.altitude)
        if not all(math.isfinite(v) for v in values) or values[:2] == (0.0, 0.0): return
        self.gps_samples.append(values)
        if len(self.gps_samples) < self.gps_samples.maxlen: return
        median = tuple(statistics.median(row[i] for row in self.gps_samples) for i in range(3))
        local = LocalENU(*median)
        spread = max(math.hypot(*local.to_enu(*row)[:2]) for row in self.gps_samples)
        if spread > float(rospy.get_param('/cage_grid/home_max_spread_m', 2.0)): return
        self.home = NavSatFix(header=Header(stamp=rospy.Time.now()), latitude=median[0], longitude=median[1], altitude=median[2])
        self.home.status.status = NavSatStatus.STATUS_FIX
        self.home_pub.publish(self.home)
        self.update_beacon_model()

    def on_geometry(self, msg):
        self.geometry = msg
        self.geometry_received = rospy.Time.now().to_sec()
        self.update_beacon_model()

    def on_task(self, msg):
        self.frozen = msg.task_state in ('TASK_READY','TASK_EXECUTING','TASK_REPEAT_WAIT','TASK_MANUAL_OVERRIDE')
        if not self.frozen: self.update_beacon_model()

    def update_beacon_model(self):
        if self.frozen or self.home is None or self.geometry is None: return
        if rospy.Time.now().to_sec()-self.geometry_received > float(rospy.get_param('/beacon/receive_timeout_sec',2.0)): return
        p = lambda n, d: rospy.get_param('/cage_grid/' + n, d)
        off = lambda n: rospy.get_param('/beacon/cage_offset/' + n, 0.0)
        self.publish_unified_geometry(self.geometry, (self.home.latitude, self.home.longitude, self.home.altitude))
        if self.geometry_mode == 'circular':
            self.model = None
            self.publish_markers()
            return
        candidate = CageGridModel.from_beacon(self.geometry, self.home,
            int(p('rows', 6)), int(p('cols', 10)), float(p('cell_size_x', 12.0)), float(p('cell_size_y', 12.0)),
            off('longitudinal_m'), off('lateral_m'), off('yaw_offset_deg'))
        if self.model is not None:
            moved=math.hypot(candidate.origin_x-self.model.origin_x,candidate.origin_y-self.model.origin_y)
            turned=abs(math.atan2(math.sin(candidate.yaw-self.model.yaw),math.cos(candidate.yaw-self.model.yaw)))
            if moved < float(p('grid_update_min_translation_m',0.25)) and turned < math.radians(float(p('grid_update_min_yaw_deg',0.5))):
                return
        self.model = candidate
        self.publish_grid(); self.publish_markers()
        if self.target_ids: self.publish_target_markers()

    def on_targets(self, msg):
        if self.geometry_mode == 'circular':
            self.target_ids = set(); self.publish_target_markers(); return
        if self.model is None: return
        if msg.cage_id != self.model.cage_id: return
        self.batch_seq = msg.header.seq; self.target_ids = {target.cell_id for target in msg.targets}; self.completed = set(); self.current = ''
        self.publish_target_markers()

    def on_state(self, msg):
        self.current = msg.current_cell_id
        if msg.state in ('FINISHED', 'RTL', 'LAND') and msg.current_cell_id:
            self.completed.add(msg.current_cell_id)
        if self.model is not None: self.publish_target_markers()

    def publish_grid(self):
        if self.geometry_mode != 'rectangular' or self.model is None: return
        self.grid_pub.publish(CageGrid(header=Header(stamp=rospy.Time.now(), frame_id='map'),
            cage_id=self.model.cage_id, rows=self.model.rows, cols=self.model.cols,
            origin_x=self.model.origin_x, origin_y=self.model.origin_y, origin_z=self.model.origin_z,
            cell_size_x=self.model.cell_size_x, cell_size_y=self.model.cell_size_y, yaw=self.model.yaw))

    def publish_fixed_geometry(self):
        """Expose config-derived A/B through the same geometry topic as real flight."""
        length = self.model.cols * self.model.cell_size_x
        ax, ay = self.model.origin_x, self.model.origin_y
        bx = ax + math.cos(self.model.yaw) * length
        by = ay + math.sin(self.model.yaw) * length
        a = self.model.local.to_wgs84(ax, ay, self.model.origin_z)
        b = self.model.local.to_wgs84(bx, by, self.model.origin_z)
        source = CageGeometry(cage_id=self.model.cage_id,
            endpoint_a_lat=a[0], endpoint_a_lon=a[1], endpoint_a_alt=a[2],
            endpoint_b_lat=b[0], endpoint_b_lon=b[1], endpoint_b_alt=b[2],
            width=self.model.rows*self.model.cell_size_y,
            rows=self.model.rows, cols=self.model.cols)
        self.publish_unified_geometry(source, self.model.reference)

    def publish_unified_geometry(self, source, reference):
        circular = CircularCageGeometry.from_wgs84(
            (source.endpoint_a_lat, source.endpoint_a_lon, source.endpoint_a_alt),
            (source.endpoint_b_lat, source.endpoint_b_lon, source.endpoint_b_alt), reference)
        out = CageGeometry(header=Header(stamp=rospy.Time.now(), frame_id='map'),
            cage_id=source.cage_id, geometry_mode=self.geometry_mode,
            endpoint_a_lat=source.endpoint_a_lat, endpoint_a_lon=source.endpoint_a_lon,
            endpoint_a_alt=source.endpoint_a_alt, endpoint_b_lat=source.endpoint_b_lat,
            endpoint_b_lon=source.endpoint_b_lon, endpoint_b_alt=source.endpoint_b_alt,
            center_lat=circular.center_wgs84[0], center_lon=circular.center_wgs84[1],
            center_alt=circular.center_wgs84[2], diameter_m=circular.diameter,
            radius_m=circular.radius, width=source.width, rows=source.rows, cols=source.cols)
        self.geometry_pub.publish(out)
        if self.geometry_mode == 'circular':
            rospy.loginfo_throttle(5.0, '[CIRCULAR] center=(%.8f, %.8f), diameter=%.2f m',
                                   out.center_lat, out.center_lon, out.diameter_m)

    def marker(self, mid, typ, x, y, z, color, scale, text=''):
        m = Marker(header=Header(stamp=rospy.Time.now(), frame_id='map'), ns='cage_grid', id=mid, type=typ,
                   action=Marker.ADD)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.color.r, m.color.g, m.color.b, m.color.a = color; m.scale.x, m.scale.y, m.scale.z = scale; m.text = text
        return m

    def publish_markers(self):
        a = MarkerArray(); mid = 0
        clear = Marker(header=Header(stamp=rospy.Time.now(), frame_id='map'), ns='cage_grid', id=0, action=Marker.DELETEALL)
        a.markers.append(clear)
        if self.geometry_mode == 'circular' or self.model is None:
            self.marker_pub.publish(a)
            return
        # Cell outlines as thin line strips; all geometry is shared with the model.
        for cell in self.model.all_cells():
            cx, cy = cell.x, cell.y; c, s = math.cos(self.model.yaw), math.sin(self.model.yaw)
            hx, hy = self.model.cell_size_x/2.0, self.model.cell_size_y/2.0
            corners = [(cx+c*dx-s*dy, cy+s*dx+c*dy, 0.25) for dx,dy in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy),(-hx,-hy)]]
            m = Marker(header=Header(stamp=rospy.Time.now(), frame_id='map'), ns='cage_grid', id=mid,
                       type=Marker.LINE_STRIP, action=Marker.ADD); mid += 1
            m.scale.x=0.10; m.color.r=m.color.g=m.color.b=0.72; m.color.a=0.75
            for x,y,z in corners: m.points.append(Point(x=x,y=y,z=z))
            a.markers.append(m)
            center = self.marker(mid, Marker.SPHERE, cell.x, cell.y, 0.35, (0.65,0.68,0.68,0.45), (0.45,0.45,0.45)); mid += 1
            a.markers.append(center)
            label = self.marker(mid, Marker.TEXT_VIEW_FACING, cell.x, cell.y, 1.0, (0.85,0.90,0.88,0.42), (0.0,0.0,0.9), cell.cell_id); mid += 1
            a.markers.append(label)
        self.marker_pub.publish(a)

    def publish_target_markers(self):
        a=MarkerArray(); mid=0
        a.markers.append(Marker(header=Header(stamp=rospy.Time.now(), frame_id='map'), ns='cage_grid_targets', id=0, action=Marker.DELETEALL))
        if self.geometry_mode == 'circular' or self.model is None:
            self.target_pub.publish(a)
            return
        for cell in self.model.all_cells():
            if cell.cell_id not in self.target_ids: continue
            m=self.marker(mid, Marker.CUBE, cell.x, cell.y, 0.20,
                          (1.0,0.75,0.05,0.90) if cell.cell_id==self.current else ((0.20,0.80,0.25,0.55) if cell.cell_id not in self.completed else (0.3,0.3,0.3,0.35)),
                          (self.model.cell_size_x*0.85,self.model.cell_size_y*0.85,0.04)); mid+=1; a.markers.append(m)
            if cell.cell_id == self.current:
                a.markers.append(self.marker(mid, Marker.TEXT_VIEW_FACING, cell.x, cell.y, 2.0, (1.0,0.9,0.1,1.0), (0.0,0.0,1.2), 'CURRENT')); mid += 1
        self.target_pub.publish(a)


if __name__ == '__main__':
    rospy.init_node('cage_grid_node'); GridNode(); rospy.spin()
