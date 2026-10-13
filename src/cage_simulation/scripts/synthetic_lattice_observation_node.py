#!/usr/bin/env python3
"""Publish causal local-lattice observations without exposing global IDs."""
import math

import rospy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header

from cage_msgs.msg import LocalLatticeCell, LocalLatticeObservation, PersistentLattice
from cage_grid.oracle_observation import (
    OracleObservationGenerator,
    cells_in_oriented_footprint,
    cells_in_window,
)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def point(values):
    values = list(values) + [0.0, 0.0, 0.0]
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


class SyntheticObservationNode:
    def __init__(self):
        self.visibility_mode = str(rospy.get_param('~visibility_mode', 'window')).strip().lower()
        if self.visibility_mode not in ('window', 'pose'):
            raise ValueError('visibility_mode must be window or pose')
        self.generator = OracleObservationGenerator(
            random_seed=int(rospy.get_param('~random_seed', 42)),
            position_noise_std_m=float(rospy.get_param('~position_noise_std_m', 0.15)),
            dropout_probability=float(rospy.get_param('~dropout_probability', 0.0)),
            edge_observed_probability=float(rospy.get_param('~edge_observed_probability', 0.85)),
        )
        self.window = {
            'row_min': rospy.get_param('~row_min', 1),
            'row_max': rospy.get_param('~row_max', 3),
            'col_min': rospy.get_param('~col_min', 1),
            'col_max': rospy.get_param('~col_max', 4),
        }
        self.footprint_width = float(rospy.get_param('~footprint_width_m', 48.0))
        self.footprint_height = float(rospy.get_param('~footprint_height_m', 36.0))
        self.forced_drop_ids = list(rospy.get_param('~forced_drop_ids', []))
        self.drop_schedule = dict(rospy.get_param('~drop_schedule', {}))
        self.frame_seq = 0
        self.cells = []
        self.cage_id = int(rospy.get_param('~cage_id', 0))
        self.pose = None
        self.publisher = rospy.Publisher('/scout/local_lattice_observation',
                                         LocalLatticeObservation, queue_size=1)
        rospy.Subscriber('/sim_gt/cage/lattice', PersistentLattice, self.on_ground_truth,
                         queue_size=1)
        if self.visibility_mode == 'pose':
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.on_pose,
                             queue_size=1)
        rate = float(rospy.get_param('~publish_rate_hz', 5.0))
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.on_timer)

    def on_ground_truth(self, message):
        if message.cage_id != self.cage_id:
            return
        self.cells = [{
            'row': cell.row,
            'col': cell.col,
            'cell_id': cell.cell_id,
            'center_enu': [cell.estimated_position.x, cell.estimated_position.y,
                           cell.estimated_position.z],
            'polygon_enu': [[value.x, value.y, value.z] for value in cell.polygon],
            'exists': cell.exists,
        } for cell in message.cells]

    def on_pose(self, message):
        self.pose = message

    def visible_cells(self):
        if self.visibility_mode == 'window':
            return cells_in_window(self.cells, **self.window)
        if self.pose is None:
            return []
        position = self.pose.pose.position
        yaw = yaw_from_quaternion(self.pose.pose.orientation)
        return cells_in_oriented_footprint(
            self.cells, (position.x, position.y), yaw,
            self.footprint_width, self.footprint_height)

    def on_timer(self, _event):
        if not self.cells:
            return
        self.frame_seq += 1
        scheduled = self.drop_schedule.get(str(self.frame_seq), [])
        forced = list(self.forced_drop_ids) + list(scheduled)
        visible = self.visible_cells()
        result = self.generator.generate(visible, self.frame_seq, forced)
        message = LocalLatticeObservation(
            header=Header(stamp=rospy.Time.now(), frame_id='map'),
            frame_seq=self.frame_seq,
            cage_id=self.cage_id,
            observation_source='oracle_noisy',
            camera_frame='uav_camera_optical_frame',
            local_rows=result['local_rows'],
            local_cols=result['local_cols'],
            orientation_normalized=True,
        )
        for source in result['cells']:
            message.cells.append(LocalLatticeCell(
                local_i=source['local_i'], local_j=source['local_j'],
                quad_uv=[point(value) for value in source['quad_uv']],
                projected_center=point(source['projected_center']),
                projected_covariance_xy=source['projected_covariance_xy'],
                edge_state=source['edge_state'],
                confidence=source['confidence'],
                expected_visible=source['expected_visible'],
            ))
        # This polygon is in map XY for oracle mode.  A real visual adapter
        # publishes image-pixel UV using the same Point container.
        if visible:
            xs = [cell['center_enu'][0] for cell in visible]
            ys = [cell['center_enu'][1] for cell in visible]
            message.fov_polygon_uv = [
                point((min(xs), min(ys), 0.0)), point((max(xs), min(ys), 0.0)),
                point((max(xs), max(ys), 0.0)), point((min(xs), max(ys), 0.0)),
            ]
        self.publisher.publish(message)


if __name__ == '__main__':
    rospy.init_node('synthetic_lattice_observation_node')
    SyntheticObservationNode()
    rospy.spin()
