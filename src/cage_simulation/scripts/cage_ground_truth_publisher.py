#!/usr/bin/env python3
"""Publish generated cage geometry on the isolated simulation-GT topic."""
import json
import math

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from cage_msgs.msg import LatticeCell, LatticeEdge, PersistentLattice


def point(values):
    values = list(values) + [0.0, 0.0, 0.0]
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


class GroundTruthPublisher:
    def __init__(self):
        path = rospy.get_param('~ground_truth_file')
        cage_id = int(rospy.get_param('~cage_id', 0))
        with open(path, encoding='utf-8') as stream:
            payload = json.load(stream)
        if 'cages' in payload:
            matches = [item for item in payload['cages'] if int(item['cage_id']) == cage_id]
            if len(matches) != 1:
                raise ValueError('expected one cage_id={} in {}'.format(cage_id, path))
            self.geometry = matches[0]
            self.map_uuid = payload.get('aggregate_sha256', self.geometry['geometry_sha256'])
        else:
            if int(payload.get('cage_id', 0)) != cage_id:
                raise ValueError('ground-truth cage id does not match requested cage')
            self.geometry = payload
            self.map_uuid = payload['geometry_sha256']
        self.publisher = rospy.Publisher('/sim_gt/cage/lattice', PersistentLattice,
                                         queue_size=1, latch=True)
        self.publish()

    def nominal_center(self, row, col):
        anchor = self.geometry['anchor_enu']
        cell_x, cell_y = self.geometry['cell_size_nominal']
        yaw = math.radians(float(self.geometry.get(
            'effective_yaw_deg', self.geometry.get('yaw_deg', 0.0))))
        local_x = (int(col) - 0.5) * float(cell_x)
        local_y = (int(row) - 0.5) * float(cell_y)
        return [
            anchor[0] + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            anchor[1] + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
            anchor[2],
        ]

    def publish(self):
        geometry = self.geometry
        message = PersistentLattice(
            header=Header(stamp=rospy.Time.now(), frame_id=geometry.get('frame_id', 'map')),
            cage_id=int(geometry['cage_id']),
            map_version=0,
            map_uuid=self.map_uuid,
            anchor_definition=geometry.get('anchor_definition', 'vertex_0_0'),
            anchor_enu=point(geometry['anchor_enu']),
            geometry_sha256=geometry['geometry_sha256'],
        )
        for source in geometry['cells']:
            cell = LatticeCell(
                row=int(source['row']), col=int(source['col']),
                cell_id=source['cell_id'],
                polygon=[point(value) for value in source.get('polygon_enu', [])],
                nominal_position=point(self.nominal_center(source['row'], source['col'])),
                estimated_position=point(source['center_enu']),
                covariance_xy=[0.0, 0.0, 0.0, 0.0],
                identity_state=LatticeCell.IDENTITY_CONFIRMED,
                visibility_state=LatticeCell.VISIBILITY_VISIBLE,
                identity_confidence=1.0,
                first_seen=message.header.stamp,
                last_seen=message.header.stamp,
                observation_count=1,
                exists=bool(source.get('exists', True)),
            )
            message.cells.append(cell)
        by_id = {cell['cell_id']: cell for cell in geometry['cells']}
        emitted = set()
        for source in geometry['cells']:
            for neighbor in source.get('neighbors', []):
                key = tuple(sorted((source['cell_id'], neighbor)))
                if key in emitted or neighbor not in by_id:
                    continue
                emitted.add(key)
                target = by_id[neighbor]
                relation = (LatticeEdge.RELATION_ROW
                            if int(source['row']) != int(target['row'])
                            else LatticeEdge.RELATION_COLUMN)
                message.edges.append(LatticeEdge(
                    from_cell_id=key[0], to_cell_id=key[1], relation=relation))
        self.publisher.publish(message)
        rospy.loginfo('Published isolated cage GT: cage=%d cells=%d sha256=%s',
                      message.cage_id, len(message.cells), message.geometry_sha256)


if __name__ == '__main__':
    rospy.init_node('cage_ground_truth_publisher')
    GroundTruthPublisher()
    rospy.spin()
