#!/usr/bin/env python3
"""Write replay-friendly Scout/Feeder/task decisions as JSON Lines."""
import json
import os
import threading

import rospy
from std_msgs.msg import String

from cage_msgs.msg import (FeederIdentityState, MissionState,
                           OffsetHypothesisArray, PersistentLattice,
                           ScoutMapActivationStatus,
                           ScoutTaskStatus)


class PersistentLatticeEventLogger:
    def __init__(self):
        path = os.path.abspath(os.path.expanduser(
            rospy.get_param('~output', '/tmp/cage_persistent_lattice_events.jsonl')))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.stream = open(path, 'a', encoding='utf-8', buffering=1)
        self.lock = threading.Lock()
        self.context = {
            'schema_version': 1,
            'scenario': rospy.get_param('~scenario', 'interactive'),
            'seed': int(rospy.get_param('/persistent_lattice/deformation_seed', 42)),
            'preset': rospy.get_param('/persistent_lattice/deformation_preset', 'medium'),
        }
        rospy.Subscriber('/scout/map_events', String, self.on_map_event, queue_size=100)
        rospy.Subscriber('/scout/offset_hypotheses', OffsetHypothesisArray,
                         self.on_offsets, queue_size=100)
        rospy.Subscriber('/scout/working_lattice_map', PersistentLattice,
                         self.on_working_map, queue_size=10)
        rospy.Subscriber('/scout/lattice_map', PersistentLattice,
                         self.on_active_map, queue_size=10)
        rospy.Subscriber('/scout/active_map_status', ScoutMapActivationStatus,
                         self.on_active_status, queue_size=10)
        rospy.Subscriber('/feeder/identity_state', FeederIdentityState,
                         self.on_feeder, queue_size=100)
        rospy.Subscriber('/mission/state', MissionState,
                         self.on_mission, queue_size=100)
        rospy.Subscriber('/scout/task_status', ScoutTaskStatus,
                         self.on_scout_task, queue_size=100)
        rospy.on_shutdown(self.close)
        rospy.loginfo('Persistent lattice JSONL log: %s', path)

    def write(self, value, durable=False):
        if not value.get('record_type'):
            raise ValueError('event-log record_type is required')
        record = dict(self.context)
        record.update(value)
        with self.lock:
            self.stream.write(json.dumps(record, sort_keys=True) + '\n')
            if durable:
                self.stream.flush()
                os.fsync(self.stream.fileno())

    def on_map_event(self, message):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            value = {'raw': message.data}
        event_type = str(value.get('type', '')).upper()
        self.write(
            {'record_type': 'map_event', 'event': value},
            durable=event_type in ('TASK_START', 'TASK_END'))

    def on_offsets(self, message):
        self.write({
            'record_type': 'offset_hypotheses',
            'stamp': message.header.stamp.to_sec(),
            'frame_seq': message.frame_seq,
            'committed': message.committed,
            'committed_offset': [message.committed_delta_row,
                                 message.committed_delta_col],
            'reason': message.decision_reason,
            'hypotheses': [{
                'offset': [value.delta_row, value.delta_col],
                'posterior': value.posterior,
                'geometry_likelihood': value.geometry_likelihood,
                'topology_likelihood': value.topology_likelihood,
                'motion_likelihood': value.motion_likelihood,
            } for value in message.hypotheses],
        })

    def on_working_map(self, message):
        self.on_map(message, 'working_map_snapshot')

    def on_active_map(self, message):
        self.on_map(message, 'active_map_snapshot')

    def on_map(self, message, record_type):
        self.write({
            'record_type': record_type,
            'stamp': message.header.stamp.to_sec(),
            'map_uuid': message.map_uuid,
            'map_version': message.map_version,
            'geometry_sha256': message.geometry_sha256,
            'cage_id': message.cage_id,
            'confirmed_cells': sum(
                1 for value in message.cells
                if value.identity_state == value.IDENTITY_CONFIRMED),
            'tentative_cells': sum(
                1 for value in message.cells
                if value.identity_state == value.IDENTITY_TENTATIVE),
            'rejected_cells': sum(
                1 for value in message.cells
                if value.identity_state == value.IDENTITY_REJECTED),
            'cells': [{
                'row': value.row,
                'col': value.col,
                'cell_id': value.cell_id,
                'identity_state': value.identity_state,
                'visibility_state': value.visibility_state,
                'nominal_position': [
                    value.nominal_position.x,
                    value.nominal_position.y,
                    value.nominal_position.z,
                ],
                'estimated_position': [
                    value.estimated_position.x,
                    value.estimated_position.y,
                    value.estimated_position.z,
                ],
                'covariance_xy': list(value.covariance_xy),
                'identity_confidence': value.identity_confidence,
                'first_seen': value.first_seen.to_sec(),
                'last_seen': value.last_seen.to_sec(),
                'observation_count': value.observation_count,
            } for value in message.cells],
            'edges': [{
                'from_cell_id': value.from_cell_id,
                'to_cell_id': value.to_cell_id,
                'relation': value.relation,
            } for value in message.edges],
        })

    def on_active_status(self, message):
        self.write({
            'record_type': 'active_map_status',
            'stamp': message.header.stamp.to_sec(),
            'status': message.status,
            'task_id': message.task_id,
            'map_uuid': message.map_uuid,
            'map_version': message.map_version,
            'map_sha256': message.map_sha256,
            'manifest_sha256': message.manifest_sha256,
            'compatible': message.compatible,
            'reason_code': message.reason_code,
            'message': message.message,
        })

    def on_feeder(self, message):
        labels = {
            FeederIdentityState.STATUS_CONFIRMED: 'CONFIRMED',
            FeederIdentityState.STATUS_AMBIGUOUS: 'AMBIGUOUS',
            FeederIdentityState.STATUS_LOST: 'LOST',
            FeederIdentityState.STATUS_STALE_MAP: 'STALE_MAP',
            FeederIdentityState.STATUS_MAP_MISMATCH: 'MAP_MISMATCH',
        }
        self.write({
            'record_type': 'feeder_decision',
            'stamp': message.header.stamp.to_sec(),
            'status': labels.get(message.status, 'UNKNOWN'),
            'map_version': message.map_version,
            'task_map_version': message.task_map_version,
            'previous_cell_id': message.previous_cell_id,
            'current_cell_id': message.current_cell_id,
            'next_cell_id': message.next_cell_id,
            'reason': message.decision_reason,
            'hypotheses': [{
                'cell_id': value.cell_id,
                'posterior': value.posterior,
                'gps_likelihood': value.gps_likelihood,
                'visual_likelihood': value.visual_likelihood,
                'transition_prior': value.transition_prior,
            } for value in message.hypotheses],
        })

    def on_mission(self, message):
        self.write({
            'record_type': 'mission_state',
            'stamp': message.header.stamp.to_sec(),
            'mission_id': message.mission_id,
            'state': message.state,
            'reason': message.reason,
            'current_cell_id': message.current_cell_id,
            'feeder_enabled': message.feeder_enabled,
            'mission_type': getattr(message, 'mission_type', 0),
            'auto_land': getattr(message, 'auto_land', False),
        })

    def on_scout_task(self, message):
        terminal = message.state in ('COMPLETED', 'FAILED', 'CANCELED')
        self.write({
            'record_type': 'scout_task_status',
            'stamp': message.header.stamp.to_sec(),
            'task_id': message.task_id,
            'mission_id': message.mission_id,
            'cage_id': message.cage_id,
            'state': message.state,
            'reason': message.reason,
            'error_code': message.error_code,
            'current_waypoint': message.current_waypoint,
            'total_waypoints': message.total_waypoints,
            'route_completed': message.route_completed,
            'map_uuid': message.map_uuid,
            'map_version': message.map_version,
            'geometry_sha256': message.geometry_sha256,
            'confirmed_cells': message.confirmed_cells,
            'expected_cells': message.expected_cells,
            'coverage': message.coverage,
            'map_finalized': message.map_finalized,
            'map_ready_for_feeding': message.map_ready_for_feeding,
            'rtl_commanded': message.rtl_commanded,
            'rtl_accepted': message.rtl_accepted,
            'returned_home': message.returned_home,
            'home_horizontal_error_m': message.home_horizontal_error_m,
            'landed': message.landed,
            'disarmed': message.disarmed,
            'output_directory': message.output_directory,
        }, durable=terminal)

    def close(self):
        with self.lock:
            if not self.stream.closed:
                self.stream.close()


if __name__ == '__main__':
    rospy.init_node('persistent_lattice_event_logger')
    PersistentLatticeEventLogger()
    rospy.spin()
