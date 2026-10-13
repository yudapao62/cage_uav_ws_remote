#!/usr/bin/env python3
"""Maintain persistent row/column identities from local lattice observations."""
import hashlib
import json
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String

from cage_grid.association import IntegerOffsetAssociator
from cage_grid.coordinate_optimizer import CoordinateOptimizer
from cage_grid.model import CageGridModel
from cage_grid.persistent_map import IDENTITY_CONFIRMED, PersistentLatticeMap
from cage_msgs.msg import (
    LatticeCell,
    LatticeEdge,
    LocalLatticeObservation,
    OffsetHypothesis,
    OffsetHypothesisArray,
    PersistentLattice,
)
from cage_msgs.srv import (
    FinalizeScoutMap,
    FinalizeScoutMapResponse,
    ResetScoutMap,
    ResetScoutMapResponse,
)


def point(values):
    values = list(values) + [0.0, 0.0, 0.0]
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


class ScoutLatticeNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.finalize_in_progress = False
        self.finalize_cache = {}
        p = lambda name, default: rospy.get_param('/persistent_lattice/' + name, default)
        g = lambda name, default: rospy.get_param('/cage_grid/' + name, default)
        self.cage_id = int(g('cage_id', 0))
        rows, cols = int(g('rows', 6)), int(g('cols', 10))
        self.model = CageGridModel(
            self.cage_id, rows, cols,
            g('origin_x', 70.0), g('origin_y', -6.0), g('origin_z', 0.0),
            g('cell_size_x', 12.0), g('cell_size_y', 12.0),
            math.radians(g('yaw_deg', 0.0)),
            (g('reference_lat', -35.363261), g('reference_lon', 149.165230),
             g('reference_alt', 584.0)),
        )
        nominal = {(cell.row, cell.col): (cell.x, cell.y, cell.z)
                   for cell in self.model.all_cells()}
        self.nominal = nominal
        self.nominal_polygons = {
            (row, col): self._nominal_polygon(row, col)
            for row in range(1, rows + 1)
            for col in range(1, cols + 1)
        }
        nominal_payload = json.dumps({
            'cage_id': self.cage_id, 'rows': rows, 'cols': cols,
            'origin': [self.model.origin_x, self.model.origin_y, self.model.origin_z],
            'cell_size': [self.model.cell_size_x, self.model.cell_size_y],
            'yaw': self.model.yaw,
        }, sort_keys=True, separators=(',', ':'))
        self.geometry_sha256 = hashlib.sha256(nominal_payload.encode('utf-8')).hexdigest()
        self.restore_map_file = str(p('restore_map_file', '')).strip()
        self.save_map_file = str(p('save_map_file', '')).strip()
        configuration = rospy.get_param('/persistent_lattice', {})
        configuration_payload = json.dumps(
            configuration, sort_keys=True, separators=(',', ':'), default=str)
        self.configuration_sha256 = hashlib.sha256(
            configuration_payload.encode('utf-8')).hexdigest()
        mapping_configuration = {
            key: value for key, value in configuration.items()
            if key not in ('restore_map_file', 'save_map_file', 'feeder',
                           'task_map_max_age_sec', 'use_for_tasks')}
        mapping_payload = json.dumps(
            mapping_configuration, sort_keys=True, separators=(',', ':'), default=str)
        self.mapping_config_sha256 = hashlib.sha256(
            mapping_payload.encode('utf-8')).hexdigest()
        reference = tuple(self.model.reference)
        reference_payload = json.dumps({
            'reference_lat': float(reference[0]),
            'reference_lon': float(reference[1]),
            'reference_alt': float(reference[2]),
        }, sort_keys=True, separators=(',', ':'))
        self.reference_sha256 = hashlib.sha256(
            reference_payload.encode('utf-8')).hexdigest()
        with open(__file__, 'rb') as source_stream:
            self.software_source_sha256 = hashlib.sha256(
                source_stream.read()).hexdigest()
        map_metadata = {
            'cage_id': self.cage_id,
            'frame_id': 'map',
            'geometry_sha256': self.geometry_sha256,
            'anchor_definition': 'vertex_0_0',
            'anchor_enu': [self.model.origin_x, self.model.origin_y,
                           self.model.origin_z],
            'configuration_sha256': self.configuration_sha256,
            'mapping_config_sha256': self.mapping_config_sha256,
            'software_source_sha256': self.software_source_sha256,
            'observation_source': str(p('observation_source', 'oracle_noisy')),
            'reference_lat': reference[0],
            'reference_lon': reference[1],
            'reference_alt': reference[2],
            'reference_sha256': self.reference_sha256,
            'nominal_geometry_sha256': self.geometry_sha256,
            'environment_geometry_sha256': str(rospy.get_param(
                '/scout_map_registry/environment_geometry_sha256', '')),
            'environment_id': str(rospy.get_param(
                '/scout_map_registry/environment_id', '')),
            'source_created_at_unix_sec': time.time(),
        }
        self.map_metadata = map_metadata
        if self.restore_map_file:
            self.lattice = PersistentLatticeMap.load_json(
                self.restore_map_file, nominal,
                expected_metadata={
                    'geometry_sha256': self.geometry_sha256,
                    'anchor_definition': 'vertex_0_0',
                    'observation_source': map_metadata['observation_source'],
                },
                nominal_polygons=self.nominal_polygons)
            rospy.logwarn('Explicitly restored persistent map %s version %d from %s',
                          self.lattice.map_uuid, self.lattice.map_version,
                          self.restore_map_file)
        else:
            self.lattice = PersistentLatticeMap(
                rows, cols, nominal,
                min_confirmations=int(p('min_confirmations', 3)),
                cage_id=self.cage_id,
                nominal_polygons=self.nominal_polygons)
        self.associator = IntegerOffsetAssociator(
            rows, cols,
            top_k=int(p('top_k', 3)),
            commit_probability=float(p('commit_probability', 0.65)),
            commit_margin=float(p('commit_margin', 0.15)),
            nominal_sigma_m=float(p('nominal_sigma_m', 3.0)),
            motion_sigma_cells=float(p('motion_sigma_cells', 1.0)),
        )
        self.nominal_sigma_m = float(p('nominal_sigma_m', 3.0))
        optimizer_prefix = '/persistent_lattice/coordinate_optimizer/'
        op = lambda name, default: rospy.get_param(optimizer_prefix + name, default)
        self.optimizer_enabled = bool(op('enabled', True))
        self.optimizer_kwargs = {
            'nominal_sigma_m': float(op('nominal_sigma_m', 4.0)),
            'anchor_sigma_m': float(op('anchor_sigma_m', 0.05)),
            'relative_sigma_m': float(op('relative_sigma_m', 0.35)),
            'huber_delta_m': float(op('huber_delta_m', 1.0)),
            'stale_variance_rate_m2_per_sec': float(
                op('stale_variance_rate_m2_per_sec', 0.0025)),
        }
        self.optimizer = self._new_optimizer()
        self.previous_offset = None
        self.active_task_id = self.lattice.task_id
        # Explicit reset is the task boundary.  Background observations before
        # a Web request must never pre-populate the next mission's result.
        self.session_active = False
        self.offset_pub = rospy.Publisher('/scout/offset_hypotheses',
                                          OffsetHypothesisArray, queue_size=5)
        self.map_pub = rospy.Publisher('/scout/working_lattice_map', PersistentLattice,
                                       queue_size=1, latch=True)
        self.event_pub = rospy.Publisher('/scout/map_events', String, queue_size=10)
        rospy.Subscriber('/scout/local_lattice_observation', LocalLatticeObservation,
                         self.on_observation, queue_size=5)
        self.reset_service = rospy.Service(
            '/scout/reset_map', ResetScoutMap, self.on_reset_map)
        self.finalize_service = rospy.Service(
            '/scout/finalize_map', FinalizeScoutMap, self.on_finalize_map)
        self.publish_map(rospy.Time.now())

    def _nominal_polygon(self, row, col):
        c, s = math.cos(self.model.yaw), math.sin(self.model.yaw)
        result = []
        for local_x, local_y in (
                ((col - 1) * self.model.cell_size_x,
                 (row - 1) * self.model.cell_size_y),
                (col * self.model.cell_size_x,
                 (row - 1) * self.model.cell_size_y),
                (col * self.model.cell_size_x,
                 row * self.model.cell_size_y),
                ((col - 1) * self.model.cell_size_x,
                 row * self.model.cell_size_y)):
            result.append((
                self.model.origin_x + c * local_x - s * local_y,
                self.model.origin_y + s * local_x + c * local_y,
                self.model.origin_z,
            ))
        return tuple(result)

    def _new_optimizer(self):
        return CoordinateOptimizer(self.nominal, **self.optimizer_kwargs)

    @staticmethod
    def _confirmed_count(lattice):
        return sum(state.identity_state == IDENTITY_CONFIRMED
                   for state in lattice.cells.values())

    def publish_event(self, event):
        value = dict(event)
        value.setdefault('time', time.time())
        value.setdefault('task_id', self.active_task_id)
        value.setdefault('cage_id', self.cage_id)
        value.setdefault('map_uuid', self.lattice.map_uuid)
        value.setdefault('map_version', self.lattice.map_version)
        self.event_pub.publish(String(data=json.dumps(value, sort_keys=True)))

    def on_reset_map(self, request):
        task_id = str(request.task_id).strip()
        if not task_id:
            return ResetScoutMapResponse(
                success=False, reason='TASK_ID_REQUIRED', map_uuid='', map_version=0)
        if int(request.cage_id) != self.cage_id:
            return ResetScoutMapResponse(
                success=False, reason='CAGE_ID_NOT_SUPPORTED', map_uuid='', map_version=0)
        with self.lock:
            if self.finalize_in_progress:
                return ResetScoutMapResponse(
                    success=False, reason='FINALIZE_IN_PROGRESS',
                    map_uuid=self.lattice.map_uuid,
                    map_version=self.lattice.map_version)
            if task_id == self.active_task_id:
                reason = ('TASK_ALREADY_ACTIVE' if self.session_active
                          else 'TASK_ALREADY_FINALIZED')
                return ResetScoutMapResponse(
                    success=self.session_active, reason=reason,
                    map_uuid=self.lattice.map_uuid,
                    map_version=self.lattice.map_version)
            if self.session_active:
                return ResetScoutMapResponse(
                    success=False, reason='MAP_SESSION_ACTIVE',
                    map_uuid=self.lattice.map_uuid,
                    map_version=self.lattice.map_version)
            self.lattice = self.lattice.fresh_session(
                task_id=task_id, cage_id=self.cage_id,
                created_at=rospy.Time.now().to_sec())
            self.active_task_id = task_id
            self.session_active = True
            self.map_metadata['source_created_at_unix_sec'] = time.time()
            self.map_metadata.pop('source_completed_at_unix_sec', None)
            self.previous_offset = None
            self.optimizer = self._new_optimizer()
            self.finalize_cache = {}
            event = {
                'type': 'TASK_START',
                'task_id': task_id,
                'cage_id': self.cage_id,
                'map_uuid': self.lattice.map_uuid,
                'map_version': self.lattice.map_version,
            }
            response = ResetScoutMapResponse(
                success=True, reason='', map_uuid=self.lattice.map_uuid,
                map_version=self.lattice.map_version)
        self.publish_event(event)
        self.publish_map(rospy.Time.now())
        return response

    @staticmethod
    def _finalize_response(value):
        return FinalizeScoutMapResponse(
            success=bool(value['success']),
            reason=str(value['reason']),
            map_uuid=str(value['map_uuid']),
            map_version=int(value['map_version']),
            confirmed_cells=int(value['confirmed_cells']),
            expected_cells=int(value['expected_cells']),
            coverage=float(value['coverage']),
            snapshot_path=str(value['snapshot_path']),
        )

    def on_finalize_map(self, request):
        task_id = str(request.task_id).strip()
        output_directory = os.path.abspath(os.path.expanduser(
            str(request.output_directory).strip())) if request.output_directory else ''
        min_coverage = float(request.min_coverage)
        if not task_id:
            return self._finalize_response({
                'success': False, 'reason': 'TASK_ID_REQUIRED', 'map_uuid': '',
                'map_version': 0, 'confirmed_cells': 0,
                'expected_cells': self.lattice.rows * self.lattice.cols,
                'coverage': 0.0, 'snapshot_path': '',
            })
        if int(request.cage_id) != self.cage_id:
            return self._finalize_response({
                'success': False, 'reason': 'CAGE_ID_NOT_SUPPORTED', 'map_uuid': '',
                'map_version': 0, 'confirmed_cells': 0,
                'expected_cells': self.lattice.rows * self.lattice.cols,
                'coverage': 0.0, 'snapshot_path': '',
            })
        if not output_directory:
            return self._finalize_response({
                'success': False, 'reason': 'OUTPUT_DIRECTORY_REQUIRED', 'map_uuid': '',
                'map_version': 0, 'confirmed_cells': 0,
                'expected_cells': self.lattice.rows * self.lattice.cols,
                'coverage': 0.0, 'snapshot_path': '',
            })
        if not 0.0 <= min_coverage <= 1.0:
            return self._finalize_response({
                'success': False, 'reason': 'INVALID_MIN_COVERAGE', 'map_uuid': '',
                'map_version': 0, 'confirmed_cells': 0,
                'expected_cells': self.lattice.rows * self.lattice.cols,
                'coverage': 0.0, 'snapshot_path': '',
            })

        with self.lock:
            cached = self.finalize_cache.get(task_id)
            if cached is not None:
                return self._finalize_response(cached)
            if task_id != self.active_task_id:
                return self._finalize_response({
                    'success': False, 'reason': 'TASK_ID_MISMATCH',
                    'map_uuid': self.lattice.map_uuid,
                    'map_version': self.lattice.map_version,
                    'confirmed_cells': self._confirmed_count(self.lattice),
                    'expected_cells': self.lattice.rows * self.lattice.cols,
                    'coverage': 0.0, 'snapshot_path': '',
                })
            if self.finalize_in_progress:
                return self._finalize_response({
                    'success': False, 'reason': 'FINALIZE_IN_PROGRESS',
                    'map_uuid': self.lattice.map_uuid,
                    'map_version': self.lattice.map_version,
                    'confirmed_cells': self._confirmed_count(self.lattice),
                    'expected_cells': self.lattice.rows * self.lattice.cols,
                    'coverage': 0.0, 'snapshot_path': '',
                })
            if not self.session_active:
                return self._finalize_response({
                    'success': False, 'reason': 'NO_ACTIVE_MAP_SESSION',
                    'map_uuid': self.lattice.map_uuid,
                    'map_version': self.lattice.map_version,
                    'confirmed_cells': self._confirmed_count(self.lattice),
                    'expected_cells': self.lattice.rows * self.lattice.cols,
                    'coverage': 0.0, 'snapshot_path': '',
                })
            self.finalize_in_progress = True
            self.session_active = False
            confirmed = self._confirmed_count(self.lattice)
            expected = self.lattice.rows * self.lattice.cols
            coverage = float(confirmed) / expected if expected else 0.0
            final_metadata = dict(self.map_metadata)
            final_metadata['source_completed_at_unix_sec'] = time.time()
            self.map_metadata = final_metadata
            payload = self.lattice.finalize_snapshot(
                final_metadata, timestamp=rospy.Time.now().to_sec())
            snapshot_path = os.path.join(output_directory, 'map.json')
            map_uuid = self.lattice.map_uuid
            map_version = self.lattice.map_version

        write_error = ''
        try:
            PersistentLatticeMap.save_json_payload(snapshot_path, payload)
        except (OSError, TypeError, ValueError) as error:
            write_error = '{}: {}'.format(type(error).__name__, error)

        if write_error:
            success = False
            reason = 'SNAPSHOT_WRITE_FAILED: ' + write_error
            saved_path = ''
        elif coverage < min_coverage:
            success = False
            reason = 'COVERAGE_BELOW_MIN: {:.6f} < {:.6f}'.format(
                coverage, min_coverage)
            saved_path = snapshot_path
        else:
            success = True
            reason = ''
            saved_path = snapshot_path
        result = {
            'success': success,
            'reason': reason,
            'map_uuid': map_uuid,
            'map_version': map_version,
            'confirmed_cells': confirmed,
            'expected_cells': expected,
            'coverage': coverage,
            'snapshot_path': saved_path,
        }
        with self.lock:
            self.finalize_in_progress = False
            self.finalize_cache[task_id] = dict(result)
        self.publish_event({
            'type': 'TASK_END',
            'task_id': task_id,
            'success': success,
            'reason': reason,
            'frozen': True,
            'confirmed_cells': confirmed,
            'expected_cells': expected,
            'coverage': coverage,
            'snapshot_path': saved_path,
        })
        self.publish_map(rospy.Time.now())
        return self._finalize_response(result)

    @staticmethod
    def observation_dict(message):
        return {
            'frame_seq': int(message.frame_seq),
            'local_rows': int(message.local_rows),
            'local_cols': int(message.local_cols),
            'cells': [{
                'local_i': int(cell.local_i),
                'local_j': int(cell.local_j),
                'projected_center': (cell.projected_center.x,
                                     cell.projected_center.y,
                                     cell.projected_center.z),
                'projected_covariance_xy': tuple(cell.projected_covariance_xy),
                'confidence': float(cell.confidence),
            } for cell in message.cells],
        }

    def publish_decision(self, header, frame_seq, decision):
        message = OffsetHypothesisArray(
            header=header,
            frame_seq=frame_seq,
            committed=decision.committed,
            committed_delta_row=decision.delta_row,
            committed_delta_col=decision.delta_col,
            decision_reason=decision.reason,
        )
        for value in decision.hypotheses:
            message.hypotheses.append(OffsetHypothesis(
                delta_row=value.delta_row, delta_col=value.delta_col,
                posterior=value.posterior,
                geometry_likelihood=value.geometry_likelihood,
                topology_likelihood=value.topology_likelihood,
                motion_likelihood=value.motion_likelihood,
                valid=value.valid, reject_reason=value.reject_reason))
        self.offset_pub.publish(message)

    def on_observation(self, message):
        if message.cage_id != self.cage_id:
            return
        with self.lock:
            if not self.session_active or self.lattice.frozen:
                return
            observation = self.observation_dict(message)
            decision = self.associator.associate(
                observation['cells'],
                self.lattice.candidate_cells(self.nominal_sigma_m),
                self.previous_offset)
            self.publish_decision(message.header, message.frame_seq, decision)
            if decision.committed:
                self.previous_offset = (decision.delta_row, decision.delta_col)
            events = self.lattice.apply_observation(
                observation, decision, message.header.stamp.to_sec())
            if decision.committed and self.optimizer_enabled:
                optimized_inputs = {}
                for source in observation['cells']:
                    index = (int(source['local_i']) + int(decision.delta_row),
                             int(source['local_j']) + int(decision.delta_col))
                    state = self.lattice.cells.get(index)
                    if state is None or state.identity_state != IDENTITY_CONFIRMED:
                        continue
                    optimized_inputs[index] = {
                        'position': source['projected_center'],
                        'covariance_xy': source['projected_covariance_xy'],
                        'confidence': source['confidence'],
                    }
                if optimized_inputs:
                    timestamp = message.header.stamp.to_sec()
                    self.optimizer.add_observation(optimized_inputs, timestamp)
                    confirmed = [index for index, state in self.lattice.cells.items()
                                 if state.identity_state == IDENTITY_CONFIRMED]
                    optimized, stats = self.optimizer.optimize(confirmed, now=timestamp)
                    for index, value in optimized.items():
                        self.lattice.cells[index].estimated_position = value['position']
                        self.lattice.cells[index].covariance_xy = value['covariance_xy']
                    events.append({
                        'type': 'COORDINATE_OPTIMIZATION',
                        'time': timestamp,
                        'map_version': self.lattice.map_version,
                        'factor_count': stats['factor_count'],
                        'outlier_count': stats['outlier_count'],
                        'rmse': stats['rmse'],
                    })
            for event in events:
                event.update(
                    frame_seq=int(message.frame_seq),
                    task_id=self.active_task_id,
                    cage_id=self.cage_id,
                    map_uuid=self.lattice.map_uuid)
                self.event_pub.publish(String(data=json.dumps(event, sort_keys=True)))
            if self.save_map_file:
                self.lattice.save_json(self.save_map_file, self.map_metadata)
        self.publish_map(message.header.stamp)

    def publish_map(self, stamp):
        with self.lock:
            message = PersistentLattice(
                header=Header(stamp=stamp, frame_id='map'),
                cage_id=self.cage_id,
                map_version=self.lattice.map_version,
                map_uuid=self.lattice.map_uuid,
                anchor_definition='vertex_0_0',
                anchor_enu=Point(x=self.model.origin_x, y=self.model.origin_y,
                                 z=self.model.origin_z),
                geometry_sha256=self.geometry_sha256,
                source_task_id=self.active_task_id,
                frozen=bool(self.lattice.frozen),
                source_created_at_ros=rospy.Time.from_sec(self.lattice.created_at),
                source_finalized_at_ros=rospy.Time.from_sec(
                    float(self.lattice.finalized_at or 0.0)),
                activated_at_ros=rospy.Time(0),
                source_created_at_unix_sec=float(self.map_metadata.get(
                    'source_created_at_unix_sec', 0.0) or 0.0),
                source_completed_at_unix_sec=float(self.map_metadata.get(
                    'source_completed_at_unix_sec', 0.0) or 0.0),
                activated_at_unix_sec=0.0,
                map_sha256='', manifest_sha256='',
                mapping_config_sha256=self.mapping_config_sha256,
                software_source_sha256=self.software_source_sha256,
                observation_source=self.map_metadata['observation_source'],
                reference_lat=self.model.reference[0],
                reference_lon=self.model.reference[1],
                reference_alt=self.model.reference[2],
                reference_sha256=self.reference_sha256,
                environment_geometry_sha256=self.map_metadata[
                    'environment_geometry_sha256'],
                confirmed_cells=self._confirmed_count(self.lattice),
                expected_cells=self.lattice.rows * self.lattice.cols,
                coverage=(float(self._confirmed_count(self.lattice)) /
                          (self.lattice.rows * self.lattice.cols)),
                restored_from_history=False,
                legacy_reference_assumed=False,
                legacy_environment_assumed=False,
            )
            for index, state in sorted(self.lattice.cells.items()):
                message.cells.append(LatticeCell(
                    row=state.row, col=state.col, cell_id=state.cell_id,
                    polygon=[point(value) for value in
                             self.lattice._cell_polygon(index, state)],
                    nominal_position=point(state.nominal_position),
                    estimated_position=point(state.estimated_position),
                    covariance_xy=list(state.covariance_xy),
                    identity_state=state.identity_state,
                    visibility_state=state.visibility_state,
                    identity_confidence=state.identity_confidence,
                    first_seen=rospy.Time.from_sec(state.first_seen),
                    last_seen=rospy.Time.from_sec(state.last_seen),
                    observation_count=state.observation_count,
                    exists=True,
                ))
            present = set(self.lattice.cells)
            for row, col in sorted(present):
                for target, relation in (
                        ((row + 1, col), LatticeEdge.RELATION_ROW),
                        ((row, col + 1), LatticeEdge.RELATION_COLUMN)):
                    if target in present:
                        message.edges.append(LatticeEdge(
                            from_cell_id=self.lattice.cell_id(row, col),
                            to_cell_id=self.lattice.cell_id(*target),
                            relation=relation))
        self.map_pub.publish(message)


if __name__ == '__main__':
    rospy.init_node('scout_lattice_node')
    ScoutLatticeNode()
    rospy.spin()
