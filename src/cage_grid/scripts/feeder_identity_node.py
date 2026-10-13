#!/usr/bin/env python3
"""Recover Feeder cell identity from Scout map, GPS and local geometry."""
import math
import time

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

from cage_geo.geodesy import LocalENU
from cage_grid.feeder_filter import (
    FeederIdentityFilter,
    STATUS_AMBIGUOUS,
    STATUS_CONFIRMED,
    STATUS_LOST,
    STATUS_STALE_MAP,
    neighborhood_visual_likelihoods,
)
from cage_msgs.msg import (
    CellTargetArray,
    FeederCellHypothesis,
    FeederCellHypothesisArray,
    FeederIdentityState,
    LocalLatticeObservation,
    MissionState,
    PersistentLattice,
    ScoutMapActivationStatus,
)


class FeederIdentityNode:
    STATUS_TO_MESSAGE = {
        STATUS_CONFIRMED: FeederIdentityState.STATUS_CONFIRMED,
        STATUS_AMBIGUOUS: FeederIdentityState.STATUS_AMBIGUOUS,
        STATUS_LOST: FeederIdentityState.STATUS_LOST,
        STATUS_STALE_MAP: FeederIdentityState.STATUS_STALE_MAP,
    }

    def __init__(self):
        config = rospy.get_param('/persistent_lattice/feeder', {})
        self.identity_filter = FeederIdentityFilter(
            chi_square_gate_2d=config.get('chi_square_gate_2d', 9.21034),
            max_candidates=config.get('max_candidates', 9),
            commit_probability=config.get('commit_probability', 0.65),
            commit_margin=config.get('commit_margin', 0.15),
            max_entropy=config.get('max_entropy', 0.90),
            max_source_age_sec=config.get('max_source_age_sec', 0.0),
            staleness_variance_rate_m2_per_sec=config.get(
                'source_age_variance_rate_m2_per_sec',
                config.get('staleness_variance_rate_m2_per_sec', 0.0)),
            continuous_next_probability=config.get(
                'continuous_next_probability', 0.80),
            continuous_stay_probability=config.get(
                'continuous_stay_probability', 0.15),
        )
        self.visual_sigma_m = float(config.get('visual_sigma_m', 1.0))
        self.default_gps_variance = float(config.get('default_gps_variance_m2', 4.0))
        self.max_visual_age_sec = float(config.get('max_visual_age_sec', 2.0))
        self.require_visual_evidence = bool(config.get('require_visual_evidence', True))
        self.local_frame = None
        self.map_msg = None
        self.map_cells = []
        self.latest_observation = None
        self.route = []
        self.task_map_version = 0
        self.task_map_uuid = ''
        self.task_map_sha256 = ''
        self.task_manifest_sha256 = ''
        self.active_status = None
        self.last_active_status_monotonic = 0.0
        self.active_status_timeout = float(rospy.get_param(
            '/scout_map_registry/active_status_timeout_sec', 3.0))
        self.route_sequence = None
        self.reentry = True

        self.state_pub = rospy.Publisher(
            '/feeder/identity_state', FeederIdentityState, queue_size=1, latch=True)
        self.hypotheses_pub = rospy.Publisher(
            '/feeder/cell_hypotheses', FeederCellHypothesisArray,
            queue_size=1, latch=True)
        rospy.Subscriber('/scout/lattice_map', PersistentLattice,
                         self.on_map, queue_size=1)
        rospy.Subscriber('/scout/active_map_status', ScoutMapActivationStatus,
                         self.on_active_status, queue_size=1)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix,
                         self.on_gps, queue_size=1)
        rospy.Subscriber('/feeder/local_lattice_observation',
                         LocalLatticeObservation, self.on_observation, queue_size=1)
        rospy.Subscriber('/feeding/target_cells', CellTargetArray,
                         self.on_targets, queue_size=1)
        rospy.Subscriber('/mission/state', MissionState,
                         self.on_mission_state, queue_size=1)

    def on_map(self, msg):
        if (not msg.frozen or not msg.map_uuid or not msg.map_sha256 or
                not msg.manifest_sha256 or
                not all(math.isfinite(value) for value in
                        (msg.reference_lat,msg.reference_lon,msg.reference_alt))):
            rospy.logwarn('Ignoring invalid active Scout map')
            return
        self.map_msg = msg
        self.local_frame = LocalENU(
            msg.reference_lat,msg.reference_lon,msg.reference_alt)
        cells = []
        for cell in msg.cells:
            if cell.identity_state != cell.IDENTITY_CONFIRMED or not cell.exists:
                continue
            cells.append({
                'row': int(cell.row),
                'col': int(cell.col),
                'cell_id': cell.cell_id,
                'position': (cell.estimated_position.x, cell.estimated_position.y),
                'covariance_xy': tuple(cell.covariance_xy),
                'confirmed': True,
            })
        self.map_cells = cells

    def on_active_status(self,msg):
        self.active_status=msg
        self.last_active_status_monotonic=time.monotonic()

    def on_observation(self, msg):
        self.latest_observation = msg

    def on_targets(self, msg):
        sequence = int(msg.header.seq)
        route = [target.cell_id for target in msg.targets]
        if sequence != self.route_sequence or route != self.route:
            self.route_sequence = sequence
            self.route = route
            self.task_map_uuid = str(msg.map_uuid)
            self.task_map_version = int(msg.map_version)
            self.task_map_sha256 = str(msg.map_sha256)
            self.task_manifest_sha256 = str(msg.manifest_sha256)
            self.reentry = True

    def on_mission_state(self, msg):
        state = str(msg.state).upper()
        if (msg.manual_override or state in (
                'MANUAL_OVERRIDE', 'RTL', 'LAND', 'TIMEOUT_ABORT',
                'RECOVERY_RTL', 'RECOVERY_RTL_VERIFY', 'RECOVERY_RETURNING',
                'RECOVERY_LANDING', 'ERROR')):
            self.reentry = True

    @staticmethod
    def _valid_gps(msg):
        return (msg.status.status >= NavSatStatus.STATUS_FIX
                and all(math.isfinite(value) for value in (
                    msg.latitude, msg.longitude, msg.altitude))
                and (msg.latitude != 0.0 or msg.longitude != 0.0))

    def _gps_covariance(self, msg):
        covariance = msg.position_covariance
        values = (covariance[0], covariance[1], covariance[3], covariance[4])
        if (msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN
                or not all(math.isfinite(value) for value in values)
                or values[0] <= 0.0 or values[3] <= 0.0):
            return (self.default_gps_variance, 0.0,
                    0.0, self.default_gps_variance)
        return values

    def _visual_likelihoods(self, stamp):
        observation = self.latest_observation
        if observation is None:
            return {}
        observation_stamp = observation.header.stamp
        if (observation_stamp.is_zero()
                or abs((stamp - observation_stamp).to_sec()) > self.max_visual_age_sec):
            return {}
        cells = [{
            'local_i': int(cell.local_i),
            'local_j': int(cell.local_j),
            'position': (cell.projected_center.x, cell.projected_center.y),
        } for cell in observation.cells if cell.confidence > 0.0]
        return neighborhood_visual_likelihoods(
            cells, self.map_cells,
            [cell['cell_id'] for cell in self.map_cells],
            sigma_m=self.visual_sigma_m)

    @staticmethod
    def _hypothesis_message(value):
        return FeederCellHypothesis(
            row=value.row,
            col=value.col,
            cell_id=value.cell_id,
            gps_likelihood=value.gps_likelihood,
            visual_likelihood=value.visual_likelihood,
            transition_prior=value.transition_prior,
            posterior=value.posterior,
            mahalanobis_d2=value.mahalanobis_d2,
            map_age_sec=value.map_age_sec,
            inside_gate=value.inside_gate,
            gate_reason=value.gate_reason)

    def on_gps(self, msg):
        if not self._valid_gps(msg):
            return
        now = rospy.Time.now()
        if self.map_msg is None:
            self._publish_empty(now, FeederIdentityState.STATUS_LOST, 'NO_SCOUT_MAP')
            return
        status=self.active_status
        if (status is None or
                time.monotonic()-self.last_active_status_monotonic > self.active_status_timeout):
            self._publish_empty(now,FeederIdentityState.STATUS_STALE_MAP,
                                'REGISTRY_HEARTBEAT_STALE'); return
        if status.status != ScoutMapActivationStatus.STATUS_ACTIVE:
            self._publish_empty(now,FeederIdentityState.STATUS_STALE_MAP,
                                status.reason_code or 'NO_ACTIVE_MAP'); return
        active_pin=(self.map_msg.map_uuid,int(self.map_msg.map_version),
                    self.map_msg.map_sha256,self.map_msg.manifest_sha256)
        status_pin=(status.map_uuid,int(status.map_version),status.map_sha256,
                    status.manifest_sha256)
        task_pin=(self.task_map_uuid,int(self.task_map_version),
                  self.task_map_sha256,self.task_manifest_sha256)
        if active_pin != status_pin or (self.task_map_uuid and active_pin != task_pin):
            self._publish_empty(now,FeederIdentityState.STATUS_MAP_MISMATCH,
                                'TASK_MAP_MISMATCH'); return
        source_completed=float(self.map_msg.source_completed_at_unix_sec)
        wall_now=time.time()
        if source_completed <= 0.0 or source_completed > wall_now + 300.0:
            self._publish_empty(now,FeederIdentityState.STATUS_STALE_MAP,
                                'SOURCE_MAP_CLOCK_INVALID'); return
        map_age=max(0.0,wall_now-source_completed)
        east, north, _ = self.local_frame.to_enu(
            msg.latitude, msg.longitude, msg.altitude)
        visual_likelihoods = self._visual_likelihoods(
            msg.header.stamp if not msg.header.stamp.is_zero() else now)
        decision = self.identity_filter.update(
            self.map_cells,
            (east, north),
            self._gps_covariance(msg),
            visual_likelihoods=visual_likelihoods,
            route=self.route,
            reentry=self.reentry,
            map_age_sec=map_age,
            allow_commit=(bool(visual_likelihoods)
                          or not self.require_visual_evidence))
        hypotheses = [self._hypothesis_message(value)
                      for value in decision.hypotheses]
        state = FeederIdentityState(
            header=msg.header,
            status=self.STATUS_TO_MESSAGE[decision.status],
            map_uuid=self.map_msg.map_uuid,
            map_version=self.map_msg.map_version,
            map_sha256=self.map_msg.map_sha256,
            manifest_sha256=self.map_msg.manifest_sha256,
            task_map_uuid=self.task_map_uuid,
            task_map_version=self.task_map_version,
            task_map_sha256=self.task_map_sha256,
            task_manifest_sha256=self.task_manifest_sha256,
            previous_cell_id=decision.previous_cell_id,
            current_cell_id=decision.current_cell_id,
            next_cell_id=decision.next_cell_id,
            top1_cell_id=decision.top1_cell_id,
            top2_cell_id=decision.top2_cell_id,
            top1_posterior=decision.top1_posterior,
            top2_posterior=decision.top2_posterior,
            entropy=decision.entropy,
            decision_reason=decision.reason,
            hypotheses=hypotheses)
        self.state_pub.publish(state)
        self.hypotheses_pub.publish(FeederCellHypothesisArray(
            header=msg.header,
            map_uuid=self.map_msg.map_uuid,
            map_version=self.map_msg.map_version,
            map_sha256=self.map_msg.map_sha256,
            manifest_sha256=self.map_msg.manifest_sha256,
            hypotheses=hypotheses))
        if decision.status == STATUS_CONFIRMED:
            self.reentry = False

    def _publish_empty(self, stamp, status, reason):
        header = Header(stamp=stamp)
        self.state_pub.publish(FeederIdentityState(
            header=header,
            status=status,
            map_uuid=self.map_msg.map_uuid if self.map_msg else '',
            map_version=self.map_msg.map_version if self.map_msg else 0,
            map_sha256=self.map_msg.map_sha256 if self.map_msg else '',
            manifest_sha256=self.map_msg.manifest_sha256 if self.map_msg else '',
            task_map_uuid=self.task_map_uuid,
            task_map_version=self.task_map_version,
            task_map_sha256=self.task_map_sha256,
            task_manifest_sha256=self.task_manifest_sha256,
            decision_reason=reason))
        self.hypotheses_pub.publish(FeederCellHypothesisArray(
            header=header,
            map_uuid=self.map_msg.map_uuid if self.map_msg else '',
            map_version=self.map_msg.map_version if self.map_msg else 0,
            map_sha256=self.map_msg.map_sha256 if self.map_msg else '',
            manifest_sha256=self.map_msg.manifest_sha256 if self.map_msg else ''))


if __name__ == '__main__':
    rospy.init_node('feeder_identity_node')
    FeederIdentityNode()
    rospy.spin()
