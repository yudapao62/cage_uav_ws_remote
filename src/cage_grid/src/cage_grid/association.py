"""Probabilistic integer-offset association for local and persistent lattices."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class OffsetHypothesisValue:
    delta_row: int
    delta_col: int
    posterior: float
    geometry_likelihood: float
    topology_likelihood: float
    motion_likelihood: float
    valid: bool = True
    reject_reason: str = ''


@dataclass(frozen=True)
class AssociationDecision:
    committed: bool
    delta_row: int
    delta_col: int
    reason: str
    hypotheses: tuple


def _inverse_2x2(matrix):
    a, b, c, d = map(float, matrix)
    determinant = a * d - b * c
    if determinant <= 1e-12:
        return None
    scale = 1.0 / determinant
    return d * scale, -b * scale, -c * scale, a * scale


def _gaussian_log_likelihood(dx, dy, covariance):
    inverse = _inverse_2x2(covariance)
    if inverse is None:
        return -math.inf
    ia, ib, ic, id_ = inverse
    quadratic = dx * (ia * dx + ib * dy) + dy * (ic * dx + id_ * dy)
    determinant = covariance[0] * covariance[3] - covariance[1] * covariance[2]
    return -0.5 * quadratic - 0.5 * math.log(max(determinant, 1e-12))


class IntegerOffsetAssociator:
    def __init__(self, rows, cols, top_k=3, commit_probability=0.65,
                 commit_margin=0.15, nominal_sigma_m=3.0, motion_sigma_cells=1.0):
        self.rows = int(rows)
        self.cols = int(cols)
        self.top_k = int(top_k)
        self.commit_probability = float(commit_probability)
        self.commit_margin = float(commit_margin)
        self.nominal_sigma_m = float(nominal_sigma_m)
        self.motion_sigma_cells = float(motion_sigma_cells)
        if self.rows < 1 or self.cols < 1 or self.top_k < 1:
            raise ValueError('rows, cols and top_k must be positive')
        if not 0.0 <= self.commit_probability <= 1.0:
            raise ValueError('commit probability must be in [0,1]')
        if not 0.0 <= self.commit_margin <= 1.0:
            raise ValueError('commit margin must be in [0,1]')
        if self.nominal_sigma_m <= 0.0 or self.motion_sigma_cells <= 0.0:
            raise ValueError('association sigmas must be positive')

    def _candidate_offsets(self, observation_cells):
        if not observation_cells:
            return []
        min_i = min(int(cell['local_i']) for cell in observation_cells)
        max_i = max(int(cell['local_i']) for cell in observation_cells)
        min_j = min(int(cell['local_j']) for cell in observation_cells)
        max_j = max(int(cell['local_j']) for cell in observation_cells)
        delta_rows = range(1 - min_i, self.rows - max_i + 1)
        delta_cols = range(1 - min_j, self.cols - max_j + 1)
        return [(delta_row, delta_col)
                for delta_row in delta_rows for delta_col in delta_cols]

    def associate(self, observation_cells, candidate_cells, previous_offset=None):
        """Return Top-K offset posteriors.

        ``candidate_cells`` is keyed by ``(row,col)`` and each value provides
        ``position`` and optional ``covariance_xy``.  Positions may be nominal
        for unseen cells or estimated for established cells.
        """
        candidates = self._candidate_offsets(observation_cells)
        if not candidates:
            return AssociationDecision(False, 0, 0, 'NO_OBSERVATIONS', tuple())
        scored = []
        for delta_row, delta_col in candidates:
            geometry_log = 0.0
            topology_log = 0.0
            valid = True
            for observation in observation_cells:
                row = int(observation['local_i']) + delta_row
                col = int(observation['local_j']) + delta_col
                target = candidate_cells.get((row, col))
                if target is None:
                    valid = False
                    break
                observed = observation.get('projected_center')
                if observed is not None:
                    expected = target['position']
                    observation_covariance = observation.get(
                        'projected_covariance_xy', [0.0, 0.0, 0.0, 0.0])
                    target_covariance = target.get(
                        'covariance_xy', [self.nominal_sigma_m ** 2, 0.0,
                                          0.0, self.nominal_sigma_m ** 2])
                    covariance = [
                        float(observation_covariance[0]) + float(target_covariance[0]) + 1e-6,
                        float(observation_covariance[1]) + float(target_covariance[1]),
                        float(observation_covariance[2]) + float(target_covariance[2]),
                        float(observation_covariance[3]) + float(target_covariance[3]) + 1e-6,
                    ]
                    log_likelihood = _gaussian_log_likelihood(
                        float(observed[0]) - float(expected[0]),
                        float(observed[1]) - float(expected[1]), covariance)
                    confidence = max(0.05, min(1.0, float(observation.get('confidence', 1.0))))
                    geometry_log += confidence * log_likelihood
                # Missing local indices are legal.  Reward only explicit local
                # four-neighbor relationships instead of penalizing gaps.
                topology_log += math.log(max(0.05, float(observation.get('confidence', 1.0))))
            if not valid:
                continue
            if previous_offset is None:
                motion_log = 0.0
            else:
                distance2 = ((delta_row - int(previous_offset[0])) ** 2
                             + (delta_col - int(previous_offset[1])) ** 2)
                motion_log = -0.5 * distance2 / (self.motion_sigma_cells ** 2)
            scored.append((delta_row, delta_col, geometry_log, topology_log, motion_log,
                           geometry_log + topology_log + motion_log))
        if not scored:
            return AssociationDecision(False, 0, 0, 'NO_VALID_OFFSET', tuple())
        maximum = max(item[-1] for item in scored)
        weights = [math.exp(max(-700.0, item[-1] - maximum)) for item in scored]
        total = sum(weights)
        hypotheses = []
        for item, weight in zip(scored, weights):
            delta_row, delta_col, geometry_log, topology_log, motion_log, _ = item
            hypotheses.append(OffsetHypothesisValue(
                delta_row=delta_row,
                delta_col=delta_col,
                posterior=weight / total,
                geometry_likelihood=math.exp(max(-700.0, geometry_log)),
                topology_likelihood=math.exp(max(-700.0, topology_log)),
                motion_likelihood=math.exp(max(-700.0, motion_log)),
            ))
        hypotheses.sort(key=lambda value: value.posterior, reverse=True)
        hypotheses = hypotheses[:self.top_k]
        top = hypotheses[0]
        second = hypotheses[1].posterior if len(hypotheses) > 1 else 0.0
        if top.posterior < self.commit_probability:
            reason = 'TOP1_BELOW_THRESHOLD'
            committed = False
        elif top.posterior - second < self.commit_margin:
            reason = 'TOP1_TOP2_MARGIN_TOO_SMALL'
            committed = False
        else:
            reason = 'COMMITTED'
            committed = True
        return AssociationDecision(
            committed, top.delta_row if committed else 0,
            top.delta_col if committed else 0, reason, tuple(hypotheses))

