"""GPS-gated adjacent-cell identity recovery with explicit abstention."""
import math
from dataclasses import dataclass


STATUS_CONFIRMED = 'CONFIRMED'
STATUS_AMBIGUOUS = 'AMBIGUOUS'
STATUS_LOST = 'LOST'
STATUS_STALE_MAP = 'STALE_MAP'


def _inverse_2x2(matrix):
    a, b, c, d = map(float, matrix)
    determinant = a * d - b * c
    if determinant <= 1e-12:
        return None, determinant
    scale = 1.0 / determinant
    return (d * scale, -b * scale, -c * scale, a * scale), determinant


def mahalanobis_and_likelihood(delta, covariance):
    inverse, determinant = _inverse_2x2(covariance)
    if inverse is None:
        return math.inf, 0.0
    dx, dy = map(float, delta)
    ia, ib, ic, id_ = inverse
    distance2 = dx * (ia * dx + ib * dy) + dy * (ic * dx + id_ * dy)
    likelihood = math.exp(-0.5 * max(0.0, distance2)) / (
        2.0 * math.pi * math.sqrt(max(determinant, 1e-12)))
    return distance2, likelihood


@dataclass(frozen=True)
class FeederHypothesisValue:
    row: int
    col: int
    cell_id: str
    gps_likelihood: float
    visual_likelihood: float
    transition_prior: float
    posterior: float
    mahalanobis_d2: float
    map_age_sec: float
    inside_gate: bool = True
    gate_reason: str = 'INSIDE_CHI_SQUARE_GATE'


@dataclass(frozen=True)
class FeederDecision:
    status: str
    previous_cell_id: str
    current_cell_id: str
    next_cell_id: str
    top1_cell_id: str
    top2_cell_id: str
    top1_posterior: float
    top2_posterior: float
    entropy: float
    reason: str
    hypotheses: tuple


class FeederIdentityFilter:
    def __init__(self, chi_square_gate_2d=9.21034, max_candidates=9,
                 commit_probability=0.65, commit_margin=0.15, max_entropy=0.90,
                 stale_map_sec=0.0, staleness_variance_rate_m2_per_sec=0.01,
                 max_source_age_sec=None,
                 continuous_next_probability=0.80,
                 continuous_stay_probability=0.15):
        self.chi_square_gate = float(chi_square_gate_2d)
        self.max_candidates = int(max_candidates)
        self.commit_probability = float(commit_probability)
        self.commit_margin = float(commit_margin)
        self.max_entropy = float(max_entropy)
        self.max_source_age_sec = float(
            stale_map_sec if max_source_age_sec is None else max_source_age_sec)
        self.staleness_variance_rate = float(staleness_variance_rate_m2_per_sec)
        self.next_probability = float(continuous_next_probability)
        self.stay_probability = float(continuous_stay_probability)
        if self.chi_square_gate <= 0.0 or self.max_candidates < 1:
            raise ValueError('invalid Feeder gate or candidate limit')
        probabilities = (self.commit_probability, self.commit_margin,
                         self.max_entropy, self.next_probability,
                         self.stay_probability)
        if not all(0.0 <= value <= 1.0 for value in probabilities):
            raise ValueError('Feeder probability thresholds must be in [0,1]')
        if self.next_probability + self.stay_probability > 1.0:
            raise ValueError('next and stay probabilities must sum to <= 1')
        if self.max_source_age_sec < 0.0 or self.staleness_variance_rate < 0.0:
            raise ValueError('invalid map staleness configuration')
        self.current_cell_id = ''

    @staticmethod
    def _route_next(route, current):
        if not route or current not in route:
            return ''
        index = route.index(current)
        return route[index + 1] if index + 1 < len(route) else ''

    def _transition_prior(self, cell_id, candidates, route, reentry):
        if reentry or not self.current_cell_id:
            return 1.0 / max(1, len(candidates))
        next_cell = self._route_next(route, self.current_cell_id)
        remainder = max(0.0, 1.0 - self.next_probability - self.stay_probability)
        other_count = max(1, len(candidates) - int(bool(next_cell)) - 1)
        if cell_id == next_cell:
            return self.next_probability
        if cell_id == self.current_cell_id:
            return self.stay_probability
        return remainder / other_count

    def update(self, map_cells, gps_xy, gps_covariance_xy, visual_likelihoods=None,
               route=None, reentry=False, map_age_sec=0.0, allow_commit=True):
        visual_likelihoods = dict(visual_likelihoods or {})
        route = list(route or [])
        map_age_sec = max(0.0, float(map_age_sec))
        previous = self.current_cell_id
        if (self.max_source_age_sec > 0.0 and
                map_age_sec > self.max_source_age_sec):
            return FeederDecision(
                STATUS_STALE_MAP, previous, '', '', '', '', 0.0, 0.0, 1.0,
                'MAP_AGE_EXCEEDS_LIMIT', tuple())
        candidates = []
        stale_variance = map_age_sec * self.staleness_variance_rate
        for cell in map_cells:
            if not cell.get('confirmed', True):
                continue
            position = cell['position']
            map_covariance = cell.get('covariance_xy', (0.0, 0.0, 0.0, 0.0))
            covariance = (
                float(gps_covariance_xy[0]) + float(map_covariance[0]) + stale_variance,
                float(gps_covariance_xy[1]) + float(map_covariance[1]),
                float(gps_covariance_xy[2]) + float(map_covariance[2]),
                float(gps_covariance_xy[3]) + float(map_covariance[3]) + stale_variance,
            )
            distance2, gps_likelihood = mahalanobis_and_likelihood(
                (float(gps_xy[0]) - float(position[0]),
                 float(gps_xy[1]) - float(position[1])), covariance)
            if distance2 <= self.chi_square_gate:
                candidates.append((distance2, gps_likelihood, cell))
        candidates.sort(key=lambda value: value[0])
        candidates = candidates[:self.max_candidates]
        if not candidates:
            return FeederDecision(
                STATUS_LOST, previous, '', '', '', '', 0.0, 0.0, 1.0,
                'NO_CELL_INSIDE_GPS_GATE', tuple())

        raw = []
        candidate_ids = [value[2]['cell_id'] for value in candidates]
        for distance2, gps_likelihood, cell in candidates:
            cell_id = cell['cell_id']
            visual = max(1e-12, float(visual_likelihoods.get(cell_id, 1.0)))
            transition = max(1e-12, self._transition_prior(
                cell_id, candidate_ids, route, bool(reentry)))
            raw.append((cell, distance2, gps_likelihood, visual, transition,
                        gps_likelihood * visual * transition))
        total = sum(value[-1] for value in raw)
        if total <= 0.0 or not math.isfinite(total):
            return FeederDecision(
                STATUS_LOST, previous, '', '', '', '', 0.0, 0.0, 1.0,
                'INVALID_POSTERIOR_NORMALIZATION', tuple())
        hypotheses = [FeederHypothesisValue(
            row=int(cell['row']), col=int(cell['col']), cell_id=cell['cell_id'],
            gps_likelihood=gps_likelihood, visual_likelihood=visual,
            transition_prior=transition, posterior=score / total,
            mahalanobis_d2=distance2, map_age_sec=map_age_sec,
        ) for cell, distance2, gps_likelihood, visual, transition, score in raw]
        hypotheses.sort(key=lambda value: value.posterior, reverse=True)
        top = hypotheses[0]
        second = hypotheses[1] if len(hypotheses) > 1 else None
        top2_probability = second.posterior if second else 0.0
        if len(hypotheses) <= 1:
            entropy = 0.0
        else:
            entropy = -sum(value.posterior * math.log(max(value.posterior, 1e-12))
                           for value in hypotheses) / math.log(len(hypotheses))

        if not allow_commit:
            status, reason = STATUS_AMBIGUOUS, 'REQUIRED_EVIDENCE_MISSING_OR_STALE'
        elif top.posterior < self.commit_probability:
            status, reason = STATUS_AMBIGUOUS, 'TOP1_BELOW_THRESHOLD'
        elif top.posterior - top2_probability < self.commit_margin:
            status, reason = STATUS_AMBIGUOUS, 'TOP1_TOP2_MARGIN_TOO_SMALL'
        elif entropy > self.max_entropy:
            status, reason = STATUS_AMBIGUOUS, 'POSTERIOR_ENTROPY_TOO_HIGH'
        else:
            status, reason = STATUS_CONFIRMED, 'COMMITTED'
            self.current_cell_id = top.cell_id
        current = top.cell_id if status == STATUS_CONFIRMED else ''
        next_cell = self._route_next(route, current) if current else ''
        return FeederDecision(
            status, previous, current, next_cell, top.cell_id,
            second.cell_id if second else '', top.posterior, top2_probability,
            entropy, reason, tuple(hypotheses))


def neighborhood_visual_likelihoods(observation_cells, map_cells, candidate_ids,
                                    sigma_m=1.0):
    """Compare local relative geometry against each candidate neighborhood.

    The observed cell nearest the observation centroid is treated as the local
    center.  Only relative positions are compared, so GPS remains solely a
    candidate generator.
    """
    if not observation_cells:
        return {cell_id: 1.0 for cell_id in candidate_ids}
    by_id = {cell['cell_id']: cell for cell in map_cells}
    by_index = {(int(cell['row']), int(cell['col'])): cell for cell in map_cells}
    mean_x = sum(cell['position'][0] for cell in observation_cells) / len(observation_cells)
    mean_y = sum(cell['position'][1] for cell in observation_cells) / len(observation_cells)
    center_observation = min(
        observation_cells,
        key=lambda cell: (cell['position'][0] - mean_x) ** 2
        + (cell['position'][1] - mean_y) ** 2)
    result = {}
    variance = max(1e-9, float(sigma_m) ** 2)
    for candidate_id in candidate_ids:
        candidate = by_id.get(candidate_id)
        if candidate is None:
            result[candidate_id] = 1e-12
            continue
        offset_i = int(candidate['row']) - int(center_observation['local_i'])
        offset_j = int(candidate['col']) - int(center_observation['local_j'])
        squared_error = 0.0
        matches = 0
        for observation in observation_cells:
            target = by_index.get((int(observation['local_i']) + offset_i,
                                   int(observation['local_j']) + offset_j))
            if target is None:
                squared_error += 9.0 * variance
                continue
            observed_dx = observation['position'][0] - center_observation['position'][0]
            observed_dy = observation['position'][1] - center_observation['position'][1]
            expected_dx = target['position'][0] - candidate['position'][0]
            expected_dy = target['position'][1] - candidate['position'][1]
            squared_error += (observed_dx - expected_dx) ** 2 + (observed_dy - expected_dy) ** 2
            matches += 1
        result[candidate_id] = math.exp(-0.5 * squared_error / variance) * (
            max(1, matches) / float(len(observation_cells)))
    return result
