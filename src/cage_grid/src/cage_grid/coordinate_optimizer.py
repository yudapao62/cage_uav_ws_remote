"""Small robust lattice-coordinate optimizer using NumPy least squares."""
import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AbsoluteFactor:
    index: tuple
    position: tuple
    variance: float
    timestamp: float
    confidence: float


@dataclass(frozen=True)
class RelativeFactor:
    source: tuple
    target: tuple
    delta: tuple
    variance: float
    timestamp: float


class CoordinateOptimizer:
    def __init__(self, nominal_cells, nominal_sigma_m=4.0, anchor_sigma_m=0.05,
                 relative_sigma_m=0.35, huber_delta_m=1.0,
                 stale_variance_rate_m2_per_sec=0.0025, max_factors_per_cell=30):
        self.nominal_cells = {index: tuple(value) for index, value in nominal_cells.items()}
        self.nominal_sigma_m = float(nominal_sigma_m)
        self.anchor_sigma_m = float(anchor_sigma_m)
        self.relative_sigma_m = float(relative_sigma_m)
        self.huber_delta_m = float(huber_delta_m)
        self.stale_variance_rate = float(stale_variance_rate_m2_per_sec)
        self.max_factors_per_cell = int(max_factors_per_cell)
        positive = (self.nominal_sigma_m, self.anchor_sigma_m,
                    self.relative_sigma_m, self.huber_delta_m)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('optimizer sigmas and Huber delta must be positive')
        if self.stale_variance_rate < 0.0 or self.max_factors_per_cell < 1:
            raise ValueError('invalid staleness rate or factor limit')
        self.absolute_factors = []
        self.relative_factors = []
        self.last_observed = {}

    def add_observation(self, observed_cells, timestamp):
        """Add committed global observations keyed by ``(row,col)``."""
        timestamp = float(timestamp)
        current = {}
        for index, value in observed_cells.items():
            if index not in self.nominal_cells:
                continue
            position = tuple(float(component) for component in value['position'][:2])
            covariance = value.get('covariance_xy', (1.0, 0.0, 0.0, 1.0))
            variance = max(1e-6, 0.5 * (float(covariance[0]) + float(covariance[3])))
            confidence = max(0.05, min(1.0, float(value.get('confidence', 1.0))))
            factor = AbsoluteFactor(index, position, variance, timestamp, confidence)
            self.absolute_factors.append(factor)
            current[index] = factor
            self.last_observed[index] = timestamp
        for source in sorted(current):
            for target in ((source[0] + 1, source[1]), (source[0], source[1] + 1)):
                if target not in current:
                    continue
                a, b = current[source], current[target]
                delta = (b.position[0] - a.position[0], b.position[1] - a.position[1])
                variance = self.relative_sigma_m ** 2 + a.variance + b.variance
                self.relative_factors.append(
                    RelativeFactor(source, target, delta, variance, timestamp))
        self._trim_factors()

    def _trim_factors(self):
        by_index = {}
        for factor in self.absolute_factors:
            by_index.setdefault(factor.index, []).append(factor)
        self.absolute_factors = []
        for factors in by_index.values():
            self.absolute_factors.extend(factors[-self.max_factors_per_cell:])
        max_relative = self.max_factors_per_cell * max(1, len(self.nominal_cells) * 2)
        if len(self.relative_factors) > max_relative:
            self.relative_factors = self.relative_factors[-max_relative:]

    def _factor_rows(self, indices):
        lookup = {index: offset for offset, index in enumerate(indices)}
        factors = []
        nominal_variance = self.nominal_sigma_m ** 2
        for index in indices:
            row = np.zeros(len(indices), dtype=float)
            row[lookup[index]] = 1.0
            nominal = self.nominal_cells[index]
            factors.append((row, np.array(nominal[:2], dtype=float),
                            1.0 / nominal_variance, 'nominal'))
        # The configured real anchor fixes translation.  With cell-center-only
        # variables, R01C01 is the closest representable anchor factor.
        anchor = min(indices)
        row = np.zeros(len(indices), dtype=float)
        row[lookup[anchor]] = 1.0
        factors.append((row, np.array(self.nominal_cells[anchor][:2], dtype=float),
                        1.0 / (self.anchor_sigma_m ** 2), 'anchor'))
        for factor in self.absolute_factors:
            if factor.index not in lookup:
                continue
            row = np.zeros(len(indices), dtype=float)
            row[lookup[factor.index]] = 1.0
            weight = factor.confidence / factor.variance
            factors.append((row, np.array(factor.position, dtype=float), weight, 'absolute'))
        for factor in self.relative_factors:
            if factor.source not in lookup or factor.target not in lookup:
                continue
            row = np.zeros(len(indices), dtype=float)
            row[lookup[factor.source]] = -1.0
            row[lookup[factor.target]] = 1.0
            factors.append((row, np.array(factor.delta, dtype=float),
                            1.0 / factor.variance, 'relative'))
        return factors

    def optimize(self, indices=None, now=None, iterations=10):
        indices = sorted(indices or self.nominal_cells)
        if not indices:
            return {}, {'factor_count': 0, 'outlier_count': 0, 'rmse': 0.0}
        factors = self._factor_rows(indices)
        design = np.stack([item[0] for item in factors], axis=0)
        targets = np.stack([item[1] for item in factors], axis=0)
        base_weights = np.array([item[2] for item in factors], dtype=float)
        robust = np.ones(len(factors), dtype=float)
        solution = np.stack([np.array([self.nominal_cells[index][axis]
                                       for index in indices], dtype=float)
                             for axis in (0, 1)], axis=1)
        for _ in range(max(1, int(iterations))):
            weights = np.sqrt(np.maximum(1e-12, base_weights * robust))
            weighted_design = design * weights[:, None]
            for axis in (0, 1):
                solution[:, axis] = np.linalg.lstsq(
                    weighted_design, targets[:, axis] * weights, rcond=None)[0]
            residual = design.dot(solution) - targets
            norms = np.linalg.norm(residual, axis=1)
            robust = np.ones_like(norms)
            mask = norms > self.huber_delta_m
            robust[mask] = self.huber_delta_m / np.maximum(norms[mask], 1e-12)

        normal = design.T.dot((base_weights * robust)[:, None] * design)
        # Normalize before the SVD used by pinv.  Dense multi-frame maps can
        # otherwise create an avoidable dynamic-range warning even though the
        # weighted least-squares solution itself is well conditioned.
        scale = max(1.0, float(np.max(np.abs(np.diag(normal)))))
        try:
            covariance = np.linalg.inv(normal / scale) / scale
        except np.linalg.LinAlgError:
            covariance = np.linalg.pinv(normal / scale) / scale
        covariance = np.nan_to_num(covariance, nan=self.nominal_sigma_m ** 2,
                                   posinf=self.nominal_sigma_m ** 2,
                                   neginf=self.nominal_sigma_m ** 2)
        now = float(now if now is not None else max(self.last_observed.values(), default=0.0))
        result = {}
        for offset, index in enumerate(indices):
            stale_sec = max(0.0, now - self.last_observed.get(index, now))
            variance = max(1e-9, float(covariance[offset, offset]))
            variance += stale_sec * self.stale_variance_rate
            result[index] = {
                'position': (float(solution[offset, 0]), float(solution[offset, 1]),
                             float(self.nominal_cells[index][2] if len(self.nominal_cells[index]) > 2 else 0.0)),
                'covariance_xy': (variance, 0.0, 0.0, variance),
                'stale_sec': stale_sec,
            }
        residual = design.dot(solution) - targets
        norms = np.linalg.norm(residual, axis=1)
        return result, {
            'factor_count': len(factors),
            'outlier_count': int(np.sum(norms > self.huber_delta_m)),
            'rmse': float(math.sqrt(float(np.mean(norms ** 2)))) if len(norms) else 0.0,
        }
