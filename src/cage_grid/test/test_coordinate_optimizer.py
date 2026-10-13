import math
import unittest

from cage_grid.coordinate_optimizer import CoordinateOptimizer


def rmse(points, truth):
    return math.sqrt(sum(
        (points[index][0] - truth[index][0]) ** 2
        + (points[index][1] - truth[index][1]) ** 2
        for index in truth) / len(truth))


class CoordinateOptimizerTest(unittest.TestCase):
    def setUp(self):
        self.nominal = {(1, col): ((col - 0.5) * 10.0, 5.0, 0.0)
                        for col in range(1, 6)}
        # R01C01 remains anchored while spacing and bend accumulate outward.
        self.truth = {(1, col): ((col - 0.5) * 10.0 + 0.35 * (col - 1),
                                 5.0 + 0.4 * math.sin((col - 1) * 0.6), 0.0)
                      for col in range(1, 6)}

    def test_observations_improve_nominal_rmse(self):
        optimizer = CoordinateOptimizer(
            self.nominal, nominal_sigma_m=5.0, anchor_sigma_m=0.1,
            relative_sigma_m=0.2, huber_delta_m=1.0)
        for timestamp in (1.0, 2.0, 3.0):
            optimizer.add_observation({
                index: {'position': value, 'covariance_xy': (0.04, 0.0, 0.0, 0.04)}
                for index, value in self.truth.items()
            }, timestamp)
        optimized, stats = optimizer.optimize(now=3.0)
        nominal_error = rmse(self.nominal, self.truth)
        optimized_error = rmse({k: v['position'] for k, v in optimized.items()}, self.truth)
        self.assertLess(optimized_error, nominal_error * 0.70)
        self.assertGreater(stats['factor_count'], 0)

    def test_huber_limits_one_large_outlier(self):
        optimizer = CoordinateOptimizer(
            self.nominal, nominal_sigma_m=5.0, anchor_sigma_m=0.1,
            relative_sigma_m=0.2, huber_delta_m=0.75)
        clean = {index: {'position': value, 'covariance_xy': (0.04, 0.0, 0.0, 0.04)}
                 for index, value in self.truth.items()}
        optimizer.add_observation(clean, 1.0)
        outlier = dict(clean)
        outlier[(1, 4)] = {'position': (200.0, -100.0, 0.0),
                           'covariance_xy': (0.04, 0.0, 0.0, 0.04)}
        optimizer.add_observation(outlier, 2.0)
        optimizer.add_observation(clean, 3.0)
        optimized, stats = optimizer.optimize(now=3.0)
        error = rmse({k: v['position'] for k, v in optimized.items()}, self.truth)
        self.assertLess(error, 1.0)
        self.assertGreater(stats['outlier_count'], 0)

    def test_staleness_increases_covariance(self):
        optimizer = CoordinateOptimizer(self.nominal, stale_variance_rate_m2_per_sec=0.1)
        optimizer.add_observation({
            (1, 1): {'position': self.truth[(1, 1)],
                     'covariance_xy': (0.1, 0.0, 0.0, 0.1)}
        }, 1.0)
        fresh, _ = optimizer.optimize(indices=[(1, 1)], now=1.0)
        stale, _ = optimizer.optimize(indices=[(1, 1)], now=11.0)
        self.assertAlmostEqual(stale[(1, 1)]['covariance_xy'][0]
                               - fresh[(1, 1)]['covariance_xy'][0], 1.0, places=6)


if __name__ == '__main__':
    unittest.main()

