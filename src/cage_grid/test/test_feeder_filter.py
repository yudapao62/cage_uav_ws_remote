import unittest

from cage_grid.feeder_filter import (
    STATUS_AMBIGUOUS,
    STATUS_CONFIRMED,
    STATUS_STALE_MAP,
    FeederIdentityFilter,
    neighborhood_visual_likelihoods,
)


def cells():
    return [
        {'row': 1, 'col': col, 'cell_id': 'R01C{:02d}'.format(col),
         'position': ((col - 1) * 10.0, 0.0, 0.0),
         'covariance_xy': (4.0, 0.0, 0.0, 4.0), 'confirmed': True}
        for col in range(1, 4)
    ]


class FeederFilterTest(unittest.TestCase):
    def test_equal_adjacent_evidence_abstains(self):
        filter_ = FeederIdentityFilter(commit_probability=0.6, commit_margin=0.15)
        decision = filter_.update(
            cells()[1:], gps_xy=(15.0, 0.0),
            gps_covariance_xy=(25.0, 0.0, 0.0, 25.0),
            visual_likelihoods={'R01C02': 1.0, 'R01C03': 1.0}, reentry=True)
        self.assertEqual(decision.status, STATUS_AMBIGUOUS)
        self.assertEqual(decision.current_cell_id, '')

    def test_visual_and_task_evidence_commit_correct_cell(self):
        filter_ = FeederIdentityFilter(commit_probability=0.60, commit_margin=0.15)
        first = filter_.update(
            cells(), gps_xy=(0.5, 0.0),
            gps_covariance_xy=(9.0, 0.0, 0.0, 9.0),
            visual_likelihoods={'R01C01': 1.0, 'R01C02': 0.05},
            route=['R01C01', 'R01C02', 'R01C03'], reentry=True)
        self.assertEqual(first.status, STATUS_CONFIRMED)
        second = filter_.update(
            cells(), gps_xy=(9.0, 0.0),
            gps_covariance_xy=(16.0, 0.0, 0.0, 16.0),
            visual_likelihoods={'R01C01': 0.2, 'R01C02': 1.0, 'R01C03': 0.2},
            route=['R01C01', 'R01C02', 'R01C03'])
        self.assertEqual(second.status, STATUS_CONFIRMED)
        self.assertEqual(second.current_cell_id, 'R01C02')
        self.assertEqual(second.previous_cell_id, 'R01C01')

    def test_stale_map_never_commits(self):
        filter_ = FeederIdentityFilter(stale_map_sec=10.0)
        decision = filter_.update(
            cells(), gps_xy=(0.0, 0.0),
            gps_covariance_xy=(1.0, 0.0, 0.0, 1.0), map_age_sec=11.0)
        self.assertEqual(decision.status, STATUS_STALE_MAP)
        self.assertEqual(decision.current_cell_id, '')

    def test_missing_required_visual_evidence_abstains(self):
        filter_ = FeederIdentityFilter()
        decision = filter_.update(
            cells(), gps_xy=(0.0, 0.0),
            gps_covariance_xy=(1.0, 0.0, 0.0, 1.0),
            allow_commit=False)
        self.assertEqual(decision.status, STATUS_AMBIGUOUS)
        self.assertEqual(decision.current_cell_id, '')

    def test_neighborhood_likelihood_prefers_matching_relative_geometry(self):
        map_cells = cells()
        observations = [
            {'local_i': 0, 'local_j': 0, 'position': (0.0, 0.0, 0.0)},
            {'local_i': 0, 'local_j': 1, 'position': (10.0, 0.0, 0.0)},
        ]
        result = neighborhood_visual_likelihoods(
            observations, map_cells, ['R01C01', 'R01C02', 'R01C03'], sigma_m=1.0)
        self.assertGreater(result['R01C01'], result['R01C03'])


if __name__ == '__main__':
    unittest.main()
