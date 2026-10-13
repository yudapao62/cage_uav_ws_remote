import unittest

from cage_grid.feeder_filter import FeederIdentityFilter, STATUS_LOST


def cell(row, col, x, variance=0.25):
    return {'row': row, 'col': col, 'cell_id': 'R{:02d}C{:02d}'.format(row, col),
            'position': (x, 0.0, 0.0),
            'covariance_xy': (variance, 0.0, 0.0, variance), 'confirmed': True}


class FeederCandidateGateTest(unittest.TestCase):
    def test_true_cell_enters_covariance_gate(self):
        filter_ = FeederIdentityFilter(commit_probability=0.0, commit_margin=0.0,
                                       max_entropy=1.0)
        decision = filter_.update(
            [cell(1, 1, 0.0), cell(1, 2, 10.0), cell(1, 3, 20.0)],
            gps_xy=(11.0, 0.0), gps_covariance_xy=(4.0, 0.0, 0.0, 4.0))
        self.assertIn('R01C02', [value.cell_id for value in decision.hypotheses])

    def test_map_uncertainty_expands_candidate_set(self):
        filter_ = FeederIdentityFilter(commit_probability=0.0, commit_margin=0.0,
                                       max_entropy=1.0)
        certain = filter_.update(
            [cell(1, 1, 0.0, 0.01), cell(1, 2, 10.0, 0.01)],
            gps_xy=(0.0, 0.0), gps_covariance_xy=(0.1, 0.0, 0.0, 0.1))
        uncertain = filter_.update(
            [cell(1, 1, 0.0, 0.01), cell(1, 2, 10.0, 100.0)],
            gps_xy=(0.0, 0.0), gps_covariance_xy=(0.1, 0.0, 0.0, 0.1), reentry=True)
        self.assertEqual(len(certain.hypotheses), 1)
        self.assertEqual(len(uncertain.hypotheses), 2)

    def test_no_candidate_is_lost(self):
        filter_ = FeederIdentityFilter()
        decision = filter_.update(
            [cell(1, 1, 0.0)], gps_xy=(100.0, 0.0),
            gps_covariance_xy=(0.1, 0.0, 0.0, 0.1))
        self.assertEqual(decision.status, STATUS_LOST)


if __name__ == '__main__':
    unittest.main()

