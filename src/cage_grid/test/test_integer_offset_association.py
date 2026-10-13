import unittest

from cage_grid.association import IntegerOffsetAssociator


def candidates(rows, cols, spacing=10.0):
    return {
        (row, col): {
            'position': ((col - 0.5) * spacing, (row - 0.5) * spacing, 0.0),
            'covariance_xy': (0.25, 0.0, 0.0, 0.25),
        }
        for row in range(1, rows + 1) for col in range(1, cols + 1)
    }


def observation(global_cells, delta_row, delta_col, spacing=10.0):
    result = []
    for row, col in global_cells:
        result.append({
            'local_i': row - delta_row,
            'local_j': col - delta_col,
            'projected_center': ((col - 0.5) * spacing, (row - 0.5) * spacing, 0.0),
            'projected_covariance_xy': (0.04, 0.0, 0.0, 0.04),
            'confidence': 1.0,
        })
    return result


class IntegerOffsetAssociationTest(unittest.TestCase):
    def test_recovers_correct_offset(self):
        associator = IntegerOffsetAssociator(4, 6)
        observed = observation([(2, 4), (2, 5), (3, 4), (3, 5)], 2, 4)
        decision = associator.associate(observed, candidates(4, 6))
        self.assertTrue(decision.committed)
        self.assertEqual((decision.delta_row, decision.delta_col), (2, 4))
        self.assertGreater(decision.hypotheses[0].posterior, 0.99)

    def test_internal_single_cell_miss_does_not_shift_offset(self):
        associator = IntegerOffsetAssociator(2, 6)
        # R01C03 is deliberately missing; local indices preserve the gap.
        observed = observation([(1, 2), (1, 4), (1, 5)], 1, 2)
        decision = associator.associate(observed, candidates(2, 6), previous_offset=(1, 2))
        self.assertTrue(decision.committed)
        self.assertEqual((decision.delta_row, decision.delta_col), (1, 2))

    def test_equal_adjacent_candidates_abstain(self):
        associator = IntegerOffsetAssociator(
            1, 2, commit_probability=0.6, commit_margin=0.2,
            nominal_sigma_m=1.0)
        observed = [{
            'local_i': 0, 'local_j': 0,
            'projected_center': (10.0, 5.0, 0.0),
            'projected_covariance_xy': (0.01, 0.0, 0.0, 0.01),
            'confidence': 1.0,
        }]
        decision = associator.associate(observed, candidates(1, 2))
        self.assertFalse(decision.committed)
        self.assertAlmostEqual(decision.hypotheses[0].posterior, 0.5, places=6)
        self.assertAlmostEqual(decision.hypotheses[1].posterior, 0.5, places=6)

    def test_motion_prior_keeps_previous_offset_when_geometry_is_weak(self):
        associator = IntegerOffsetAssociator(
            1, 3, commit_probability=0.5, commit_margin=0.1,
            nominal_sigma_m=100.0, motion_sigma_cells=0.25)
        observed = [{
            'local_i': 0, 'local_j': 0,
            'projected_center': (15.0, 5.0, 0.0),
            'projected_covariance_xy': (10000.0, 0.0, 0.0, 10000.0),
            'confidence': 0.2,
        }]
        decision = associator.associate(observed, candidates(1, 3), previous_offset=(1, 2))
        self.assertTrue(decision.committed)
        self.assertEqual((decision.delta_row, decision.delta_col), (1, 2))


if __name__ == '__main__':
    unittest.main()

