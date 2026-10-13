import unittest

from cage_grid.association import AssociationDecision
from cage_grid.persistent_map import PersistentLatticeMap


def nominal(rows, cols):
    return {(row, col): ((col - 0.5) * 10.0, (row - 0.5) * 10.0, 0.0)
            for row in range(1, rows + 1) for col in range(1, cols + 1)}


def frame(cells, rows, cols):
    return {
        'local_rows': rows,
        'local_cols': cols,
        'cells': [{
            'local_i': i, 'local_j': j,
            'projected_center': ((j + 0.5) * 10.0, (i + 0.5) * 10.0, 0.0),
            'projected_covariance_xy': (1.0, 0.0, 0.0, 1.0),
            'confidence': 1.0,
        } for i, j in cells],
    }


class BirthVisibilityTest(unittest.TestCase):
    def test_boundary_birth_grows_one_column(self):
        lattice = PersistentLatticeMap(1, 4, nominal(1, 4), min_confirmations=1)
        lattice.apply_observation(
            frame([(0, 0), (0, 1)], 1, 2),
            AssociationDecision(True, 1, 1, 'COMMITTED', tuple()), 1.0)
        lattice.apply_observation(
            frame([(0, 0)], 1, 1),
            AssociationDecision(True, 1, 3, 'COMMITTED', tuple()), 2.0)
        self.assertIn((1, 3), lattice.cells)
        self.assertEqual(lattice.cells[(1, 3)].cell_id, 'R01C03')

    def test_internal_missing_corner_is_not_born_later(self):
        lattice = PersistentLatticeMap(2, 2, nominal(2, 2), min_confirmations=1)
        lattice.apply_observation(
            frame([(0, 0), (0, 1), (1, 0)], 2, 2),
            AssociationDecision(True, 1, 1, 'COMMITTED', tuple()), 1.0)
        self.assertNotIn((2, 2), lattice.cells)
        events = lattice.apply_observation(
            frame([(0, 0)], 1, 1),
            AssociationDecision(True, 2, 2, 'COMMITTED', tuple()), 2.0)
        self.assertNotIn((2, 2), lattice.cells)
        self.assertTrue(any(event['type'] == 'BIRTH_REJECTED_INTERNAL_OR_DISCONNECTED'
                            for event in events))

    def test_never_expected_frontier_can_fill_inside_existing_bounds(self):
        lattice = PersistentLatticeMap(3, 4, nominal(3, 4), min_confirmations=1)
        lattice.apply_observation(
            frame([(0, 0), (0, 1), (0, 2), (0, 3)], 1, 4),
            AssociationDecision(True, 1, 1, 'COMMITTED', tuple()), 1.0)
        lattice.apply_observation(
            frame([(0, 0), (0, 1)], 1, 2),
            AssociationDecision(True, 2, 3, 'COMMITTED', tuple()), 1.5)
        self.assertNotIn((2, 1), lattice.cells)
        lattice.apply_observation(
            frame([(0, 0), (0, 1)], 1, 2),
            AssociationDecision(True, 2, 1, 'COMMITTED', tuple()), 2.0)
        self.assertIn((2, 1), lattice.cells)
        self.assertIn((2, 2), lattice.cells)

    def test_previously_expected_miss_requires_repeated_evidence(self):
        lattice = PersistentLatticeMap(2, 2, nominal(2, 2), min_confirmations=1)
        decision = AssociationDecision(True, 1, 1, 'COMMITTED', tuple())
        lattice.apply_observation(frame([(0, 0), (0, 1), (1, 0)], 2, 2),
                                  decision, 1.0)
        lattice.apply_observation(frame([(1, 1)], 2, 2), decision, 2.0)
        self.assertNotIn((2, 2), lattice.cells)
        lattice.apply_observation(frame([(1, 1)], 2, 2), decision, 3.0)
        self.assertIn((2, 2), lattice.cells)


if __name__ == '__main__':
    unittest.main()
