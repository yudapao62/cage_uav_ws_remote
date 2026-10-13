import unittest

from cage_grid.deformed_model import DeformationConfig, DeformedGrid
from cage_grid.oracle_observation import (
    OracleObservationGenerator,
    cells_in_oriented_footprint,
    cells_in_window,
)


def manifest_cells():
    return DeformedGrid(
        3, 4, 10.0, 10.0,
        config=DeformationConfig(enabled=False),
    ).manifest()['cells']


class OracleObservationTest(unittest.TestCase):
    def test_window_and_local_indices(self):
        selected = cells_in_window(manifest_cells(), 2, 3, 2, 4)
        result = OracleObservationGenerator().generate(selected, frame_seq=1)
        self.assertEqual((result['local_rows'], result['local_cols']), (2, 3))
        self.assertEqual((result['cells'][0]['local_i'], result['cells'][0]['local_j']), (0, 0))
        self.assertEqual((result['cells'][-1]['local_i'], result['cells'][-1]['local_j']), (1, 2))

    def test_internal_forced_miss_preserves_neighbor_indices(self):
        selected = cells_in_window(manifest_cells(), 1, 1, 1, 4)
        result = OracleObservationGenerator().generate(
            selected, frame_seq=7, forced_drop_ids=['R01C02'])
        pairs = [(item['local_j'], item['_ground_truth_cell_id']) for item in result['cells']]
        self.assertEqual(pairs, [(0, 'R01C01'), (2, 'R01C03'), (3, 'R01C04')])

    def test_same_frame_and_seed_are_reproducible(self):
        selected = cells_in_window(manifest_cells(), 1, 2, 1, 3)
        generator = OracleObservationGenerator(
            random_seed=11, position_noise_std_m=0.2,
            dropout_probability=0.1, edge_observed_probability=0.7)
        self.assertEqual(generator.generate(selected, 99), generator.generate(selected, 99))

    def test_oriented_footprint(self):
        cells = manifest_cells()
        selected = cells_in_oriented_footprint(cells, (10.0, 10.0), 0.0, 21.0, 21.0)
        ids = {item['cell_id'] for item in selected}
        self.assertEqual(ids, {'R01C01', 'R01C02', 'R02C01', 'R02C02'})


if __name__ == '__main__':
    unittest.main()

