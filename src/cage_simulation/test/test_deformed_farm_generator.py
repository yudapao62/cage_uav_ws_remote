import dataclasses
import json
import math
import os
import sys
import tempfile
import unittest


HERE = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(WORKSPACE, 'src', 'cage_grid', 'src'))
sys.path.insert(0, os.path.join(WORKSPACE, 'src', 'cage_simulation', 'scripts'))

from cage_grid.deformed_model import DeformationConfig, DeformedGrid  # noqa: E402
from generate_fish_farm import generate  # noqa: E402


class DeformedGridTest(unittest.TestCase):
    def build(self, config=None, holes=None):
        return DeformedGrid(
            rows=6,
            cols=10,
            cell_size_x=12.0,
            cell_size_y=12.0,
            origin_east=70.0,
            origin_north=-6.0,
            yaw_deg=12.0,
            config=config or DeformationConfig(),
            structural_holes=holes or [],
        )

    def test_anchor_cell_count_and_shared_edges(self):
        grid = self.build()
        self.assertEqual((grid.vertices[(0, 0)].east, grid.vertices[(0, 0)].north),
                         (70.0, -6.0))
        self.assertEqual(len(grid.vertices), 77)
        self.assertEqual(len(grid.cells), 60)
        left = grid.cells[(2, 3)]
        right = grid.cells[(2, 4)]
        self.assertEqual(left.vertex_indices[1], right.vertex_indices[0])
        self.assertEqual(left.vertex_indices[2], right.vertex_indices[3])
        self.assertEqual(left.polygon_enu[1], right.polygon_enu[0])
        self.assertEqual(left.polygon_enu[2], right.polygon_enu[3])

    def test_same_seed_is_reproducible_and_different_seed_changes_geometry(self):
        first = self.build(DeformationConfig(random_seed=123)).manifest()
        second = self.build(DeformationConfig(random_seed=123)).manifest()
        third = self.build(DeformationConfig(random_seed=124)).manifest()
        self.assertEqual(first['geometry_sha256'], second['geometry_sha256'])
        self.assertNotEqual(first['geometry_sha256'], third['geometry_sha256'])

    def test_nominal_mode_is_affine(self):
        config = dataclasses.replace(DeformationConfig(), enabled=False)
        grid = self.build(config)
        yaw = math.radians(12.0)
        vertex = grid.vertices[(2, 3)]
        local_x, local_y = 36.0, 24.0
        self.assertAlmostEqual(vertex.east, 70.0 + math.cos(yaw) * local_x - math.sin(yaw) * local_y)
        self.assertAlmostEqual(vertex.north, -6.0 + math.sin(yaw) * local_x + math.cos(yaw) * local_y)

    def test_all_presets_pass_geometry_validation(self):
        presets = {
            'mild': dict(row_spacing_cv=0.02, col_spacing_cv=0.02,
                         smooth_warp_amplitude_m=0.35, warp_correlation_cells=3.5,
                         bend_amplitude_m=0.25, shear_deg=1.0,
                         local_residual_std_m=0.04),
            'medium': dict(row_spacing_cv=0.04, col_spacing_cv=0.04,
                           smooth_warp_amplitude_m=0.80, warp_correlation_cells=3.0,
                           bend_amplitude_m=0.60, shear_deg=2.0,
                           local_residual_std_m=0.08),
            'strong': dict(row_spacing_cv=0.07, col_spacing_cv=0.07,
                           smooth_warp_amplitude_m=1.30, warp_correlation_cells=2.5,
                           bend_amplitude_m=1.00, shear_deg=4.0,
                           local_residual_std_m=0.12),
        }
        for name, values in presets.items():
            with self.subTest(name=name):
                self.assertTrue(self.build(DeformationConfig(**values)).validate())

    def test_structural_hole_is_not_a_neighbor(self):
        grid = self.build(holes=['R02C03'])
        self.assertFalse(grid.cells[(2, 3)].exists)
        self.assertNotIn('R02C03', grid.cells[(2, 2)].neighbors)
        self.assertNotIn('R02C03', grid.cells[(1, 3)].neighbors)

    def test_generator_writes_matching_sdf_and_manifest(self):
        config = os.path.join(WORKSPACE, 'src', 'cage_simulation', 'config', 'fish_farm.yaml')
        with tempfile.TemporaryDirectory() as directory:
            sdf = os.path.join(directory, 'fish_farm', 'model.sdf')
            manifest = os.path.join(directory, 'generated', 'fish_farm_gt.json')
            result = generate(config, sdf, manifest)
            self.assertTrue(os.path.exists(sdf))
            self.assertTrue(os.path.exists(manifest))
            with open(manifest, encoding='utf-8') as stream:
                payload = json.load(stream)
            self.assertEqual(result['geometry_sha256'], payload['geometry_sha256'])
            self.assertEqual(len(payload['vertices']), 77)
            self.assertEqual(len(payload['cells']), 60)


if __name__ == '__main__':
    unittest.main()

