#!/usr/bin/env python3
import unittest
from cage_grid.model import CageGridModel, CircularCageGeometry, normalize_geometry_mode
from cage_geo.geodesy import LocalENU


class GridModelTest(unittest.TestCase):
    def setUp(self):
        self.model = CageGridModel(rows=2, cols=3, cell_size_x=10.0, cell_size_y=10.0)

    def test_cell_ids_are_one_based(self):
        cell = self.model.cell_center(1, 2)
        self.assertEqual(cell.cell_id, 'R01C02')

    def test_full_grid_is_snake_order(self):
        ids = [c.cell_id for c in self.model.targets('FULL_GRID')]
        self.assertEqual(ids, ['R01C01', 'R01C02', 'R01C03', 'R02C03', 'R02C02', 'R02C01'])

    def test_selected_cells_can_use_nearest_order(self):
        ids = [c.cell_id for c in self.model.targets('SELECTED_CELLS', selected_cell_ids=['R02C03', 'R01C01'], start_xy=(0, 0))]
        self.assertEqual(ids[0], 'R01C01')

    def test_geometry_mode_validation(self):
        self.assertEqual(normalize_geometry_mode('RECTANGULAR'), 'rectangular')
        self.assertEqual(normalize_geometry_mode('circular'), 'circular')
        with self.assertRaises(ValueError): normalize_geometry_mode('invalid_mode')

    def test_circular_enu_midpoint_and_radius(self):
        home=(22.5,114.0,5.0); local=LocalENU(*home)
        a=local.to_wgs84(-50.0,0.0,0.0); b=local.to_wgs84(50.0,0.0,0.0)
        geometry=CircularCageGeometry.from_wgs84(a,b,home)
        self.assertAlmostEqual(geometry.center[0],0.0,places=5)
        self.assertAlmostEqual(geometry.center[1],0.0,places=5)
        self.assertAlmostEqual(geometry.radius,50.0,places=4)

    def test_circular_non_axis_midpoint_and_wgs84_round_trip(self):
        home=(22.5,114.0,5.0); local=LocalENU(*home)
        a=local.to_wgs84(10.0,20.0,0.0); b=local.to_wgs84(30.0,60.0,0.0)
        geometry=CircularCageGeometry.from_wgs84(a,b,home)
        self.assertAlmostEqual(geometry.center[0],20.0,places=5)
        self.assertAlmostEqual(geometry.center[1],40.0,places=5)
        center_enu=local.to_enu(*geometry.center_wgs84)
        self.assertAlmostEqual(center_enu[0],20.0,places=5)
        self.assertAlmostEqual(center_enu[1],40.0,places=5)


if __name__ == '__main__':
    unittest.main()
