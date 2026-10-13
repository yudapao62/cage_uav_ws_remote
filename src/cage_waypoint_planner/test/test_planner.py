#!/usr/bin/env python3
import math
import unittest
from cage_geo.geodesy import LocalENU
from cage_waypoint_planner.planner import MissionPlanner, PlannerConfig
from cage_grid.model import CageGridModel


class PlannerTest(unittest.TestCase):
    def setUp(self):
        self.home = (-35.363261, 149.165230, 584.0)
        local = LocalENU(*self.home)
        self.a = local.to_wgs84(100.0, 30.0, 0.0)
        self.b = local.to_wgs84(160.0, 30.0, 0.0)

    def test_required_sequence_and_geometry(self):
        points = MissionPlanner().plan(self.home, self.a, self.b)
        self.assertEqual([p.waypoint_type for p in points],
                         ['TAKEOFF','TRANSIT','CAGE_ENTER','FEED_START','FEED_END','CAGE_EXIT'])
        local = LocalENU(*self.home)
        enu = [local.to_enu(p.lat, p.lon, self.home[2]) for p in points]
        self.assertAlmostEqual(enu[1][0], 80.0, delta=0.05)
        self.assertAlmostEqual(enu[3][0], 100.0, delta=0.05)
        self.assertAlmostEqual(enu[4][0], 160.0, delta=0.05)
        self.assertAlmostEqual(enu[5][0], 180.0, delta=0.05)
        self.assertTrue(all(abs(p.yaw-90.0) < 1e-5 for p in points))

    def test_relative_altitudes(self):
        points = MissionPlanner().plan(self.home, self.a, self.b)
        self.assertEqual([p.alt for p in points], [20, 30, 15, 15, 15, 15])

    def test_max_distance_guard(self):
        with self.assertRaises(ValueError):
            MissionPlanner(PlannerConfig(max_leg_distance=50)).plan(self.home, self.a, self.b)

    def test_cell_mission_sequence(self):
        model = CageGridModel(rows=1, cols=2, reference=self.home)
        targets = model.targets('FULL_GRID')
        points = MissionPlanner().plan_cells(self.home, targets, 25.0)
        self.assertEqual([p.waypoint_type for p in points],
                         ['TAKEOFF', 'TRANSIT', 'GRID_ENTER', 'CELL_TARGET', 'CELL_TARGET', 'GRID_EXIT'])
        self.assertEqual([p.cell_id for p in points[3:5]], ['R01C01', 'R01C02'])

    def test_circular_center_single_feeding_target(self):
        points=MissionPlanner().plan_circular(self.home,self.a,self.b,5.0,25.0,1.5)
        self.assertEqual([p.waypoint_type for p in points],['TAKEOFF','TRANSIT','FEED_POINT'])
        self.assertEqual(sum(p.waypoint_type=='FEED_POINT' for p in points),1)
        center=LocalENU(*self.home).to_enu(points[-1].lat,points[-1].lon,self.home[2])
        self.assertAlmostEqual(center[0],130.0,delta=0.05)
        self.assertAlmostEqual(center[1],30.0,delta=0.05)
        self.assertEqual(points[-1].alt,15.0)
        self.assertEqual(points[-1].hold_time,5.0)
        self.assertEqual(points[-1].acceptance_radius,1.5)

    def test_circular_distance_and_dwell_guards(self):
        with self.assertRaises(ValueError): MissionPlanner().plan_circular(self.home,self.a,self.b,0.0)
        with self.assertRaises(ValueError):
            MissionPlanner(PlannerConfig(max_leg_distance=50)).plan_circular(self.home,self.a,self.b,5.0)


if __name__ == '__main__': unittest.main()
