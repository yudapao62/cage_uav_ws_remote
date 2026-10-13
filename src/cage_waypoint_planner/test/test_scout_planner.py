import math
import unittest

from cage_geo.geodesy import LocalENU
from cage_waypoint_planner.scout_planner import (
    ScoutRouteConfig,
    plan_scout_mission,
    serpentine_survey_points,
)


class ScoutPlannerTest(unittest.TestCase):
    def setUp(self):
        self.reference = (-35.363261, 149.165230, 584.0)

    def test_default_route_has_twelve_survey_endpoints(self):
        config = ScoutRouteConfig()
        points = serpentine_survey_points(config)
        self.assertEqual(12, len(points))
        self.assertLess(points[0][0], points[1][0])
        self.assertGreater(points[2][0], points[3][0])

        mission = plan_scout_mission(self.reference, self.reference, config)
        self.assertEqual('TAKEOFF', mission[0].waypoint_type)
        self.assertEqual('TRANSIT', mission[1].waypoint_type)
        survey = [value for value in mission
                  if value.waypoint_type.startswith('SCOUT_')]
        self.assertEqual(12, len(survey))
        self.assertEqual('SCOUT_ENTER', survey[0].waypoint_type)
        self.assertEqual('SCOUT_EXIT', survey[-1].waypoint_type)
        self.assertTrue(all(value.alt == 25.0 for value in mission))

    def test_route_uses_nominal_geometry_not_ground_truth(self):
        config = ScoutRouteConfig(origin_x=70.0, origin_y=-6.0,
                                  rows=2, cols=3, yaw_deg=15.0)
        mission = plan_scout_mission(self.reference, self.reference, config)
        local = LocalENU(*self.reference)
        survey = [value for value in mission
                  if value.waypoint_type.startswith('SCOUT_')]
        decoded = [local.to_enu(value.lat, value.lon, self.reference[2])[:2]
                   for value in survey]
        expected = serpentine_survey_points(config)
        for actual, target in zip(decoded, expected):
            self.assertLess(math.hypot(actual[0] - target[0],
                                       actual[1] - target[1]), 0.05)

    def test_invalid_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            serpentine_survey_points(ScoutRouteConfig(rows=0))
        with self.assertRaises(ValueError):
            serpentine_survey_points(ScoutRouteConfig(survey_relative_alt=-1.0))


if __name__ == '__main__':
    unittest.main()
