#!/usr/bin/env python3
import math
import unittest

from cage_visualization.geometry import operation_zone


class OperationZoneTest(unittest.TestCase):
    def test_ab_is_operation_zone_diameter(self):
        point_a = (100.0, 30.0)
        point_b = (160.0, 30.0)
        zone = operation_zone(point_a, point_b, 20.0, 100)
        self.assertEqual(zone.center, (130.0, 30.0))
        self.assertAlmostEqual(math.dist(point_a, zone.center), zone.radius, delta=1e-6)
        self.assertAlmostEqual(math.dist(point_b, zone.center), zone.radius, delta=1e-6)

    def test_circle_and_rectangle_are_closed(self):
        zone = operation_zone((2.0, 3.0), (8.0, 11.0), 4.0, 100)
        self.assertEqual(len(zone.circle), 101)
        self.assertAlmostEqual(math.dist(zone.circle[0], zone.circle[-1]), 0.0, delta=1e-6)
        self.assertEqual(zone.rectangle[0], zone.rectangle[-1])

    def test_rejects_degenerate_zone(self):
        with self.assertRaises(ValueError):
            operation_zone((1.0, 1.0), (1.0, 1.0), 20.0)


if __name__ == '__main__':
    unittest.main()
