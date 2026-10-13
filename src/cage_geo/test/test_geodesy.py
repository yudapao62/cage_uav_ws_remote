#!/usr/bin/env python3
import math
import unittest
from cage_geo.geodesy import LocalENU, cage_frame


class GeodesyTest(unittest.TestCase):
    def test_home_is_origin(self):
        enu = LocalENU(-35.363261, 149.165230, 584.0)
        self.assertLess(max(map(abs, enu.to_enu(-35.363261, 149.165230, 584.0))), 1e-7)

    def test_round_trip(self):
        enu = LocalENU(22.5, 114.0, 12.0)
        original = (22.5007, 114.0012, 45.0)
        result = enu.to_wgs84(*enu.to_enu(*original))
        self.assertAlmostEqual(result[0], original[0], places=8)
        self.assertAlmostEqual(result[1], original[1], places=8)
        self.assertAlmostEqual(result[2], original[2], places=3)

    def test_cage_axes(self):
        frame = cage_frame((100.0, 30.0), (160.0, 30.0))
        self.assertEqual(frame.length, 60.0)
        self.assertEqual((frame.unit_east, frame.unit_north), (1.0, 0.0))
        self.assertEqual((frame.normal_east, frame.normal_north), (0.0, 1.0))

    def test_reject_short_cage(self):
        with self.assertRaises(ValueError): cage_frame((1, 1), (1, 1))


if __name__ == '__main__': unittest.main()
