#!/usr/bin/env python3
import unittest
from cage_msgs.msg import MissionWaypoint
from cage_mavros_adapter.adapter import MavrosAdapter

class AdapterTest(unittest.TestCase):
    def test_takeoff_conversion_uses_relative_frame(self):
        item = MissionWaypoint(seq=0, lat=22.0, lon=114.0, alt=20.0, yaw=90,
                               acceptance_radius=3, speed=5, waypoint_type='TAKEOFF')
        wp = MavrosAdapter.to_mavros_waypoint(item)
        self.assertEqual(wp.frame, wp.FRAME_GLOBAL_REL_ALT)
        self.assertEqual(wp.command, 22)
        self.assertTrue(wp.is_current)
    def test_feed_point_sets_mavlink_hold_time(self):
        item = MissionWaypoint(seq=2, lat=22.0, lon=114.0, alt=15.0, yaw=0,
                               acceptance_radius=1.5, speed=5, hold_time=5.0,
                               waypoint_type='FEED_POINT')
        wp = MavrosAdapter.to_mavros_waypoint(item)
        self.assertEqual(wp.command,16)
        self.assertEqual(wp.param1,5.0)

if __name__ == '__main__': unittest.main()
