#!/usr/bin/env python3
import unittest

from cage_feeder.rc9_logic import FeederCycle, rc_override_values, rc_release_value


class FeederCycleTest(unittest.TestCase):
    def test_override_changes_only_channel_nine(self):
        values = rc_override_values(9, 2000)
        self.assertEqual(len(values), 18)
        self.assertEqual(values[8], 2000)
        self.assertTrue(all(value == 65535 for index, value in enumerate(values)
                            if index != 8))

    def test_mavlink_release_sentinel_depends_on_channel_range(self):
        self.assertEqual(rc_release_value(8), 0)
        self.assertEqual(rc_release_value(9), 65534)

    def test_timed_run_closes_then_releases(self):
        cycle = FeederCycle(close_hold_sec=1.0)
        cycle.start_timed(3.0, now=10.0)
        self.assertEqual(cycle.state, FeederCycle.ON)
        self.assertIsNone(cycle.update(12.999))
        self.assertEqual(cycle.update(13.0), (FeederCycle.ON, FeederCycle.CLOSING))
        self.assertFalse(cycle.enabled)
        self.assertIsNone(cycle.update(13.999))
        self.assertEqual(cycle.update(14.0), (FeederCycle.CLOSING, FeederCycle.RELEASED))

    def test_new_timed_command_restarts_deadline(self):
        cycle = FeederCycle(close_hold_sec=0.5)
        cycle.start_timed(2.0, now=0.0)
        cycle.start_timed(4.0, now=1.0)
        self.assertIsNone(cycle.update(4.9))
        self.assertEqual(cycle.update(5.0), (FeederCycle.ON, FeederCycle.CLOSING))

    def test_explicit_off_cancels_timed_run(self):
        cycle = FeederCycle(close_hold_sec=1.0)
        cycle.start_timed(10.0, now=0.0)
        cycle.set_enabled(False, now=2.0)
        self.assertEqual(cycle.state, FeederCycle.CLOSING)
        self.assertIsNone(cycle.deadline)
        self.assertEqual(cycle.update(3.0), (FeederCycle.CLOSING, FeederCycle.RELEASED))

    def test_continuous_command_has_no_deadline(self):
        cycle = FeederCycle(close_hold_sec=1.0)
        cycle.set_enabled(True, now=0.0)
        self.assertIsNone(cycle.deadline)
        self.assertIsNone(cycle.update(1000.0))
        self.assertTrue(cycle.enabled)

    def test_rejects_invalid_values(self):
        with self.assertRaises(ValueError):
            FeederCycle(-1.0)
        with self.assertRaises(ValueError):
            FeederCycle(0.0)
        with self.assertRaises(ValueError):
            FeederCycle(float('nan'))
        cycle = FeederCycle(1.0)
        with self.assertRaises(ValueError):
            cycle.start_timed(0.0, now=0.0)
        with self.assertRaises(ValueError):
            cycle.start_timed(float('inf'), now=0.0)
        with self.assertRaises(ValueError):
            cycle.start_timed(float('nan'), now=0.0)


if __name__ == '__main__':
    unittest.main()
