import unittest

from cage_mission_fsm.identity_gate import (
    STATUS_AMBIGUOUS,
    STATUS_CONFIRMED,
    STATUS_LOST,
    STATUS_MAP_MISMATCH,
    STATUS_STALE_MAP,
    identity_gate_decision,
)


class IdentityGateTest(unittest.TestCase):
    def test_only_fresh_matching_confirmed_identity_allows_feeding(self):
        self.assertEqual(identity_gate_decision(
            True, STATUS_CONFIRMED, 'R01C02', 'R01C02', 0.1, 2.0),
            (True, 'CONFIRMED'))

    def test_all_abstention_states_fail_closed(self):
        for status, reason in ((STATUS_AMBIGUOUS, 'AMBIGUOUS'),
                               (STATUS_LOST, 'LOST'),
                               (STATUS_STALE_MAP, 'STALE_MAP')):
            self.assertEqual(identity_gate_decision(
                True, status, '', 'R01C02', 0.1, 2.0), (False, reason))

    def test_stale_or_mismatched_identity_fails_closed(self):
        self.assertEqual(identity_gate_decision(
            True, STATUS_CONFIRMED, 'R01C02', 'R01C02', 2.1, 2.0),
            (False, 'STALE_IDENTITY'))
        self.assertEqual(identity_gate_decision(
            True, STATUS_CONFIRMED, 'R01C03', 'R01C02', 0.1, 2.0),
            (False, 'CELL_MISMATCH'))

    def test_full_map_pin_is_required_and_rechecked(self):
        expected = ('uuid-1', 7, 'map-sha', 'manifest-sha')
        self.assertEqual(identity_gate_decision(
            True, STATUS_CONFIRMED, 'R01C02', 'R01C02', 0.1, 2.0,
            active_map_pin=expected, identity_task_map_pin=expected,
            expected_task_map_pin=expected), (True, 'CONFIRMED'))
        self.assertEqual(identity_gate_decision(
            True, STATUS_CONFIRMED, 'R01C02', 'R01C02', 0.1, 2.0,
            active_map_pin=('uuid-2', 7, 'map-sha', 'manifest-sha'),
            identity_task_map_pin=expected,
            expected_task_map_pin=expected), (False, 'ACTIVE_MAP_MISMATCH'))
        self.assertEqual(identity_gate_decision(
            True, STATUS_MAP_MISMATCH, '', 'R01C02', 0.1, 2.0,
            active_map_pin=expected, identity_task_map_pin=expected,
            expected_task_map_pin=expected), (False, 'TASK_MAP_MISMATCH'))


if __name__ == '__main__':
    unittest.main()
