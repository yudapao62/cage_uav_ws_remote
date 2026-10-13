#!/usr/bin/env python3
"""Transport-free contract tests for the Scout Web API gate."""
import os
import sys
import threading
import unittest
from types import SimpleNamespace


SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from web_bridge_node import ApiError, WebBridge  # noqa: E402

# These tests exercise the request gate without starting a ROS node.  Enable
# wall-clock Time.now() so constructing the outbound message remains faithful
# to production while no ROS master is required.
import rospy  # noqa: E402
rospy.rostime.set_rostime_initialized(True)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def request(task_id='scout-web-test', cage_id=0, auto_land=True):
    return {
        'task_id': task_id,
        'cage_id': cage_id,
        'survey_relative_alt': 25.0,
        'auto_rtl': True,
        'auto_land': auto_land,
    }


def bridge_stub():
    bridge = WebBridge.__new__(WebBridge)
    bridge._lock = threading.RLock()
    bridge.targetable_cages = frozenset((0,))
    bridge.minimum_scout_alt = 1.0
    bridge.maximum_scout_alt = 120.0
    bridge._pending_scout = None
    bridge._scout_cancel_requested_for = ''
    bridge._used_scout_task_ids = set()
    bridge.scout_status = SimpleNamespace(state='IDLE', task_id='', cage_id=0)
    bridge.status = SimpleNamespace(task_state='TASK_IDLE')
    bridge.mission = SimpleNamespace(state='IDLE')
    bridge.scout_start_pub = FakePublisher()
    bridge.scout_cancel_pub = FakePublisher()
    return bridge


class ScoutWebApiTest(unittest.TestCase):
    def test_only_cage_zero_is_targetable(self):
        bridge = bridge_stub()
        with self.assertRaises(ApiError) as caught:
            bridge.validate_scout_payload(request(cage_id=1))
        self.assertEqual(caught.exception.status, 409)
        self.assertEqual(caught.exception.code, 'CAGE_NOT_TARGETABLE')

    def test_auto_land_is_mandatory(self):
        bridge = bridge_stub()
        with self.assertRaises(ApiError) as caught:
            bridge.validate_scout_payload(request(auto_land=False))
        self.assertEqual(caught.exception.status, 400)
        self.assertEqual(caught.exception.code, 'AUTO_LAND_REQUIRED')

    def test_same_id_is_idempotent_but_different_task_is_busy(self):
        bridge = bridge_stub()
        first = bridge.start_scout(request('same-id'))
        retry = bridge.start_scout(request('same-id'))
        self.assertTrue(first['accepted'])
        self.assertFalse(first['idempotent'])
        self.assertTrue(retry['idempotent'])
        self.assertEqual(len(bridge.scout_start_pub.messages), 1)
        with self.assertRaises(ApiError) as caught:
            bridge.start_scout(request('different-id'))
        self.assertEqual(caught.exception.status, 409)
        self.assertEqual(caught.exception.code, 'SCOUT_TASK_BUSY')

    def test_cancel_is_idempotent_and_publishes_task_id_once(self):
        bridge = bridge_stub()
        bridge.start_scout(request('cancel-id'))
        first = bridge.cancel_scout({'task_id': 'cancel-id'})
        retry = bridge.cancel_scout({'task_id': 'cancel-id'})
        self.assertFalse(first['idempotent'])
        self.assertTrue(retry['idempotent'])
        self.assertEqual(len(bridge.scout_cancel_pub.messages), 1)
        self.assertEqual(bridge.scout_cancel_pub.messages[0].data, 'cancel-id')


if __name__ == '__main__':
    unittest.main()
