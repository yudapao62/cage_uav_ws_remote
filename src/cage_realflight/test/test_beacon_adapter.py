import importlib.util
import json
import os
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from sensor_msgs.msg import NavSatFix
from cage_grid.model import CageGridModel


path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'beacon_gps_adapter_node.py')
spec = importlib.util.spec_from_file_location('beacon_adapter_node', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Publisher:
    def __init__(self): self.items = []
    def publish(self, msg): self.items.append(msg)


class BeaconAdapterTest(unittest.TestCase):
    def setUp(self):
        self.params = patch.object(module.rospy, 'get_param', side_effect=lambda key, default=None: default)
        self.params.start()
        self.addCleanup(self.params.stop)
        self.clock = patch.object(module.rospy.Time, 'now', return_value=module.rospy.Time(1000))
        self.clock.start()
        self.addCleanup(self.clock.stop)
        adapter = module.BeaconAdapter.__new__(module.BeaconAdapter)
        adapter.scene = 1; adapter.ids = {'A': 'alpha', 'B': 'beta'}
        adapter.timeout = 2.0; adapter.skew = 1.0; adapter.min_sat = 6
        adapter.min_baseline = 5.0; adapter.max_baseline = 500.0
        adapter.max_alt_difference = 20.0; adapter.max_jump = 5.0
        adapter.require_valid = True; adapter.filter_enabled = True
        adapter.window = 10; adapter.anchors = {}
        adapter.status_pub = Publisher(); adapter.live_pub = Publisher(); adapter.valid_pub = Publisher()
        self.adapter = adapter

    def fix(self, anchor_id, seq, lon, sat=10, valid=True, scene=1, expected_role=None):
        self.adapter.on_fix(SimpleNamespace(scene_id=scene, anchor_id=anchor_id,
            lat=22.5, lon=lon, altitude=5.0, valid=valid, satellites=sat,
            seq=seq, stamp='unparsed stamp'), expected_role)

    def state(self):
        self.adapter.tick(None)
        return json.loads(self.adapter.status_pub.items[-1].data)

    def test_pair_validation_and_common_enu(self):
        self.fix('alpha', 1, 114.0)
        self.assertEqual(self.state()['state'], 'WAITING_FOR_ANCHORS')
        self.assertEqual(len(self.adapter.valid_pub.items), 0)
        self.fix('beta', 1, 114.0007)
        self.assertTrue(self.state()['geometry_valid'])
        home = NavSatFix(latitude=22.5, longitude=114.0, altitude=5.0)
        model = CageGridModel.from_beacon(self.adapter.valid_pub.items[-1], home, 6, 10, 12.0, 12.0)
        self.assertAlmostEqual(model.origin_x, 0.0, delta=0.1)
        self.assertAlmostEqual(model.origin_y, 0.0, delta=0.1)

    def test_invalid_satellite_duplicate_jump_and_reset(self):
        self.fix('alpha', 100, 114.0); self.fix('beta', 100, 114.0007)
        self.assertTrue(self.state()['geometry_valid'])
        received = self.adapter.anchors[(1, 'alpha')]['received']
        self.fix('alpha', 100, 114.0)
        self.assertEqual(self.adapter.anchors[(1, 'alpha')]['received'], received)
        self.fix('beta', 101, 114.0007, sat=3)
        self.assertEqual(self.state()['state'], 'LOW_SATELLITES')
        self.fix('beta', 102, 114.0007)
        self.fix('alpha', 101, 114.001)
        self.assertEqual(self.state()['state'], 'ANCHOR_INVALID')
        entry = self.adapter.anchors[(1, 'alpha')]
        entry['error'] = ''; entry['received'] -= 10.0
        self.assertEqual(self.state()['state'], 'ANCHOR_STALE')
        self.fix('alpha', 0, 114.0)
        self.assertEqual(self.adapter.anchors[(1, 'alpha')]['seq'], 0)

    def test_scene_filter_and_invalid_flag(self):
        self.fix('alpha', 1, 114.0, scene=2)
        self.assertEqual(len(self.adapter.anchors), 0)
        self.fix('alpha', 1, 114.0)
        self.fix('beta', 1, 114.0007, valid=False)
        self.assertEqual(self.state()['state'], 'ANCHOR_INVALID')
        self.assertEqual(len(self.adapter.valid_pub.items), 0)

    def test_rejects_anchor_on_wrong_dedicated_topic(self):
        self.fix('alpha', 1, 114.0, expected_role='B')
        self.assertEqual(len(self.adapter.anchors), 0)
        self.fix('alpha', 1, 114.0, expected_role='A')
        self.assertIn((1, 'alpha'), self.adapter.anchors)


if __name__ == '__main__': unittest.main()
