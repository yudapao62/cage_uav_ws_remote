#!/usr/bin/env python3
"""Validate the existing beacon message and publish cage geometry.

The message class is resolved from the configured ROS type. No replacement
beacon message is defined here; an unknown type leaves the system unready.
"""
import json
import math
import statistics
from collections import deque

import rospy
from roslib.message import get_message_class
from std_msgs.msg import Header, String
from cage_msgs.msg import CageGeometry
from cage_geo.geodesy import LocalENU


class BeaconAdapter:
    def __init__(self):
        p = lambda key, default: rospy.get_param('/beacon/' + key, default)
        self.topics = {
            'A': p('input_topic_a', ''),
            'B': p('input_topic_b', ''),
        }
        self.msg_type = p('input_msg_type', '')
        self.scene = int(p('active_scene_id', 1))
        self.ids = {'A': p('anchor_a_id', ''), 'B': p('anchor_b_id', '')}
        self.timeout = float(p('receive_timeout_sec', 2.0))
        self.skew = float(p('max_pair_time_skew_sec', 1.0))
        self.min_sat = int(p('min_satellites', 6))
        self.min_baseline = float(p('min_baseline_m', 5.0))
        self.max_baseline = float(p('max_baseline_m', 500.0))
        self.max_alt_difference = float(p('max_altitude_difference_m', 20.0))
        self.max_jump = float(p('max_anchor_jump_m', 5.0))
        self.require_valid = bool(p('require_valid_flag', True))
        self.filter_enabled = bool(p('filtering/enabled', True))
        self.window = int(p('filtering/sample_window', 10))
        if p('filtering/method', 'median') != 'median' or self.window < 1:
            raise ValueError('Beacon filtering requires median and sample_window >= 1')
        if not self.ids['A'] or not self.ids['B'] or self.ids['A'] == self.ids['B']:
            rospy.logerr('Configure distinct, actual anchor_a_id and anchor_b_id')
        self.anchors = {}
        self.status_pub = rospy.Publisher('/cage/beacon/status', String, queue_size=1)
        self.live_pub = rospy.Publisher('/cage/geometry_live', CageGeometry, queue_size=1)
        self.valid_pub = rospy.Publisher('/cage/geometry_validated', CageGeometry, queue_size=1)
        self.subscribers = []
        topics_valid = all(topic.startswith('/') and not topic.startswith('/TODO')
                           for topic in self.topics.values())
        if (topics_valid and
                not self.msg_type.startswith('TODO') and all(self.ids.values()) and
                self.ids['A'] != self.ids['B'] and all(not name.startswith('TODO') for name in self.ids.values())):
            cls = get_message_class(self.msg_type)
            expected = {'scene_id', 'anchor_id', 'lat', 'lon', 'altitude', 'valid', 'satellites', 'seq', 'stamp'}
            if cls is None:
                rospy.logerr('Beacon type %s is not installed; adapter remains unready', self.msg_type)
            elif not expected.issubset(set(cls.__slots__)):
                rospy.logerr('Beacon type %s lacks required fields; adapter remains unready', self.msg_type)
            else:
                for role in ('A', 'B'):
                    self.subscribers.append(rospy.Subscriber(
                        self.topics[role], cls, self.on_fix, callback_args=role, queue_size=20))
                    rospy.loginfo('Beacon anchor %s (%s) <- %s [%s]',
                                  role, self.ids[role], self.topics[role], self.msg_type)
        else:
            rospy.logerr('Beacon A/B topics, type, or anchor IDs are not configured; no geometry will be published')
        rospy.Timer(rospy.Duration(0.2), self.tick)

    def on_fix(self, msg, expected_role=None):
        if int(msg.scene_id) != self.scene:
            rospy.logwarn_throttle(10, 'Ignoring beacon from non-active scene')
            return
        role = next((r for r, v in self.ids.items() if v == msg.anchor_id), None)
        if role is None:
            rospy.logwarn_throttle(10, 'Ignoring unconfigured beacon anchor_id')
            return
        if expected_role is not None and role != expected_role:
            rospy.logwarn_throttle(10, 'Ignoring anchor_id received on the wrong beacon topic')
            return
        now = rospy.Time.now().to_sec()
        key = (self.scene, msg.anchor_id)
        entry = self.anchors.get(key)
        if entry and now - entry['received'] <= self.timeout:
            if int(msg.seq) <= entry['seq']:
                rospy.logwarn_throttle(10, 'Duplicate or out-of-order beacon seq')
                return
        values = (float(msg.lat), float(msg.lon), float(msg.altitude))
        reason = ''
        if self.require_valid and not msg.valid:
            reason = 'ANCHOR_INVALID'
        elif int(msg.satellites) < self.min_sat:
            reason = 'LOW_SATELLITES'
        elif not all(math.isfinite(v) for v in values) or not (-90 <= values[0] <= 90 and -180 <= values[1] <= 180) or values[:2] == (0.0, 0.0):
            reason = 'ANCHOR_INVALID'
        if entry is None or now - entry['received'] > self.timeout:
            entry = {'samples': deque(maxlen=self.window), 'received': 0.0, 'seq': -1}
            self.anchors[key] = entry
        entry['latest_seq'] = int(msg.seq)
        entry['latest_satellites'] = int(msg.satellites)
        entry['raw_stamp'] = str(msg.stamp)
        entry['latest_valid'] = bool(msg.valid)
        entry['error'] = reason
        if reason:
            return  # Keep the last good sample until it becomes stale.
        if entry['samples']:
            previous = self.filtered(entry)
            local = LocalENU(*previous)
            east, north, _ = local.to_enu(*values)
            if math.hypot(east, north) > self.max_jump:
                entry['error'] = 'ANCHOR_INVALID'
                rospy.logwarn_throttle(10, 'Beacon jump rejected')
                return
        entry['samples'].append(values)
        entry['received'] = now
        entry['seq'] = int(msg.seq)

    def filtered(self, entry):
        samples = entry['samples'] if self.filter_enabled else [entry['samples'][-1]]
        return tuple(statistics.median(row[i] for row in samples) for i in range(3))

    def tick(self, _event):
        now = rospy.Time.now().to_sec()
        entries = {role: self.anchors.get((self.scene, name)) for role, name in self.ids.items()}
        report = {'scene_id': self.scene, 'state': 'WAITING_FOR_ANCHORS', 'geometry_valid': False,
                  'baseline_m': None, 'published_at': now, 'anchors': {}}
        for role, name in self.ids.items():
            e = entries[role]
            age = now - e['received'] if e and e['received'] else None
            filtered = self.filtered(e) if e and e.get('samples') else (None, None, None)
            report['anchors'][role] = {'id': name, 'valid': bool(e and not e.get('error') and age is not None and age <= self.timeout),
                'satellites': e.get('latest_satellites') if e else None, 'age_sec': age,
                'seq': e.get('latest_seq') if e else None, 'raw_stamp': e.get('raw_stamp') if e else None,
                'lat': filtered[0], 'lon': filtered[1], 'alt': filtered[2]}
        if not entries['A'] or not entries['B']:
            report['state'] = 'WAITING_FOR_ANCHORS'
            report['missing_anchor'] = 'A' if not entries['A'] else 'B'
        elif any(e.get('error') for e in entries.values()):
            report['state'] = 'LOW_SATELLITES' if any(e.get('error') == 'LOW_SATELLITES' for e in entries.values()) else 'ANCHOR_INVALID'
        elif any(now - e['received'] > self.timeout for e in entries.values()):
            report['state'] = 'ANCHOR_STALE'
        elif abs(entries['A']['received'] - entries['B']['received']) >= self.skew:
            report['state'] = 'PAIR_TIME_SKEW'
        else:
            a, b = self.filtered(entries['A']), self.filtered(entries['B'])
            local = LocalENU(*a)
            east, north, _ = local.to_enu(*b)
            baseline = math.hypot(east, north)
            report['baseline_m'] = baseline
            geometry = CageGeometry(header=Header(stamp=rospy.Time.now()),
                cage_id=int(rospy.get_param('/cage_grid/cage_id', 0)),
                geometry_mode=str(rospy.get_param('/cage/geometry_mode', 'rectangular')).lower(),
                endpoint_a_lat=a[0], endpoint_a_lon=a[1], endpoint_a_alt=a[2],
                endpoint_b_lat=b[0], endpoint_b_lon=b[1], endpoint_b_alt=b[2],
                width=float(rospy.get_param('/cage_grid/rows', 6))*float(rospy.get_param('/cage_grid/cell_size_y', 12.0)),
                rows=int(rospy.get_param('/cage_grid/rows', 6)), cols=int(rospy.get_param('/cage_grid/cols', 10)))
            self.live_pub.publish(geometry)
            if not (self.min_baseline <= baseline <= self.max_baseline) or abs(a[2]-b[2]) >= self.max_alt_difference:
                report['state'] = 'BASELINE_INVALID' if not (self.min_baseline <= baseline <= self.max_baseline) else 'GEOMETRY_INVALID'
            else:
                report['state'] = 'GEOMETRY_VALID'
                report['geometry_valid'] = True
                self.valid_pub.publish(geometry)
        self.status_pub.publish(String(data=json.dumps(report, allow_nan=False)))


if __name__ == '__main__':
    rospy.init_node('beacon_gps_adapter_node')
    BeaconAdapter()
    rospy.spin()
