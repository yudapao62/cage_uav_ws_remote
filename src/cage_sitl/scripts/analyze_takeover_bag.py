#!/usr/bin/env python3
"""Report takeover sequencing and observed response from a ROS bag."""
import math
import sys

import rosbag


def stamp_seconds(stamp):
    return stamp.to_sec()


def first_after(events, start):
    return next((value for value in events if value >= start), None)


def delta(label, start, end):
    if start is None or end is None:
        print('{}: unavailable'.format(label))
    else:
        print('{}: {:.3f} s'.format(label, end - start))


def main(path):
    takeover = []
    manual_owner = []
    manual_mode = []
    manual_rc = []
    positions = []
    with rosbag.Bag(path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[
                '/manual/takeover_request', '/control/owner', '/mavros/state',
                '/mavros/rc/override', '/mavros/local_position/pose']):
            when = stamp_seconds(stamp)
            if topic == '/manual/takeover_request' and msg.data:
                takeover.append(when)
            elif topic == '/control/owner' and msg.data == 'MANUAL':
                manual_owner.append(when)
            elif topic == '/mavros/state' and msg.mode == 'LOITER':
                manual_mode.append(when)
            elif topic == '/mavros/rc/override':
                values = list(msg.channels[:4])
                if any(value not in (0, 1500, 65535) for value in values):
                    manual_rc.append(when)
            elif topic == '/mavros/local_position/pose':
                p = msg.pose.position
                positions.append((when, p.x, p.y, p.z))

    t0 = takeover[0] if takeover else None
    t1 = first_after(manual_owner, t0) if t0 is not None else None
    t2 = first_after(manual_mode, t1) if t1 is not None else None
    t3 = first_after(manual_rc, t2) if t2 is not None else None
    t4 = None
    if t3 is not None:
        baseline = next(((x, y, z) for when, x, y, z in positions if when >= t3), None)
        if baseline:
            for when, x, y, z in positions:
                if when >= t3 and math.sqrt((x-baseline[0])**2 + (y-baseline[1])**2 +
                                             (z-baseline[2])**2) >= 0.10:
                    t4 = when
                    break
    print('t0 takeover_request: {}'.format(t0))
    print('t1 owner_MANUAL: {}'.format(t1))
    print('t2 LOITER_confirmed: {}'.format(t2))
    print('t3 first_manual_RC: {}'.format(t3))
    print('t4 first_0.10m_response: {}'.format(t4))
    delta('takeover request -> owner MANUAL', t0, t1)
    delta('owner MANUAL -> mode LOITER', t1, t2)
    delta('manual input -> UAV movement', t3, t4)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit('usage: analyze_takeover_bag.py BAG')
    main(sys.argv[1])
