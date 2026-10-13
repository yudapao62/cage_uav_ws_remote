#!/usr/bin/env python3
"""Explicit bench/SITL test. Arming and mode changes require opt-in parameters."""
import rospy
from cage_mavros_adapter.adapter import MavrosAdapter

if __name__ == '__main__':
    rospy.init_node('mavros_interface_test')
    adapter = MavrosAdapter()
    deadline = rospy.Time.now() + rospy.Duration(15)
    while not adapter.is_connected() and rospy.Time.now() < deadline: rospy.sleep(0.1)
    if not adapter.is_connected(): raise SystemExit('FAIL: FCU not connected')
    rospy.loginfo('PASS state: mode=%s', adapter.get_mode())
    pulled = adapter.download_mission()
    rospy.loginfo('PASS mission pull: %d items', len(pulled))
    if rospy.get_param('~test_mode_change', False):
        rospy.loginfo('set_mode result=%s', adapter.set_mode(rospy.get_param('~test_mode', 'LOITER')))
    if rospy.get_param('~test_arm', False):
        rospy.logwarn('ARM test explicitly enabled')
        rospy.loginfo('arm=%s', adapter.arm()); rospy.sleep(1); rospy.loginfo('disarm=%s', adapter.disarm())
