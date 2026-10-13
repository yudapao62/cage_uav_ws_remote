#!/usr/bin/env python3
import rospy
from cage_mavros_adapter.adapter import MavrosAdapter

if __name__ == '__main__':
    rospy.init_node('mavros_adapter_node')
    adapter = MavrosAdapter(rospy.get_param('~namespace', '/mavros'))
    rospy.loginfo('MAVROS adapter ready; compatibility assumptions are isolated here')
    rospy.spin()
