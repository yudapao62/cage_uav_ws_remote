#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from cage_msgs.msg import CageGeometry, MissionRequest, MissionState

if __name__ == '__main__':
    rospy.init_node('publish_demo_mission')
    request_pub = rospy.Publisher('/mission/request', MissionRequest, queue_size=1, latch=True)
    deadline = rospy.Time.now() + rospy.Duration(60)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        try:
            if rospy.wait_for_message('/mission/state', MissionState, timeout=2.0).state == 'IDLE': break
        except rospy.ROSException: pass
    else: raise SystemExit('FSM did not become IDLE within 60 seconds')
    c = rospy.wait_for_message('/cage/geometry', CageGeometry, timeout=10.0)
    request_pub.publish(MissionRequest(header=Header(stamp=rospy.Time.now()), mission_id='simulation_demo',
                        target_cage_id=c.cage_id, start=True, resume_after_override=False,
                        rtl_after_mission=True,
                        feed_relative_alt=rospy.get_param('/feeding/feed_relative_alt',15.0),
                        transit_relative_alt=rospy.get_param('/feeding/transit_relative_alt',25.0)))
    rospy.loginfo('Published %s cage mission request', c.geometry_mode)
    rospy.sleep(1.0)
