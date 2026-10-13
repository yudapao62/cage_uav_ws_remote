#!/usr/bin/env python3
import json
import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from cage_msgs.msg import MissionState

seen = {'connected':False, 'gps':False, 'states':set()}
def on_fcu(m): seen['connected'] |= m.connected
def on_gps(m): seen['gps'] |= (m.latitude != 0 or m.longitude != 0)
def on_state(m): seen['states'].add(m.state)
if __name__ == '__main__':
    rospy.init_node('simulation_acceptance')
    rospy.Subscriber('/mavros/state', State, on_fcu); rospy.Subscriber('/mavros/global_position/global', NavSatFix, on_gps)
    rospy.Subscriber('/mission/state', MissionState, on_state)
    deadline = rospy.Time.now()+rospy.Duration(rospy.get_param('~timeout',900))
    rate=rospy.Rate(2)
    while not rospy.is_shutdown() and rospy.Time.now()<deadline and 'FINISHED' not in seen['states'] and not ({'ERROR','FAILSAFE','MANUAL_OVERRIDE'} & seen['states']): rate.sleep()
    mode=rospy.get_param('/cage/geometry_mode','rectangular')
    feed_state='FEEDING_DWELL' if mode=='circular' else 'FEEDING'
    common={'WAIT_FCU','WAIT_GPS','IDLE','PLAN','UPLOAD_MISSION','WAIT_ARM','TAKEOFF','TRANSIT',feed_state,'RTL','LAND','FINISHED'}
    legacy_extra={'EXIT'}
    grid_ok = common <= seen['states']
    legacy_ok = common | legacy_extra <= seen['states']
    missing=sorted((common | legacy_extra)-seen['states']) if not grid_ok else []
    rospy.loginfo('acceptance connected=%s gps=%s states=%s missing=%s',seen['connected'],seen['gps'],sorted(seen['states']),missing)
    raise SystemExit(0 if seen['connected'] and seen['gps'] and (grid_ok or legacy_ok) else 1)
