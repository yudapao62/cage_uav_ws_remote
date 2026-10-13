#!/usr/bin/env python3
import math
import rospy
from cage_msgs.msg import CageGeometry


class CageGeoNode:
    def __init__(self):
        self.pub = rospy.Publisher('/cage/geometry_validated', CageGeometry, queue_size=1, latch=True)
        rospy.Subscriber('/cage/geometry', CageGeometry, self.callback, queue_size=1)

    def callback(self, msg):
        values = [msg.endpoint_a_lat, msg.endpoint_a_lon, msg.endpoint_b_lat, msg.endpoint_b_lon]
        if not all(math.isfinite(v) for v in values):
            rospy.logerr('Rejecting cage %s: non-finite endpoint', msg.cage_id); return
        if not (-90 <= msg.endpoint_a_lat <= 90 and -90 <= msg.endpoint_b_lat <= 90 and
                -180 <= msg.endpoint_a_lon <= 180 and -180 <= msg.endpoint_b_lon <= 180):
            rospy.logerr('Rejecting cage %s: endpoint outside WGS84 bounds', msg.cage_id); return
        if msg.endpoint_a_lat == msg.endpoint_b_lat and msg.endpoint_a_lon == msg.endpoint_b_lon:
            rospy.logerr('Rejecting cage %s: identical endpoints', msg.cage_id); return
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('cage_geo_node'); CageGeoNode(); rospy.spin()
