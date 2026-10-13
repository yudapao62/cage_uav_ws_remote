#!/usr/bin/env python3
"""Display beacon anchors in the same home ENU frame as the grid."""
import json
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from cage_msgs.msg import CageGeometry
from cage_geo.geodesy import LocalENU


class Monitor:
    def __init__(self):
        self.home=None; self.geometry=None; self.status={}
        self.pub=rospy.Publisher('/cage/beacon/markers',MarkerArray,queue_size=1)
        rospy.Subscriber('/cage/home',NavSatFix,lambda m:setattr(self,'home',m),queue_size=1)
        rospy.Subscriber('/cage/geometry_validated',CageGeometry,lambda m:setattr(self,'geometry',m),queue_size=1)
        rospy.Subscriber('/cage/beacon/status',String,self.on_status,queue_size=1)
        rospy.Timer(rospy.Duration(0.5),self.tick)

    def on_status(self,msg):
        try: self.status=json.loads(msg.data)
        except (ValueError,TypeError): self.status={}

    def tick(self,_):
        markers=MarkerArray()
        markers.markers.append(Marker(header=Header(stamp=rospy.Time.now(),frame_id='map'),ns='beacon',id=0,action=Marker.DELETEALL))
        if self.home and self.geometry:
            local=LocalENU(self.home.latitude,self.home.longitude,self.home.altitude)
            for i,role in enumerate(('A','B')):
                g=self.geometry
                lat,lon,alt=(g.endpoint_a_lat,g.endpoint_a_lon,g.endpoint_a_alt) if role=='A' else (g.endpoint_b_lat,g.endpoint_b_lon,g.endpoint_b_alt)
                x,y,_=local.to_enu(lat,lon,alt)
                info=self.status.get('anchors',{}).get(role,{})
                m=Marker(header=Header(stamp=rospy.Time.now(),frame_id='map'),ns='beacon',id=i+1,
                    type=Marker.TEXT_VIEW_FACING,action=Marker.ADD)
                m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=2.0
                m.pose.orientation.w=1.0; m.scale.z=1.0
                m.color.r=0.2 if info.get('valid') else 1.0; m.color.g=1.0 if info.get('valid') else 0.2
                m.color.b=0.2; m.color.a=1.0
                m.text='ANCHOR_{} | SAT {}'.format(role,info.get('satellites','?'))
                markers.markers.append(m)
        self.pub.publish(markers)


if __name__=='__main__':
    rospy.init_node('realflight_status_monitor')
    Monitor()
    rospy.spin()
