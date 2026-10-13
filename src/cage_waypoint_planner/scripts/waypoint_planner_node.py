#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from cage_msgs.msg import CageGeometry, CellTargetArray, MissionRequest, MissionWaypoint, MissionWaypointArray
from cage_waypoint_planner.planner import MissionPlanner, PlannerConfig
from cage_grid.model import normalize_geometry_mode


class PlannerNode:
    def __init__(self):
        self.geometry_mode = normalize_geometry_mode(rospy.get_param('/cage/geometry_mode', 'rectangular'))
        cfg = PlannerConfig(
            enter_distance=rospy.get_param('/mission/enter_distance', 20.0),
            exit_distance=rospy.get_param('/mission/exit_distance', 20.0),
            takeoff_relative_alt=rospy.get_param('/vehicle/takeoff_relative_alt', 20.0),
            cruise_relative_alt=rospy.get_param('/vehicle/cruise_relative_alt', 30.0),
            feed_relative_alt=rospy.get_param('/vehicle/feed_relative_alt', 15.0),
            acceptance_radius=rospy.get_param('/mission/waypoint_acceptance_radius', 3.0),
            speed=rospy.get_param('/mission/speed', 5.0),
            max_leg_distance=rospy.get_param('/safety/max_leg_distance', 5000.0))
        self.planner, self.gps, self.geometry = MissionPlanner(cfg), None, None
        self.pub = rospy.Publisher('/mission/waypoints', MissionWaypointArray, queue_size=1, latch=True)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.on_gps, queue_size=1)
        rospy.Subscriber('/cage/geometry', CageGeometry, self.on_geometry, queue_size=1)
        rospy.Subscriber('/mission/request', MissionRequest, self.on_request, queue_size=1)
        rospy.Subscriber('/feeding/target_cells', CellTargetArray, self.on_targets, queue_size=1)
        self.targets = []
        self.target_batch_seq = None

    def on_gps(self, msg): self.gps = msg
    def on_geometry(self, msg):
        if msg.geometry_mode and msg.geometry_mode != self.geometry_mode:
            rospy.logerr('Geometry message mode %s does not match configured mode %s', msg.geometry_mode, self.geometry_mode)
            return
        self.geometry = msg

    def on_targets(self, msg):
        if self.geometry_mode == 'circular':
            self.targets = []
            return
        if msg.cage_id == int(rospy.get_param('/cage_grid/cage_id', 0)):
            self.target_batch_seq = msg.header.seq
            self.targets = list(msg.targets)
            if rospy.get_param('/cage_grid/geometry_source','fixed') == 'beacon' and self.gps and self.targets:
                try:
                    home=(self.gps.latitude,self.gps.longitude,self.gps.altitude)
                    planned=self.planner.plan_cells(home,self.targets,
                        rospy.get_param('/feeding/transit_relative_alt',self.planner.config.cruise_relative_alt))
                    self.publish_plan(planned,'PREVIEW',msg.cage_id,msg.header)
                except ValueError as exc:
                    rospy.logerr('Preview planning rejected: %s',exc)

    def publish_plan(self, planned, mission_id, cage_id, header):
        out=MissionWaypointArray(header=header,mission_id=mission_id,cage_id=cage_id)
        out.header.stamp=rospy.Time.now()
        out.waypoints=[MissionWaypoint(seq=p.seq,lat=p.lat,lon=p.lon,alt=p.alt,yaw=p.yaw,
                         acceptance_radius=p.acceptance_radius,speed=p.speed,hold_time=p.hold_time,waypoint_type=p.waypoint_type,
                         cell_id=p.cell_id) for p in planned]
        self.pub.publish(out)
        return out

    def on_request(self, request):
        # ScoutTaskManager owns Scout route publication.  This legacy planner
        # must stay silent so two different MissionWaypointArray messages can
        # never race for the same mission_id.
        mission_scout = getattr(MissionRequest, 'MISSION_SCOUT', 1)
        if int(getattr(request, 'mission_type', 0)) == mission_scout:
            return
        if self.gps is None:
            rospy.logerr('Cannot plan: GPS is unavailable'); return
        active_cage = int(rospy.get_param('/cage_grid/cage_id', 0))
        if self.geometry_mode == 'rectangular' and not request.start:
            return  # Rectangular previews are produced by on_targets().
        if self.geometry_mode == 'circular':
            if self.geometry is None:
                rospy.logerr('Cannot plan circular mission: cage geometry is unavailable'); return
            if request.target_cage_id != self.geometry.cage_id:
                rospy.logerr('Requested cage %s is unavailable', request.target_cage_id); return
        elif self.targets and request.target_cage_id != active_cage:
            rospy.logerr('Requested cage %s is unavailable', request.target_cage_id); return
        elif not self.targets and self.geometry is None:
            rospy.logerr('Cannot plan: cage geometry is unavailable'); return
        elif not self.targets and request.target_cage_id != self.geometry.cage_id:
            rospy.logerr('Requested cage %s is unavailable', request.target_cage_id); return
        try:
            home = (self.gps.latitude, self.gps.longitude, self.gps.altitude)
            if self.geometry_mode == 'circular':
                planned = self.planner.plan_circular(home,
                    (self.geometry.endpoint_a_lat, self.geometry.endpoint_a_lon, self.geometry.endpoint_a_alt),
                    (self.geometry.endpoint_b_lat, self.geometry.endpoint_b_lon, self.geometry.endpoint_b_alt),
                    rospy.get_param('/cage/circular/feeding_dwell_sec', 5.0),
                    request.transit_relative_alt or rospy.get_param('/feeding/transit_relative_alt', self.planner.config.cruise_relative_alt),
                    rospy.get_param('/cage/circular/target_acceptance_radius_m', self.planner.config.acceptance_radius),
                    request.feed_relative_alt or rospy.get_param('/feeding/feed_relative_alt', self.planner.config.feed_relative_alt))
            elif self.targets:
                planned = self.planner.plan_cells(home, self.targets,
                    rospy.get_param('/feeding/transit_relative_alt', self.planner.config.cruise_relative_alt))
            else:
                planned = self.planner.plan(home,
                    (self.geometry.endpoint_a_lat, self.geometry.endpoint_a_lon, self.geometry.endpoint_a_alt),
                    (self.geometry.endpoint_b_lat, self.geometry.endpoint_b_lon, self.geometry.endpoint_b_alt))
        except ValueError as exc:
            rospy.logerr('Mission planning rejected: %s', exc); return
        mission_id = request.mission_id if request.start else 'PREVIEW'
        out = self.publish_plan(planned,mission_id,request.target_cage_id,request.header)
        rospy.loginfo('%s planned %d GPS waypoints for cage %d',
                      '[CIRCULAR]' if self.geometry_mode == 'circular' else '[RECTANGULAR]',
                      len(out.waypoints), out.cage_id)


if __name__ == '__main__':
    rospy.init_node('waypoint_planner_node'); PlannerNode(); rospy.spin()
