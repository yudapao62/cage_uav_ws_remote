#!/usr/bin/env python3
"""Publish a compact, color-coded mission status panel as an RViz text marker."""
import rospy
from cage_msgs.msg import FeedingTaskStatus, MissionState, MissionWaypointArray
from cage_visualization.markers import color, point, text_marker
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker


class MissionStatusVisualizer:
    NS = 'mission_status'

    def __init__(self):
        self.frame_id = rospy.get_param('/visualization/frame_id', 'map')
        self.position = rospy.get_param('/visualization/status_position', [15.0, -15.0, 10.0])
        self.text_height = rospy.get_param('/visualization/text_height', 1.2)
        self.enabled = rospy.get_param('/visualization/show/mission_status', True)
        self.target_cage = rospy.get_param('/cage/cage_id', 0)
        self.mission = MissionState(state='WAIT_FCU', current_waypoint=-1)
        self.fcu = State(connected=False, mode='UNKNOWN')
        self.feeder_enabled = False
        self.control_owner = 'AUTO'
        self.last_valid_gps = rospy.Time(0)
        self.gps_timeout = rospy.get_param('/safety/gps_timeout', 2.0)
        self.waypoint_count = 0
        self.task = FeedingTaskStatus(task_state='TASK_IDLE')
        self.publisher = rospy.Publisher('/visualization/mission_status', Marker,
                                         queue_size=1, latch=True)
        rospy.Subscriber('/mission/state', MissionState, self.on_mission, queue_size=1)
        rospy.Subscriber('/mission/waypoints', MissionWaypointArray,
                         self.on_waypoints, queue_size=1)
        rospy.Subscriber('/feeding/task_status', FeedingTaskStatus,
                         self.on_task, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.on_fcu, queue_size=1)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix,
                         self.on_gps, queue_size=1)
        rospy.Subscriber('/feeder/state', Bool, self.on_feeder, queue_size=1)
        rospy.Subscriber('/control/owner', String, self.on_control_owner, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self.on_timer)

    def on_mission(self, msg):
        self.mission = msg

    def on_waypoints(self, msg):
        self.waypoint_count = len(msg.waypoints)
        self.target_cage = msg.cage_id

    def on_task(self, msg):
        self.task = msg

    def on_fcu(self, msg):
        self.fcu = msg

    def on_gps(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.last_valid_gps = rospy.Time.now()

    def on_feeder(self, msg):
        self.feeder_enabled = msg.data

    def on_control_owner(self, msg):
        self.control_owner = msg.data

    def state_color(self):
        if self.mission.state in {'FAILSAFE', 'ERROR'}:
            return color(1.0, 0.12, 0.12, 1.0)
        if self.mission.state == 'MANUAL_OVERRIDE':
            return color(1.0, 0.55, 0.10, 1.0)
        if self.mission.state in {'IDLE', 'WAIT_FCU', 'WAIT_GPS'}:
            return color(0.70, 0.74, 0.78, 1.0)
        return color(0.20, 1.0, 0.35, 1.0)

    def on_timer(self, _event):
        if not self.enabled:
            return
        if self.mission.state == 'FINISHED':
            displayed_wp = self.waypoint_count
        else:
            displayed_wp = min(max(self.mission.current_waypoint + 2, 1),
                               max(self.waypoint_count, 1))
        panel = (
            'TASK MODE     {task_mode}\n'
            'TASK ID       {task_id}\n'
            'TASK STATE    {task_state}\n'
            'REPEAT        {repeat_now} / {repeat_total}\n'
            'PROGRESS      {done} / {total}\n'
            'CURRENT CELL  {cell}\n'
            'CONTROL       {owner}\n'
            'FCU MODE      {mode}\n'
            'FSM           {fsm}\n'
            'TARGET        CAGE_{cage:02d}\n'
            'WP            {wp} / {count}\n'
            'GPS           {gps}\n'
            'FCU           {fcu}\n'
            'FEEDER        {feeder}\n'
            'RC_OVERRIDE   {override}'
        ).format(
            task_mode=self.task.mode or '-', task_id=self.task.task_id or '-',
            task_state=self.task.task_state or '-', repeat_now=self.task.current_repeat,
            repeat_total=self.task.total_repeat, done=self.task.completed_targets,
            total=self.task.total_targets, cell=self.task.current_cell_id or '-',
            owner=self.control_owner or 'UNKNOWN',
            mode=self.fcu.mode or 'UNKNOWN', fsm=self.mission.state or 'UNKNOWN',
            cage=self.target_cage, wp=displayed_wp, count=self.waypoint_count,
            gps='OK' if self.last_valid_gps != rospy.Time(0) and
                (rospy.Time.now() - self.last_valid_gps).to_sec() <= self.gps_timeout
                else 'NO FIX',
            fcu='CONNECTED' if self.fcu.connected else 'DISCONNECTED',
            feeder='ON' if self.feeder_enabled else 'OFF',
            override='TRUE' if self.mission.manual_override else 'FALSE')
        output = text_marker(self.frame_id, self.NS, 0, rospy.Time.now(),
            point(*self.position), panel, self.text_height, self.state_color())
        self.publisher.publish(output)


if __name__ == '__main__':
    rospy.init_node('mission_status_visualizer')
    MissionStatusVisualizer()
    rospy.spin()
