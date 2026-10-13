#!/usr/bin/env python3
"""Deprecated compatibility trigger for the request-driven Scout task.

This node intentionally has no MAVROS imports, services, or ground-truth
dependency. Existing CI launch files may still invoke its executable name;
when ``~auto_start`` is explicitly true it publishes exactly one normal Scout
request. Production launch files use ``scout_task_manager_node.py`` directly
and leave automatic start disabled.
"""
import time

import rospy
from std_msgs.msg import Header

from cage_msgs.msg import ScoutTaskRequest


class ScoutRouteCommanderCompatibility:
    def __init__(self):
        self.auto_start = bool(rospy.get_param('~auto_start', False))
        self.delay = max(0.0, float(rospy.get_param('~start_delay_sec', 3.0)))
        self.started_at = time.monotonic()
        self.sent = False
        self.publisher = rospy.Publisher(
            '/scout/task_request', ScoutTaskRequest, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.tick)

    def tick(self, _event):
        if (self.sent or not self.auto_start or
                time.monotonic() - self.started_at < self.delay):
            return
        self.sent = True
        task_id = str(rospy.get_param(
            '~task_id', 'scout-ci-{}'.format(int(time.time()))))
        self.publisher.publish(ScoutTaskRequest(
            header=Header(stamp=rospy.Time.now()),
            task_id=task_id,
            cage_id=int(rospy.get_param('~cage_id', 0)),
            survey_relative_alt=float(rospy.get_param('~altitude_m', 25.0)),
            auto_rtl=True,
            auto_land=True))
        rospy.logwarn(
            'Deprecated scout_route_commander published request %s; '
            'use the Web Scout API instead', task_id)


if __name__ == '__main__':
    rospy.init_node('scout_route_commander')
    ScoutRouteCommanderCompatibility()
    rospy.spin()
