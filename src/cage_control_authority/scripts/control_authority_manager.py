#!/usr/bin/env python3
"""Arbitrate AUTO/MANUAL ownership; MANUAL always wins and is latched."""
import time

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


class ControlAuthorityManager:
    AUTO = 'AUTO'
    MANUAL = 'MANUAL'

    def __init__(self):
        self.auto_mode = str(rospy.get_param('/control_authority/auto_mode', 'GUIDED'))
        self.manual_mode = str(rospy.get_param('/control_authority/manual_mode', 'LOITER'))
        self.land_mode = str(rospy.get_param('/control_authority/land_mode', 'LAND'))
        self.request_interval = float(rospy.get_param('/control_authority/mode_request_interval', 1.0))
        self.release_delay = float(rospy.get_param('/control_authority/release_confirm_delay', 0.25))
        self.owner = str(rospy.get_param('/control_authority/initial_owner', self.AUTO)).upper()
        if self.owner not in (self.AUTO, self.MANUAL):
            raise ValueError('invalid /control_authority/initial_owner: {}'.format(self.owner))

        self.fcu = State()
        self.pending_manual = False
        self.pending_auto = False
        self.pending_land = False
        self.rc_released = False
        self.release_requested_at = 0.0
        self.last_mode_request = 0.0

        self.owner_pub = rospy.Publisher('/control/owner', String, queue_size=1, latch=True)
        self.release_pub = rospy.Publisher('/control/release_rc', Bool, queue_size=1, latch=True)
        rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=1)
        rospy.Subscriber('/manual/takeover_request', Bool, self.on_takeover, queue_size=10)
        rospy.Subscriber('/manual/land_request', Bool, self.on_land, queue_size=10)
        rospy.Subscriber('/control/rc_override_released', Bool, self.on_rc_released, queue_size=1)
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.Service('/control/request_auto', Trigger, self.on_request_auto)
        self.owner_pub.publish(String(data=self.owner))
        self.release_pub.publish(Bool(data=self.owner == self.AUTO))
        rospy.Timer(rospy.Duration(0.1), self.tick)

    def set_owner(self, owner):
        if owner != self.owner:
            rospy.logwarn('Control authority %s -> %s', self.owner, owner)
            self.owner = owner
        self.owner_pub.publish(String(data=self.owner))

    def on_state(self, msg):
        self.fcu = msg

    def on_rc_released(self, msg):
        self.rc_released = bool(msg.data)

    def on_takeover(self, msg):
        if not msg.data:
            return
        # Logical ownership changes before any potentially slow FCU mode request.
        self.pending_auto = False
        self.pending_land = False
        self.pending_manual = True
        self.rc_released = False
        self.release_pub.publish(Bool(data=False))
        self.set_owner(self.MANUAL)
        rospy.logwarn('Manual takeover latched; requesting %s', self.manual_mode)

    def on_land(self, msg):
        if msg.data and self.owner == self.MANUAL:
            self.pending_auto = False
            self.pending_manual = False
            self.pending_land = True
            rospy.logwarn('Manual LAND requested; authority remains MANUAL')

    def on_request_auto(self, _request):
        if self.owner == self.AUTO and not self.pending_auto:
            return TriggerResponse(success=True, message='AUTO already owns control')
        if self.pending_auto:
            return TriggerResponse(success=True, message='AUTO recovery already pending')
        self.pending_manual = False
        self.pending_land = False
        self.pending_auto = True
        self.rc_released = False
        self.release_requested_at = time.monotonic()
        self.release_pub.publish(Bool(data=True))
        rospy.logwarn('Explicit AUTO request accepted; releasing RC before %s', self.auto_mode)
        return TriggerResponse(success=True, message='RC release requested; waiting for mode confirmation')

    def request_mode(self, mode):
        now = time.monotonic()
        if not self.fcu.connected or now - self.last_mode_request < self.request_interval:
            return False
        self.last_mode_request = now
        try:
            response = self.mode_service(base_mode=0, custom_mode=mode)
            if not response.mode_sent:
                rospy.logwarn('FCU rejected mode request %s', mode)
            return bool(response.mode_sent)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn_throttle(2.0, 'Mode request %s failed: %s', mode, exc)
            return False

    def tick(self, _event):
        if not self.fcu.connected:
            return

        if self.pending_auto:
            if not self.rc_released:
                return
            if time.monotonic() - self.release_requested_at < self.release_delay:
                return
            if self.fcu.mode == self.auto_mode:
                self.pending_auto = False
                self.set_owner(self.AUTO)
                rospy.logwarn('%s confirmed; AUTO authority enabled in IDLE/HOLD', self.auto_mode)
            else:
                self.request_mode(self.auto_mode)
            return

        if self.owner != self.MANUAL:
            return
        if self.pending_land:
            if self.fcu.mode == self.land_mode:
                self.pending_land = False
            else:
                self.request_mode(self.land_mode)
            return
        if self.pending_manual:
            if self.fcu.mode == self.manual_mode:
                self.pending_manual = False
                rospy.logwarn('%s confirmed; manual RC output is now permitted', self.manual_mode)
            else:
                self.request_mode(self.manual_mode)


if __name__ == '__main__':
    rospy.init_node('control_authority_manager')
    ControlAuthorityManager()
    rospy.spin()
