#!/usr/bin/env python3
"""Convert Joy axes to MAVROS RC override only while MANUAL is confirmed."""
import time

import rospy
from mavros_msgs.msg import OverrideRCIn, State
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class JoystickRCBridge:
    def __init__(self):
        prefix = '/manual_control/'
        self.axes = {
            'roll': int(rospy.get_param(prefix + 'roll_axis', 0)),
            'pitch': int(rospy.get_param(prefix + 'pitch_axis', 1)),
            'throttle': int(rospy.get_param(prefix + 'throttle_axis', 4)),
            'yaw': int(rospy.get_param(prefix + 'yaw_axis', 3)),
        }
        self.reverse = {
            name: bool(rospy.get_param(prefix + name + '_reverse', name == 'pitch'))
            for name in self.axes
        }
        self.buttons = {
            'takeover': int(rospy.get_param(prefix + 'takeover_button', 0)),
            'auto': int(rospy.get_param(prefix + 'request_auto_button', 1)),
            'land': int(rospy.get_param(prefix + 'land_button', 2)),
        }
        self.pwm_min = int(rospy.get_param(prefix + 'pwm_min', 1000))
        self.pwm_trim = int(rospy.get_param(prefix + 'pwm_trim', 1500))
        self.pwm_max = int(rospy.get_param(prefix + 'pwm_max', 2000))
        self.deadzone = float(rospy.get_param(prefix + 'deadzone', 0.08))
        self.publish_rate = float(rospy.get_param(prefix + 'publish_rate', 30.0))
        self.joy_timeout = float(rospy.get_param(prefix + 'joystick_timeout', 0.5))
        self.release_frames = int(rospy.get_param(prefix + 'release_frames', 5))
        self.manual_mode = str(rospy.get_param('/control_authority/manual_mode', 'LOITER'))

        self.owner = 'AUTO'
        self.fcu = State()
        self.joy = None
        self.last_joy = 0.0
        self.previous_buttons = []
        self.external_release = True
        self.release_remaining = self.release_frames
        self.release_announced = False
        self.joystick_timed_out = False

        self.rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
        self.takeover_pub = rospy.Publisher('/manual/takeover_request', Bool, queue_size=10)
        self.land_pub = rospy.Publisher('/manual/land_request', Bool, queue_size=10)
        self.released_pub = rospy.Publisher('/control/rc_override_released', Bool,
                                            queue_size=1, latch=True)
        rospy.Subscriber('/joy', Joy, self.on_joy, queue_size=1)
        rospy.Subscriber('/control/owner', String, self.on_owner, queue_size=1)
        rospy.Subscriber('/control/release_rc', Bool, self.on_release, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=1)
        self.auto_service = rospy.ServiceProxy('/control/request_auto', Trigger)
        self.released_pub.publish(Bool(data=False))
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.tick)

    def on_state(self, msg):
        self.fcu = msg

    def on_owner(self, msg):
        previous = self.owner
        self.owner = str(msg.data).upper()
        if previous == 'MANUAL' and self.owner != 'MANUAL':
            self.start_release()

    def on_release(self, msg):
        if msg.data:
            self.external_release = True
            self.start_release()
        else:
            self.external_release = False
            self.release_remaining = 0
            self.release_announced = False
            self.released_pub.publish(Bool(data=False))

    def start_release(self):
        self.release_remaining = max(self.release_remaining, self.release_frames)
        self.release_announced = False
        self.released_pub.publish(Bool(data=False))

    def button_rising(self, msg, name):
        index = self.buttons[name]
        current = index >= 0 and index < len(msg.buttons) and bool(msg.buttons[index])
        previous = index >= 0 and index < len(self.previous_buttons) and bool(self.previous_buttons[index])
        return current and not previous

    def on_joy(self, msg):
        if self.button_rising(msg, 'takeover'):
            self.takeover_pub.publish(Bool(data=True))
        if self.button_rising(msg, 'auto'):
            try:
                self.auto_service()
            except (rospy.ServiceException, rospy.ROSException) as exc:
                rospy.logwarn('AUTO request service failed: %s', exc)
        if self.button_rising(msg, 'land'):
            self.land_pub.publish(Bool(data=True))
        self.previous_buttons = list(msg.buttons)
        self.joy = msg
        self.last_joy = time.monotonic()
        self.joystick_timed_out = False

    def axis_value(self, name):
        index = self.axes[name]
        if self.joy is None or index < 0 or index >= len(self.joy.axes):
            return 0.0
        value = max(-1.0, min(1.0, float(self.joy.axes[index])))
        if abs(value) < self.deadzone:
            value = 0.0
        if self.reverse[name]:
            value = -value
        return value

    def to_pwm(self, value):
        if value >= 0.0:
            return int(round(self.pwm_trim + value * (self.pwm_max - self.pwm_trim)))
        return int(round(self.pwm_trim + value * (self.pwm_trim - self.pwm_min)))

    @staticmethod
    def release_message():
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_RELEASE] * 18
        return msg

    def publish_release(self):
        if not self.fcu.connected:
            return
        self.rc_pub.publish(self.release_message())
        if self.release_remaining > 0:
            self.release_remaining -= 1
        if self.release_remaining == 0 and not self.release_announced:
            self.release_announced = True
            self.released_pub.publish(Bool(data=True))

    def tick(self, _event):
        if not self.fcu.connected:
            return
        if self.external_release or self.owner != 'MANUAL':
            if self.release_remaining > 0:
                self.publish_release()
            return
        if self.fcu.mode != self.manual_mode:
            self.start_release()
            self.publish_release()
            return
        if self.joy is None or time.monotonic() - self.last_joy > self.joy_timeout:
            # Timeout never restores AUTO. Releasing pilot input leaves FCU in LOITER.
            if not self.joystick_timed_out:
                self.joystick_timed_out = True
                self.start_release()
                rospy.logwarn('Joystick timeout: RC released; owner remains MANUAL')
            if self.release_remaining > 0:
                self.publish_release()
            return

        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[0] = self.to_pwm(self.axis_value('roll'))
        msg.channels[1] = self.to_pwm(self.axis_value('pitch'))
        msg.channels[2] = self.to_pwm(self.axis_value('throttle'))
        msg.channels[3] = self.to_pwm(self.axis_value('yaw'))
        self.rc_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('joystick_rc_bridge')
    JoystickRCBridge()
    rospy.spin()
