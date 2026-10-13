#!/usr/bin/env python3
"""Terminal keyboard source that publishes sensor_msgs/Joy."""
import select
import sys
import termios
import time
import tty

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


class KeyboardJoyNode:
    HELP = 'W/S pitch  A/D roll  Q/E yaw  arrows throttle  M takeover  G AUTO  SPACE land'

    def __init__(self):
        self.roll_axis = int(rospy.get_param('/manual_control/roll_axis', 0))
        self.pitch_axis = int(rospy.get_param('/manual_control/pitch_axis', 1))
        self.throttle_axis = int(rospy.get_param('/manual_control/throttle_axis', 4))
        self.yaw_axis = int(rospy.get_param('/manual_control/yaw_axis', 3))
        self.takeover_button = int(rospy.get_param('/manual_control/takeover_button', 0))
        self.auto_button = int(rospy.get_param('/manual_control/request_auto_button', 1))
        self.land_button = int(rospy.get_param('/manual_control/land_button', 2))
        self.axis_count = max(8, self.roll_axis + 1, self.pitch_axis + 1,
                              self.throttle_axis + 1, self.yaw_axis + 1)
        self.button_count = max(12, self.takeover_button + 1,
                                self.auto_button + 1, self.land_button + 1)
        self.axes = [0.0] * self.axis_count
        self.buttons = [0] * self.button_count
        self.throttle = 0.0
        self.motion_until = {}
        self.pub = rospy.Publisher('/joy', Joy, queue_size=1)
        self.fd = sys.stdin.fileno()
        self.terminal_settings = None
        if sys.stdin.isatty():
            self.terminal_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            rospy.on_shutdown(self.restore_terminal)
        else:
            rospy.logwarn('keyboard_joy_node has no TTY; it will publish centered controls only')
        rospy.loginfo(self.HELP)
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.tick)

    def restore_terminal(self):
        if self.terminal_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.terminal_settings)

    def read_key(self):
        if self.terminal_settings is None or not select.select([sys.stdin], [], [], 0.0)[0]:
            return ''
        key = sys.stdin.read(1)
        if key == '\x1b' and select.select([sys.stdin], [], [], 0.01)[0]:
            key += sys.stdin.read(1)
            if select.select([sys.stdin], [], [], 0.01)[0]:
                key += sys.stdin.read(1)
        return key

    def pulse_axis(self, index, value):
        self.axes[index] = value
        self.motion_until[index] = time.monotonic() + 0.18

    def pulse_button(self, index):
        self.buttons[index] = 1

    def handle_key(self, key):
        key_lower = key.lower()
        if key_lower == 'w':
            self.pulse_axis(self.pitch_axis, 1.0)
        elif key_lower == 's':
            self.pulse_axis(self.pitch_axis, -1.0)
        elif key_lower == 'a':
            self.pulse_axis(self.roll_axis, -1.0)
        elif key_lower == 'd':
            self.pulse_axis(self.roll_axis, 1.0)
        elif key_lower == 'q':
            self.pulse_axis(self.yaw_axis, -1.0)
        elif key_lower == 'e':
            self.pulse_axis(self.yaw_axis, 1.0)
        elif key == '\x1b[A':
            self.throttle = min(1.0, self.throttle + 0.1)
        elif key == '\x1b[B':
            self.throttle = max(-1.0, self.throttle - 0.1)
        elif key_lower == 'm':
            self.pulse_button(self.takeover_button)
        elif key_lower == 'g':
            self.pulse_button(self.auto_button)
        elif key == ' ':
            self.pulse_button(self.land_button)

    def tick(self, _event):
        while True:
            key = self.read_key()
            if not key:
                break
            self.handle_key(key)
        now = time.monotonic()
        for index, deadline in list(self.motion_until.items()):
            if now >= deadline:
                self.axes[index] = 0.0
                del self.motion_until[index]
        self.axes[self.throttle_axis] = self.throttle
        msg = Joy(header=Header(stamp=rospy.Time.now()), axes=list(self.axes),
                  buttons=list(self.buttons))
        self.pub.publish(msg)
        self.buttons = [0] * self.button_count


if __name__ == '__main__':
    rospy.init_node('keyboard_joy_node')
    KeyboardJoyNode()
    rospy.spin()
